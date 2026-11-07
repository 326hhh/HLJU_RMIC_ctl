// ===========================================================================
//  gimbal_ctl (Windows) —— 云台/底盘控制客户端，原生 Win32 + Winsock
//
//  职责: 键盘输入 -> 换算指令 -> UDP 发 Jetson -> 显示遥测 + MJPEG 视频
//  与 ../src/main.cpp（Linux/X11 版）功能逐项对齐；线协议直接复用
//  ../gimbal/include/gimbal/link/control_link.hpp（只有一个源头，不复制）。
//
//  编译（Windows 上，或从 Linux 交叉编译）:
//      g++ -std=c++17 -O2 -static main_win.cpp -o gimbal_ctl.exe \
//          -lws2_32 -lgdi32 -luser32 -lwinmm
//  或直接跑 build_win.sh（Linux 上交叉编译）。
//
//  运行:
//      gimbal_ctl.exe                        双击/默认 192.168.2.2
//      gimbal_ctl.exe --host 10.42.0.1       走 Jetson 热点
//      gimbal_ctl.exe --no-video             只要控制，不拉视频
//      gimbal_ctl.exe --selftest             只验 UDP 链路（不开窗口）
//
//  按键（与 Linux 版一致）:
//      ←/→ = 云台 yaw 左/右     ↑/↓ = pitch 抬头/低头（按住转，松开平滑停）
//      空格 = 停手/继续          g = 云台使能开关      j = 底盘使能开关
//      W/S = 底盘前后            A/D = 左右平移        Q/E = 底盘转向
//      F（按住）= 水弹发射       x = 急停（云台+底盘） h = 回中   z = 归零
//      k = 清空当前速率          c = 底盘里程计清零    v = 视频开关
//      R = 小陀螺模式切换        Shift（按一下）= 快速/普通档切换（锁存）
//      Esc = 退出（退出前自动发停手帧）
//
//  ⚠️ Windows 与 X11 的两处关键差异:
//    1) 键盘用 GetAsyncKeyState 轮询物理状态（等价 X11 的 XQueryKeymap），
//       天然免疫 WM_KEYDOWN 的自动重复 —— 长按键不会被刷成连续事件。
//    2) GetAsyncKeyState 是**全局**的：窗口不在前台也照样读得到。
//       所以每拍都查前台窗口，失焦一律按"全松手"处理（见 Keys::setActive），
//       免得你在浏览器里打字把车开走。
//
//  ⚠️ 最小化/失焦时画面不刷新，但控制循环照常跑（指令为 0），
//     不会因为不渲染就断流触发 Jetson 侧看门狗。
// ===========================================================================
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX

#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <mmsystem.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_JPEG
#include "stb_image.h"

#include "gimbal/link/control_link.hpp"

using namespace gimbal;

namespace {

// ---------------------------------------------------------------------------
// 底盘 status 位
//   数值必须与 gimbal/link/chassis_link.hpp 的 chassis_status 一致；
//   那个头文件依赖 <termios.h>/<unistd.h>（POSIX），Windows 端本地重声明。
// ---------------------------------------------------------------------------
namespace chassis_status {
constexpr uint16_t ImuOnline = 0x0001;   ///< 底盘 IMU 在线（小陀螺补偿判据）
}  // namespace chassis_status

constexpr int kHttpPort = 8080;          ///< mjpeg_server 默认端口
constexpr int kLinePx   = 17;            ///< 状态栏一行高度（像素）
constexpr int kRenderMs = 33;            ///< 渲染周期 ≈30 fps

using Clock = std::chrono::steady_clock;

double nowS() {
    return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
}

/// 睡到 dead_line（Windows 的 Sleep 精度靠 timeBeginPeriod(1) 撑到 1~2 ms）
void sleepUntil(Clock::time_point dead_line) {
    for (;;) {
        const auto left = dead_line - Clock::now();
        if (left <= Clock::duration::zero()) return;
        const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(left).count();
        if (ms > 1) ::Sleep(static_cast<DWORD>(ms - 1));
        else        ::Sleep(0);
    }
}

}  // namespace

// ===========================================================================
// 命令行参数
// ===========================================================================
struct Options {
    std::string host      = "192.168.2.2";
    int         ctrl_port = CTRL_DEFAULT_PORT;
    int         http_port = kHttpPort;
    bool        video     = true;
    double      tau       = 0.045;      ///< 平滑时间常数 s
    double      max_rate  = 180.0;      ///< 云台角速度上限 °/s
    double      gimbal_rate = 120.0;    ///< 方向键控制云台的速率 °/s
    int         win_w     = 960;
    int         win_h     = 540;
    double      hz        = 100.0;
    bool        selftest  = false;

    // ---- 底盘（vx 前进 / vy 左移 / wz 逆时针）----
    double      chassis_speed      = 300.0;   ///< mm/s 普通档平移
    double      chassis_turn       = 1500.0;  ///< mrad/s 普通档转向
    double      chassis_speed_fast = 600.0;   ///< mm/s 快速档平移
    double      chassis_turn_fast  = 3000.0;  ///< mrad/s 快速档转向

    // ---- 小陀螺（R 键）：底盘原地自旋 + yaw 反向补偿，云台在世界系站住 ----
    double      spin_rate_mradps = 2000.0;    ///< + = 逆时针，负值 = 反向
    double      spin_gain        = 1.0;       ///< 补偿倍率（方向反了填 -1）

    // ---- 驾驶坐标系：默认「按枪口方向走」，而不是按车头 ----
    bool        gun_relative   = true;   ///< true = W 沿枪口指向；false = 沿车头
    double      gun_offset_deg = 0.0;    ///< 枪口正对车头时 yaw_deg 的值（标定用）
    double      gun_sign       = 1.0;    ///< +1 / -1：A/D 左右反了就填 -1
};

static void usage(const char* a0) {
    std::printf(
        "用法: %s [选项]\n"
        "  --host <ip|名字>        Jetson 地址 (默认 192.168.2.2)\n"
        "  --ctrl-port <n>         UDP 控制口 (默认 %u)\n"
        "  --http-port <n>         MJPEG 端口 (默认 %d)\n"
        "  --no-video              不显示视频\n"
        "  --tau <s>               平滑时间常数 (默认 0.045)\n"
        "  --max-rate <dps>        角速度上限 (默认 180)\n"
        "  --gimbal-rate <dps>     方向键云台速率 (默认 120)\n"
        "  --size <w> <h>          窗口尺寸 (默认 960 540)\n"
        "  --hz <n>                发送频率 (默认 100)\n"
        "  --chassis-speed <mm/s>  底盘平移满速 (默认 300)\n"
        "  --chassis-turn <mrad/s> 底盘转向满速 (默认 1500)\n"
        "  --shift-speed <mm/s>    快速档平移满速 (默认 600)\n"
        "  --shift-turn <mrad/s>   快速档转向满速 (默认 3000)\n"
        "  --spin-rate <mrad/s>    小陀螺自旋速率 (默认 2000 = 2.0 rad/s)\n"
        "  --spin-gain <f>         小陀螺 yaw 补偿倍率 (默认 1.0)\n"
        "  --gun-relative <0|1>    1 = 按枪口方向走（默认），0 = 按车头方向走\n"
        "  --gun-offset <deg>      枪口正对车头时的 yaw 角（标定用，默认 0）\n"
        "  --gun-sign <1|-1>       A/D 左右反了就填 -1\n"
        "  --selftest              只测 UDP 收发，不开窗口\n"
        "  -h, --help\n"
        "\n"
        "  同级目录下的 gimbal_ctl.ini 会先被读取（key=value），命令行覆盖它。\n"
        "  可用 key: host ctrl_port http_port video hz tau max_rate gimbal_rate\n"
        "            size chassis_speed chassis_turn shift_speed shift_turn\n"
        "            spin_rate spin_gain gun_relative gun_offset gun_sign\n",
        a0, CTRL_DEFAULT_PORT, kHttpPort);
}

/// 读 exe 同目录的 gimbal_ctl.ini（双击运行时不方便带参数）
static void loadIni(Options& o) {
    char path[MAX_PATH] = {0};
    ::GetModuleFileNameA(nullptr, path, MAX_PATH);
    std::string ini(path);
    const size_t slash = ini.find_last_of("\\/");
    ini = (slash == std::string::npos) ? std::string("gimbal_ctl.ini")
                                       : ini.substr(0, slash + 1) + "gimbal_ctl.ini";

    FILE* f = std::fopen(ini.c_str(), "r");
    if (!f) return;
    char line[256];
    while (std::fgets(line, sizeof(line), f)) {
        std::string s(line);
        while (!s.empty() && (s.back() == '\n' || s.back() == '\r' || s.back() == ' '))
            s.pop_back();
        if (s.empty() || s[0] == '#' || s[0] == ';' || s[0] == '[') continue;
        const size_t eq = s.find('=');
        if (eq == std::string::npos) continue;
        std::string k = s.substr(0, eq);
        std::string v = s.substr(eq + 1);
        while (!k.empty() && (k.back() == ' ' || k.back() == '\t')) k.pop_back();
        while (!v.empty() && (v.front() == ' ' || v.front() == '\t')) v.erase(v.begin());
        if (k.empty() || v.empty()) continue;

        const double d = std::atof(v.c_str());
        const int    i = std::atoi(v.c_str());
        if      (k == "host")           o.host = v;
        else if (k == "ctrl_port")      o.ctrl_port = i;
        else if (k == "http_port")      o.http_port = i;
        else if (k == "video")          o.video = (i != 0);
        else if (k == "hz")             o.hz = d;
        else if (k == "tau")            o.tau = d;
        else if (k == "max_rate")       o.max_rate = d;
        else if (k == "gimbal_rate")    o.gimbal_rate = d;
        else if (k == "chassis_speed")  o.chassis_speed = d;
        else if (k == "chassis_turn")   o.chassis_turn = d;
        else if (k == "shift_speed")    o.chassis_speed_fast = d;
        else if (k == "shift_turn")     o.chassis_turn_fast = d;
        else if (k == "spin_rate")      o.spin_rate_mradps = d;
        else if (k == "spin_gain")      o.spin_gain = d;
        else if (k == "gun_relative")   o.gun_relative = (i != 0);
        else if (k == "gun_offset")     o.gun_offset_deg = d;
        else if (k == "gun_sign")       o.gun_sign = (d < 0 ? -1.0 : 1.0);
        else if (k == "size") {
            const size_t x = v.find_first_of("xX*");
            if (x != std::string::npos) {
                o.win_w = std::atoi(v.substr(0, x).c_str());
                o.win_h = std::atoi(v.substr(x + 1).c_str());
            }
        }
    }
    std::fclose(f);
    std::printf("[ini] 已读取 %s\n", ini.c_str());
}

// ===========================================================================
// UDP 客户端（Winsock）
// ===========================================================================
class UdpClient {
public:
    ~UdpClient() { close(); }

    bool open(const std::string& host, int port) {
        close();
        addrinfo hints{};
        hints.ai_family   = AF_UNSPEC;
        hints.ai_socktype = SOCK_DGRAM;
        hints.ai_protocol = IPPROTO_UDP;
        addrinfo* res = nullptr;
        const std::string ps = std::to_string(port);
        if (::getaddrinfo(host.c_str(), ps.c_str(), &hints, &res) != 0) return false;

        sock_ = ::socket(res->ai_family, res->ai_socktype, res->ai_protocol);
        if (sock_ == INVALID_SOCKET) {
            ::freeaddrinfo(res);
            return false;
        }
        if (::connect(sock_, res->ai_addr, static_cast<int>(res->ai_addrlen)) != 0) {
            ::freeaddrinfo(res);
            close();
            return false;
        }
        ::freeaddrinfo(res);

        u_long nb = 1;   // 非阻塞收，主循环不被卡住
        ::ioctlsocket(sock_, FIONBIO, &nb);
        return true;
    }

    void close() {
        if (sock_ != INVALID_SOCKET) { ::closesocket(sock_); sock_ = INVALID_SOCKET; }
    }
    bool ok() const { return sock_ != INVALID_SOCKET; }

    void send(const ControlCommand& c) {
        if (sock_ == INVALID_SOCKET) return;
        const auto b = c.encode();
        if (::send(sock_, reinterpret_cast<const char*>(b.data()),
                   static_cast<int>(b.size()), 0) > 0) {
            sent_at_[c.seq % kRing] = nowS();
        }
    }

    /// 收遥测（非阻塞，一次把积压的全收完，取最后一帧）
    bool recv(ControlTelemetry& t) {
        if (sock_ == INVALID_SOCKET) return false;
        bool got = false;
        for (;;) {
            uint8_t buf[512];
            const int n = ::recv(sock_, reinterpret_cast<char*>(buf),
                                 static_cast<int>(sizeof(buf)), 0);
            if (n <= 0) break;
            ControlTelemetry u;
            if (!ControlTelemetry::decode(buf, static_cast<size_t>(n), u)) {
                ++bad_;
                continue;
            }
            const double t0 = sent_at_[u.echo_seq % kRing];
            if (t0 > 0.0) rtt_ms_ = (nowS() - t0) * 1000.0;
            t   = u;
            got = true;
        }
        return got;
    }

    uint64_t bad() const { return bad_; }
    double   rttMs() const { return rtt_ms_; }

private:
    static constexpr size_t kRing = 256;
    SOCKET   sock_ = INVALID_SOCKET;
    double   sent_at_[kRing] = {0};
    uint64_t bad_  = 0;
    double   rtt_ms_ = -1.0;
};

// ===========================================================================
// MJPEG 拉流 + stb_image 解码（独立线程，主循环只取最新帧）
//
//  为什么用 stb_image 而不是 libjpeg：Windows 上零依赖，单头文件即可，
//  UVC 摄像头（Jetson 侧 mjpeg_server 直出 V4L2 MJPG）都是 baseline JPEG，
//  stb_image 够用；遇到 progressive JPEG 会解码失败（日志会打出来）。
// ===========================================================================
class MjpegView {
public:
    struct Frame {
        int                  width  = 0;
        int                  height = 0;
        std::vector<uint8_t> bgra;    ///< 32bpp BGRA，行序自上而下
        uint64_t             seq    = 0;
    };

    ~MjpegView() { stop(); }

    bool start(const std::string& host, int port, const std::string& path = "/stream") {
        stop();
        host_ = host;
        port_ = port;
        path_ = path;
        run_  = true;
        th_   = std::thread(&MjpegView::rxLoop, this);
        return true;
    }

    void stop() {
        run_ = false;
        const SOCKET s = sock_.load();
        if (s != INVALID_SOCKET) ::shutdown(s, SD_BOTH);   // 让阻塞的 recv 立刻返回
        if (th_.joinable()) th_.join();
        connected_ = false;
    }

    /// 取最新一帧（拷贝）；没有新帧返回 false
    bool getLatest(Frame& out, uint64_t& last_seq) {
        std::lock_guard<std::mutex> lk(mu_);
        if (seq_ == last_seq || latest_.bgra.empty()) return false;
        out      = latest_;
        last_seq = seq_;
        return true;
    }

    bool        connected() const { return connected_.load(); }
    double      fps() const { return fps_.load(); }
    std::string lastError() const {
        std::lock_guard<std::mutex> lk(mu_);
        return err_;
    }

private:
    /// 非阻塞 connect + select（阻塞 connect 到不可达主机会卡 20 s，窗口关不掉）
    static SOCKET connectTcp(const std::string& host, int port, std::string& err) {
        addrinfo hints{};
        hints.ai_family   = AF_UNSPEC;
        hints.ai_socktype = SOCK_STREAM;
        hints.ai_protocol = IPPROTO_TCP;

        addrinfo* res = nullptr;
        const std::string ps = std::to_string(port);
        if (::getaddrinfo(host.c_str(), ps.c_str(), &hints, &res) != 0) {
            err = "DNS 解析失败: " + host;
            return INVALID_SOCKET;
        }

        SOCKET s = INVALID_SOCKET;
        for (addrinfo* p = res; p; p = p->ai_next) {
            s = ::socket(p->ai_family, p->ai_socktype, p->ai_protocol);
            if (s == INVALID_SOCKET) continue;

            u_long nb = 1;
            ::ioctlsocket(s, FIONBIO, &nb);
            int rc = ::connect(s, p->ai_addr, static_cast<int>(p->ai_addrlen));
            if (rc == SOCKET_ERROR && ::WSAGetLastError() == WSAEWOULDBLOCK) {
                fd_set wset;
                FD_ZERO(&wset);
                FD_SET(s, &wset);
                timeval tv = {2, 0};
                rc = ::select(0, nullptr, &wset, nullptr, &tv) > 0 ? 0 : SOCKET_ERROR;
                if (rc == 0) {
                    int so_err = 0;
                    int len = static_cast<int>(sizeof(so_err));
                    ::getsockopt(s, SOL_SOCKET, SO_ERROR,
                                 reinterpret_cast<char*>(&so_err), &len);
                    if (so_err != 0) rc = SOCKET_ERROR;
                }
            }
            if (rc == 0) {
                u_long blk = 0;   // 还原成阻塞模式
                ::ioctlsocket(s, FIONBIO, &blk);
                break;
            }
            ::closesocket(s);
            s = INVALID_SOCKET;
        }
        ::freeaddrinfo(res);
        if (s == INVALID_SOCKET) err = "连接 " + host + ":" + ps + " 失败";
        return s;
    }

    static bool decodeJpeg(const uint8_t* data, size_t len, Frame& out) {
        int w = 0, h = 0, comp = 0;
        stbi_uc* px = stbi_load_from_memory(data, static_cast<int>(len), &w, &h, &comp, 3);
        if (!px) return false;
        if (w <= 0 || h <= 0 || w > 4096 || h > 4096) {
            stbi_image_free(px);
            return false;
        }
        out.width  = w;
        out.height = h;
        out.bgra.assign(static_cast<size_t>(w) * static_cast<size_t>(h) * 4, 255);
        uint8_t* dst = out.bgra.data();
        const size_t n = static_cast<size_t>(w) * static_cast<size_t>(h);
        for (size_t i = 0; i < n; ++i) {
            dst[i * 4 + 0] = px[i * 3 + 2];   // B
            dst[i * 4 + 1] = px[i * 3 + 1];   // G
            dst[i * 4 + 2] = px[i * 3 + 0];   // R
            dst[i * 4 + 3] = 255;
        }
        stbi_image_free(px);
        return true;
    }

    void rxLoop() {
        while (run_) {
            std::string err;
            const SOCKET fd = connectTcp(host_, port_, err);
            if (fd == INVALID_SOCKET) {
                {
                    std::lock_guard<std::mutex> lk(mu_);
                    err_ = err;
                }
                connected_ = false;
                for (int i = 0; i < 20 && run_; ++i)
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                continue;
            }
            sock_ = fd;

            int one = 1;
            ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY,
                         reinterpret_cast<const char*>(&one), sizeof(one));
            DWORD tmo = 1000;   // 1 s 收不到东西就回到 run_ 检查，退出能及时响应
            ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO,
                         reinterpret_cast<const char*>(&tmo), sizeof(tmo));

            const std::string req = "GET " + path_ + " HTTP/1.0\r\nHost: " + host_ +
                                    "\r\nConnection: close\r\n\r\n";
            if (::send(fd, req.data(), static_cast<int>(req.size()), 0) == SOCKET_ERROR) {
                if (sock_.exchange(INVALID_SOCKET) == fd) ::closesocket(fd);
                continue;
            }
            {
                std::lock_guard<std::mutex> lk(mu_);
                err_.clear();
            }
            connected_ = true;
            std::printf("[mjpeg] 已连接 %s:%d%s\n", host_.c_str(), port_, path_.c_str());

            std::vector<uint8_t> buf;
            buf.reserve(1 << 20);
            uint8_t  chunk[16384];
            uint64_t frame_count = 0;
            size_t   total_rx    = 0;
            auto     t_win       = Clock::now();
            size_t   hdr_done    = 0;

            while (run_) {
                const int n = ::recv(fd, reinterpret_cast<char*>(chunk),
                                     static_cast<int>(sizeof(chunk)), 0);
                if (n <= 0) {
                    if (n == SOCKET_ERROR &&
                        ::WSAGetLastError() == WSAETIMEDOUT) continue;   // 超时不算断线
                    break;
                }
                total_rx += static_cast<size_t>(n);
                buf.insert(buf.end(), chunk, chunk + n);

                // 跳过 HTTP 响应头
                if (hdr_done == 0) {
                    for (size_t i = 0; i + 4 <= buf.size(); ++i) {
                        if (std::memcmp(&buf[i], "\r\n\r\n", 4) == 0) {
                            hdr_done = i + 4;
                            break;
                        }
                    }
                    if (hdr_done == 0) continue;
                    buf.erase(buf.begin(), buf.begin() + static_cast<std::ptrdiff_t>(hdr_done));
                }

                // 按 SOI(FFD8) ... EOI(FFD9) 切帧
                for (;;) {
                    size_t soi = 0, eoi = 0;
                    bool   ok  = false;
                    for (size_t i = 0; i + 1 < buf.size(); ++i) {
                        if (buf[i] == 0xFF && buf[i + 1] == 0xD8) {
                            for (size_t j = i + 2; j + 1 < buf.size(); ++j) {
                                if (buf[j] == 0xFF && buf[j + 1] == 0xD9) {
                                    soi = i;
                                    eoi = j + 2;
                                    ok  = true;
                                    break;
                                }
                            }
                            break;   // 只有 SOI 没 EOI：这帧还没收全
                        }
                    }
                    if (!ok) {
                        size_t cut = 0;   // 丢掉 SOI 前的填充，别让缓冲无限涨
                        while (cut + 1 < buf.size() &&
                               !(buf[cut] == 0xFF && buf[cut + 1] == 0xD8))
                            ++cut;
                        if (cut > 0)
                            buf.erase(buf.begin(), buf.begin() + static_cast<std::ptrdiff_t>(cut));
                        break;
                    }

                    Frame f;
                    if (decodeJpeg(buf.data() + soi, eoi - soi, f)) {
                        std::lock_guard<std::mutex> lk(mu_);
                        latest_ = std::move(f);
                        ++seq_;
                        ++frame_count;
                        if (seq_ <= 2 || seq_ % 200 == 0) {
                            std::printf("[mjpeg] 第 %llu 帧 %dx%d (%zu 字节 JPEG)\n",
                                        static_cast<unsigned long long>(seq_), latest_.width,
                                        latest_.height, eoi - soi);
                        }
                    } else {
                        std::printf("[mjpeg] JPEG 解码失败 (%zu 字节，progressive 不支持)\n",
                                    eoi - soi);
                    }
                    buf.erase(buf.begin(), buf.begin() + static_cast<std::ptrdiff_t>(eoi));

                    const auto nowT = Clock::now();
                    const double dt = std::chrono::duration<double>(nowT - t_win).count();
                    if (dt >= 1.0) {
                        fps_        = frame_count / dt;
                        frame_count = 0;
                        t_win       = nowT;
                    }
                }

                if (buf.size() > (4u << 20)) buf.clear();   // 异常保护
            }

            std::printf("[mjpeg] 断开 (共收 %zu 字节)\n", total_rx);
            if (sock_.exchange(INVALID_SOCKET) == fd) ::closesocket(fd);
            connected_ = false;
            if (run_)
                for (int i = 0; i < 10 && run_; ++i)
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    std::thread         th_;
    std::atomic<bool>   run_{false};
    std::atomic<bool>   connected_{false};
    std::atomic<double> fps_{0.0};
    std::atomic<SOCKET> sock_{INVALID_SOCKET};
    mutable std::mutex  mu_;
    Frame               latest_;
    uint64_t            seq_ = 0;
    std::string         err_;
    std::string         host_;
    int                 port_ = kHttpPort;
    std::string         path_ = "/stream";
};

// ===========================================================================
// 键盘：轮询物理状态 + 上升沿检测
// ===========================================================================
class Keys {
public:
    /// 失焦时全部按"松手"处理（GetAsyncKeyState 是全局的，不放行会串键）
    void setActive(bool active) {
        if (active_ && !active) prev_.fill(false);   // 失焦时清边沿历史
        active_ = active;
    }

    bool down(int vk) {
        if (!active_) return false;
        return (::GetAsyncKeyState(vk) & 0x8000) != 0;
    }

    /// 只在"刚按下"那一拍返回 true（长按不重复触发）
    bool edge(int vk) {
        const bool n = down(vk);
        const bool e = n && !prev_[static_cast<size_t>(vk & 0xFF)];
        prev_[static_cast<size_t>(vk & 0xFF)] = n;
        return e;
    }

private:
    bool             active_ = true;
    std::array<bool, 256> prev_{};
};

// ===========================================================================
// GDI 渲染（双缓冲：内存 DC 画完再 BitBlt，避免闪烁）
// ===========================================================================
class Renderer {
public:
    ~Renderer() { destroy(); }

    bool init(HWND hwnd) {
        hwnd_ = hwnd;
        release();                       // 先把早到的 WM_SIZE 建的位图卸掉
        if (mem_dc_) { ::DeleteDC(mem_dc_); mem_dc_ = nullptr; }
        width_ = height_ = 0;
        font_ = ::CreateFontA(-15, 0, 0, 0, FW_SEMIBOLD, FALSE, FALSE, FALSE,
                              DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
                              CLEARTYPE_QUALITY, FIXED_PITCH | FF_MODERN, "Consolas");
        RECT rc = {0, 0, 0, 0};
        if (::GetClientRect(hwnd, &rc)) resize(rc.right, rc.bottom);
        return true;
    }

    bool ready() const { return mem_dc_ != nullptr; }

    void destroy() {
        release();
        if (font_) { ::DeleteObject(font_); font_ = nullptr; }
        if (mem_dc_) { ::DeleteDC(mem_dc_); mem_dc_ = nullptr; }
    }

    void resize(int w, int h) {
        if (w == width_ && h == height_) return;
        release();
        width_  = std::max(1, w);
        height_ = std::max(1, h);
        HDC screen = ::GetDC(hwnd_);
        mem_dc_ = ::CreateCompatibleDC(screen);
        bmp_    = ::CreateCompatibleBitmap(screen, width_, height_);
        old_bmp_ = ::SelectObject(mem_dc_, bmp_);
        ::ReleaseDC(hwnd_, screen);
    }

    int width() const { return width_; }
    int height() const { return height_; }

    void begin() {
        if (!mem_dc_) return;
        RECT r = {0, 0, width_, height_};
        ::FillRect(mem_dc_, &r, static_cast<HBRUSH>(::GetStockObject(BLACK_BRUSH)));
    }

    /// 等比缩放居中贴画面（窗口比画面小/大都不会变形）
    void drawVideo(const MjpegView::Frame& f) {
        if (!mem_dc_ || f.width <= 0 || f.height <= 0 || f.bgra.empty()) return;
        const int margin = 4;
        const double sx = static_cast<double>(width_ - 2 * margin) / f.width;
        const double sy = static_cast<double>(height_ - 2 * margin) / f.height;
        const double s  = std::min(sx, sy);
        if (!(s > 0.0)) return;
        const int dw = std::max(1, static_cast<int>(f.width * s + 0.5));
        const int dh = std::max(1, static_cast<int>(f.height * s + 0.5));
        const int ox = (width_ - dw) / 2;
        const int oy = (height_ - dh) / 2;

        BITMAPINFO bi{};
        bi.bmiHeader.biSize        = sizeof(BITMAPINFOHEADER);
        bi.bmiHeader.biWidth       = f.width;
        bi.bmiHeader.biHeight      = -f.height;   // 负 = 自上而下
        bi.bmiHeader.biPlanes      = 1;
        bi.bmiHeader.biBitCount    = 32;
        bi.bmiHeader.biCompression = BI_RGB;

        ::SetStretchBltMode(mem_dc_, COLORONCOLOR);
        ::StretchDIBits(mem_dc_, ox, oy, dw, dh, 0, 0, f.width, f.height, f.bgra.data(),
                        &bi, DIB_RGB_COLORS, SRCCOPY);
    }

    /// 状态文字（透明背景 + 1 px 黑阴影，压在视频上也读得清）
    void text(int x, int y, const char* s, COLORREF color = RGB(235, 235, 235)) {
        if (!mem_dc_ || !s || !*s) return;
        ::SelectObject(mem_dc_, font_);
        ::SetBkMode(mem_dc_, TRANSPARENT);
        const int len = static_cast<int>(std::strlen(s));
        ::SetTextColor(mem_dc_, RGB(0, 0, 0));
        ::TextOutA(mem_dc_, x + 1, y + 1, s, len);
        ::SetTextColor(mem_dc_, color);
        ::TextOutA(mem_dc_, x, y, s, len);
    }

    /// 瞄准十字（正中央，半透明观感靠暗色描边）
    void reticle() {
        if (!mem_dc_) return;
        const int cx = width_ / 2;
        const int cy = height_ / 2;
        const int r  = 16;
        HPEN pen = ::CreatePen(PS_SOLID, 1, RGB(120, 255, 120));
        HGDIOBJ old = ::SelectObject(mem_dc_, pen);
        // 上下左右四段 + 中心点，避免十字盖住目标
        ::MoveToEx(mem_dc_, cx - r, cy, nullptr); ::LineTo(mem_dc_, cx - 5, cy);
        ::MoveToEx(mem_dc_, cx + 5, cy, nullptr); ::LineTo(mem_dc_, cx + r, cy);
        ::MoveToEx(mem_dc_, cx, cy - r, nullptr); ::LineTo(mem_dc_, cx, cy - 5);
        ::MoveToEx(mem_dc_, cx, cy + 5, nullptr); ::LineTo(mem_dc_, cx, cy + r);
        ::Ellipse(mem_dc_, cx - 2, cy - 2, cx + 2, cy + 2);
        ::SelectObject(mem_dc_, old);
        ::DeleteObject(pen);
    }

    void end() {
        if (!mem_dc_) return;
        HDC dc = ::GetDC(hwnd_);
        ::BitBlt(dc, 0, 0, width_, height_, mem_dc_, 0, 0, SRCCOPY);
        ::ReleaseDC(hwnd_, dc);
    }

    /// WM_PAINT 里把上一帧原样贴回去（不重画内容，避免和主循环打架）
    void blitBackBuffer(HDC dc) {
        if (mem_dc_) ::BitBlt(dc, 0, 0, width_, height_, mem_dc_, 0, 0, SRCCOPY);
    }

private:
    void release() {
        if (mem_dc_ && old_bmp_) ::SelectObject(mem_dc_, old_bmp_);
        old_bmp_ = nullptr;
        if (bmp_) { ::DeleteObject(bmp_); bmp_ = nullptr; }
    }

    HWND     hwnd_    = nullptr;
    HDC      mem_dc_  = nullptr;
    HBITMAP  bmp_     = nullptr;
    HGDIOBJ  old_bmp_ = nullptr;
    HFONT    font_    = nullptr;
    int      width_   = 0;
    int      height_  = 0;
};

// ===========================================================================
// 自检（不开窗口，只验协议与 UDP 链路）
// ===========================================================================
static int runSelftest(const Options& o) {
    std::printf("=== gimbal_ctl (Windows) 自检 ===\n");
    int fail = 0;
    auto CHECK = [&](bool ok, const char* m) {
        std::printf("  [%s] %s\n", ok ? " OK " : "FAIL", m);
        if (!ok) ++fail;
    };

    ControlCommand c;
    c.seq = 42;
    c.yaw_dps = 45.0;
    c.pitch_dps = -30.0;
    c.flags = ctrl_flag::Enable;
    const auto b = c.encode();
    ControlCommand d;
    CHECK(ControlCommand::decode(b.data(), b.size(), d), "上行编解码");
    CHECK(std::fabs(d.yaw_dps - 45.0) < 1e-9 && std::fabs(d.pitch_dps + 30.0) < 1e-9,
          "角速度往返");

    ControlCommand cs;
    cs.seq = 9;
    cs.chassis_vx_mmps   = 500.0;
    cs.chassis_vy_mmps   = -250.0;
    cs.chassis_wz_mradps = 1200.0;
    cs.chassis_flags     = chs_flag::Enable;
    const auto cb = cs.encode();
    CHECK(cb.size() == CTRL_UP_LEN, "上行包长 24 字节（v2）");
    ControlCommand cd;
    CHECK(ControlCommand::decode(cb.data(), cb.size(), cd), "上行 v2 解码");
    CHECK(cd.chassis_vx_mmps == 500.0 && cd.chassis_vy_mmps == -250.0 &&
              cd.chassis_wz_mradps == 1200.0 && cd.chassisEnable(),
          "底盘字段往返一致");
    auto cbad = cb;
    cbad[16] = static_cast<uint8_t>(cbad[16] ^ 0xFF);
    CHECK(!ControlCommand::decode(cbad.data(), cbad.size(), cd), "底盘字段篡改被拒");

    // 下行遥测：造一帧再解回来（验证 Windows 端字节偏移/字节序没错）
    ControlTelemetry tm;
    tm.echo_seq = 7;
    tm.yaw_dps = 12.5;
    tm.chassis_x_mm = -1234.0;
    tm.chassis_wheel_rpm[2] = 88.25;
    tm.chassis_valid = true;
    const auto db = tm.encode();
    ControlTelemetry td;
    CHECK(db.size() == CTRL_DOWN_LEN, "下行包长 64 字节");
    CHECK(ControlTelemetry::decode(db.data(), db.size(), td), "下行解码");
    CHECK(td.echo_seq == 7 && std::fabs(td.yaw_dps - 12.5) < 1e-6 &&
              td.chassis_x_mm == -1234.0 && std::fabs(td.chassis_wheel_rpm[2] - 88.25) < 1e-6,
          "遥测字段往返一致");

    UdpClient u;
    const bool opened = u.open(o.host, o.ctrl_port);
    std::printf("  [%s] UDP socket -> %s:%d\n", opened ? " OK " : "FAIL",
                o.host.c_str(), o.ctrl_port);
    if (!opened) return 1;

    std::printf("  发 50 个包并等 1 秒看回包…\n");
    for (uint32_t i = 1; i <= 50; ++i) {
        ControlCommand s;
        s.seq = i;
        s.flags = 0;   // 不使能，纯探路
        u.send(s);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    ControlTelemetry t;
    bool  got = false;
    const double t0 = nowS();
    while (nowS() - t0 < 1.0 && !got) {
        got = u.recv(t);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    if (got) {
        std::printf("  收到遥测: echo_seq=%u yaw=%+.2f°/s pitch=%+.2f°/s "
                    "角度 %+.2f/%+.2f° 母线 %dmV %d°C status=0x%02X\n",
                    t.echo_seq, t.yaw_dps, t.pitch_dps, t.yaw_deg, t.pitch_deg,
                    t.vbus_mv, t.temp_c, t.status);
    }
    CHECK(got, "收到 Jetson 遥测回包（没收到 = Jetson 上 gimbal 没跑）");
    CHECK(u.bad() == 0, "无坏包");
    std::printf("\n%s（失败 %d 项）\n", fail ? "有失败" : "全部通过", fail);
    return fail ? 1 : 0;
}

// ===========================================================================
// 主程序
// ===========================================================================
class App {
public:
    explicit App(const Options& o) : o_(o) {}

    int run();

    static LRESULT CALLBACK wndProcStatic(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp) {
        App* self = reinterpret_cast<App*>(::GetWindowLongPtrA(hwnd, GWLP_USERDATA));
        if (self) return self->wndProc(hwnd, msg, wp, lp);
        return ::DefWindowProcA(hwnd, msg, wp, lp);
    }

private:
    LRESULT wndProc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp);
    bool    renderFrame(const std::vector<std::pair<std::string, uint32_t>>& lines);
    bool    createWindow();
    void    shutdown();

    Options        o_;
    HWND           hwnd_ = nullptr;
    Renderer       rnd_;
    Keys           keys_;
    UdpClient      udp_;
    MjpegView      view_;
    volatile bool  running_ = true;
    uint64_t       shown_seq_ = 0;
};

// 控制台 Ctrl+C 优雅停手（别让云台保持上一拍速率）
static volatile LONG g_ctrl_c = 0;
static BOOL WINAPI consoleCtrlHandler(DWORD type) {
    if (type == CTRL_C_EVENT || type == CTRL_BREAK_EVENT ||
        type == CTRL_CLOSE_EVENT || type == CTRL_SHUTDOWN_EVENT) {
        ::InterlockedExchange(&g_ctrl_c, 1);
        return TRUE;
    }
    return FALSE;
}

bool App::createWindow() {
    WNDCLASSEXA wc{};
    wc.cbSize        = sizeof(WNDCLASSEXA);
    wc.style         = CS_HREDRAW | CS_VREDRAW;
    wc.lpfnWndProc   = &App::wndProcStatic;
    wc.hInstance     = ::GetModuleHandleA(nullptr);
    wc.hCursor       = ::LoadCursorA(nullptr, IDC_ARROW);
    wc.hbrBackground = static_cast<HBRUSH>(::GetStockObject(BLACK_BRUSH));
    wc.lpszClassName = "gimbal_ctl_win";
    if (!::RegisterClassExA(&wc) && ::GetLastError() != ERROR_CLASS_ALREADY_EXISTS) {
        std::printf("RegisterClassEx 失败 (%lu)\n", ::GetLastError());
        return false;
    }

    RECT r = {0, 0, o_.win_w, o_.win_h};
    ::AdjustWindowRect(&r, WS_OVERLAPPEDWINDOW, FALSE);
    hwnd_ = ::CreateWindowExA(0, wc.lpszClassName, "gimbal_ctl (win)",
                              WS_OVERLAPPEDWINDOW, CW_USEDEFAULT, CW_USEDEFAULT,
                              r.right - r.left, r.bottom - r.top, nullptr, nullptr,
                              wc.hInstance, this);
    if (!hwnd_) {
        std::printf("CreateWindow 失败 (%lu)\n", ::GetLastError());
        return false;
    }
    ::SetWindowLongPtrA(hwnd_, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(this));
    ::ShowWindow(hwnd_, SW_SHOW);
    ::UpdateWindow(hwnd_);
    return true;
}

LRESULT App::wndProc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp) {
    switch (msg) {
        case WM_SIZE: {
            rnd_.resize(LOWORD(lp), HIWORD(lp));
            return 0;
        }
        case WM_ERASEBKGND:
            return 1;   // 自己双缓冲刷，别让系统擦背景（会闪）
        case WM_KEYDOWN:
            if (wp == VK_ESCAPE) running_ = false;
            return 0;
        case WM_CLOSE:
            running_ = false;
            return 0;
        case WM_DESTROY:
            running_ = false;
            ::PostQuitMessage(0);
            return 0;
        case WM_PAINT: {
            PAINTSTRUCT ps;
            HDC dc = ::BeginPaint(hwnd, &ps);
            if (rnd_.ready()) {
                rnd_.blitBackBuffer(dc);
            } else {
                ::FillRect(dc, &ps.rcPaint,
                           static_cast<HBRUSH>(::GetStockObject(BLACK_BRUSH)));
            }
            ::EndPaint(hwnd, &ps);
            return 0;
        }
        default:
            break;
    }
    return ::DefWindowProcA(hwnd, msg, wp, lp);
}

/// 画一帧：视频打底 → 十字 → 状态文字（全部画在同一个双缓冲 DC 上，没有闪烁）
bool App::renderFrame(const std::vector<std::pair<std::string, uint32_t>>& lines) {
    bool new_frame = false;
    rnd_.begin();
    if (o_.video) {
        MjpegView::Frame f;
        if (view_.getLatest(f, shown_seq_) && f.width > 0) {
            rnd_.drawVideo(f);
            new_frame = true;
        }
    }
    rnd_.reticle();

    int y = 8;
    for (const auto& ln : lines) {
        rnd_.text(10, y, ln.first.c_str(), static_cast<COLORREF>(ln.second));
        y += kLinePx;
    }
    rnd_.end();
    return new_frame;
}

void App::shutdown() {
    view_.stop();
    rnd_.destroy();
    if (hwnd_) {
        ::DestroyWindow(hwnd_);
        hwnd_ = nullptr;
    }
}

int App::run() {
    if (!createWindow()) return 1;
    rnd_.init(hwnd_);

    if (!udp_.open(o_.host, o_.ctrl_port)) {
        std::printf("无法建立到 %s:%d 的 UDP 通道\n", o_.host.c_str(), o_.ctrl_port);
        shutdown();
        return 1;
    }
    if (o_.video) {
        view_.start(o_.host, o_.http_port);
        std::printf("[video] 拉流 http://%s:%d/stream\n", o_.host.c_str(), o_.http_port);
    }
    ::SetConsoleCtrlHandler(consoleCtrlHandler, TRUE);

    std::printf("========================================================\n");
    std::printf("  gimbal_ctl (Windows) 云台/底盘控制\n");
    std::printf("  目标    : %s  UDP:%d  MJPEG:%d\n", o_.host.c_str(), o_.ctrl_port,
                o_.http_port);
    std::printf("  云台    : 方向键速率 %.0f°/s   平滑 tau=%.0fms   上限 %.0f°/s\n",
                o_.gimbal_rate, o_.tau * 1000, o_.max_rate);
    std::printf("  按键    : ↑/↓=云台俯仰  ←/→=云台左右  空格=停手  g=云台使能  j=底盘使能\n");
    std::printf("            W/S=前后  A/D=左右  Q/E=底盘转向  F=按住发射  x=急停  h=回中\n");
    std::printf("            z=归零  k=清速率  c=底盘清零  v=视频  R=小陀螺  Shift=快速档\n");
    std::printf("            Esc=退出（退出前自动发停手帧）\n");
    std::printf("  底盘    : 普通档 平移 %.0f mm/s  转向 %.0f mrad/s\n",
                o_.chassis_speed, o_.chassis_turn);
    std::printf("            快速档 平移 %.0f mm/s  转向 %.0f mrad/s\n",
                o_.chassis_speed_fast, o_.chassis_turn_fast);
    std::printf("  小陀螺  : 自旋 %+.2f rad/s（%s）  云台补偿倍率 %+.2f\n",
                o_.spin_rate_mradps / 1000.0,
                o_.spin_rate_mradps >= 0 ? "逆时针" : "顺时针", o_.spin_gain);
    std::printf("  驾驶系  : %s   gun_offset=%.2f°  gun_sign=%+.0f\n",
                o_.gun_relative ? "枪口方向（W 沿枪口指向走）" : "车体方向（W 沿车头走）",
                o_.gun_offset_deg, o_.gun_sign);
    std::printf("            T = 切换驾驶系   Y = 把当前枪口方向设成正前\n");
    std::printf("========================================================\n");

    // ---- 控制状态（与 Linux 版一一对应）----
    ControlTelemetry last_telem;
    bool     have_telem = false;
    double   rate[2]    = {0.0, 0.0};          // yaw / pitch 平滑后 °/s
    uint32_t seq        = 0;
    bool     hold_mode  = false;
    bool     en_state   = true;
    uint8_t  oneshot    = 0;
    bool     fire_hold  = false;

    bool     chassis_en        = true;
    bool     chassis_estop     = false;
    bool     chassis_fast      = false;        // Shift 锁存
    bool     spin_mode         = false;        // R 小陀螺
    double   spin_wz_lp        = 0.0;
    double   spin_comp_until   = 0.0;
    double   t_last_telem      = -1e9;
    bool     oneshot_chassis_clear = false;
    double   cvx = 0.0, cvy = 0.0, cwz = 0.0;  // 平滑后 mm/s、mrad/s（输入坐标系）
    bool     gun_drive     = o_.gun_relative;  // T 键切换：枪口系 / 车体系
    double   gun_off_deg   = o_.gun_offset_deg;// Y 键标定：这个 yaw 角 = 正前
    double   gun_theta     = 0.0;              // 本拍实际用的枪口-车体夹角（°）
    bool     gun_fellback  = false;            // 没遥测时退回车体系
    double   lost_pkts         = 0.0;
    uint32_t last_echo         = 0;
    uint64_t video_frames      = 0;

    // COLORREF = 0x00BBGGRR
    const uint32_t kWhite = 0x00EBEBEBu;
    const uint32_t kWarn  = 0x006060FFu;   // RGB(255, 96, 96)
    const uint32_t kGood  = 0x0080FF80u;   // RGB(128, 255, 128)

    const double dt_target = 1.0 / o_.hz;
    auto t_next   = Clock::now();
    auto t_render = Clock::now();
    auto t_vstat  = Clock::now();
    std::vector<std::pair<std::string, uint32_t>> lines;
    lines.reserve(24);

    while (running_ && !g_ctrl_c) {
        // ---------- 1. 窗口消息 ----------
        MSG msg;
        while (::PeekMessageA(&msg, nullptr, 0, 0, PM_REMOVE)) {
            if (msg.message == WM_QUIT) {
                running_ = false;
                break;
            }
            ::TranslateMessage(&msg);
            ::DispatchMessageA(&msg);
        }
        if (!running_ || g_ctrl_c) break;

        // ---------- 1b. 前台判断（失焦 = 全松手）----------
        keys_.setActive(::GetForegroundWindow() == hwnd_);

        // ---------- 2. 一次性按键（上升沿，长按不重复）----------
        if (keys_.edge(VK_ESCAPE)) running_ = false;
        if (keys_.edge(VK_SPACE))  hold_mode = !hold_mode;

        if (keys_.edge('G')) {
            en_state = !en_state;
            oneshot |= ctrl_flag::Enable;    // 让 Jetson 重新授权/恢复
            std::printf("[gimbal] 使能=%d\n", en_state ? 1 : 0);
        }
        if (keys_.edge('J')) {
            chassis_en = !chassis_en;
            if (chassis_en) chassis_estop = false;
            std::printf("[chassis] 使能=%d\n", chassis_en ? 1 : 0);
        }
        if (keys_.edge('C')) oneshot_chassis_clear = true;
        if (keys_.edge('X')) {
            oneshot |= ctrl_flag::EStop;
            chassis_estop = true;
            std::printf("[chassis] 急停（按 j 恢复底盘）\n");
        }
        if (keys_.edge('H')) oneshot |= ctrl_flag::Home;
        if (keys_.edge('Z')) {
            oneshot |= ctrl_flag::Zero;
            // 云台归零同时也把「枪口正前」标定复位：z 的语义就是"现在这里是正前"
            gun_off_deg = 0.0;
            std::printf("[drive] 云台归零，同时把 gun_offset 复位为 0\n");
        }
        if (keys_.edge('K')) rate[0] = rate[1] = 0.0;
        if (keys_.edge('R')) {
            spin_mode  = !spin_mode;
            spin_wz_lp = 0.0;
            // 关掉时底盘还要约 0.33 s 才停转，这段时间继续补偿，相机不会被甩
            if (!spin_mode) spin_comp_until = nowS() + 0.6;
            std::printf("[chassis] 小陀螺 %s（自旋 %+.2f rad/s）\n",
                        spin_mode ? "开" : "关", o_.spin_rate_mradps / 1000.0);
        }
        if (keys_.edge('V')) {
            o_.video = !o_.video;
            if (o_.video) view_.start(o_.host, o_.http_port);
            else          view_.stop();
            std::printf("[video] %s\n", o_.video ? "开" : "关");
        }
        if (keys_.edge('T')) {
            gun_drive = !gun_drive;
            std::printf("[drive] %s\n", gun_drive
                        ? "枪口方向：W = 枪口指向哪就走哪（A/D = 枪口左右侧）"
                        : "车体方向：W = 车头指向");
        }
        if (keys_.edge('Y')) {
            if (have_telem) {
                gun_off_deg = last_telem.yaw_deg;
                std::printf("[drive] 已把当前枪口方向设成\"正前方\"（gun_offset=%.2f°，"
                            "想固定就写进 ini 的 gun_offset）\n", gun_off_deg);
            } else {
                std::printf("[drive] 没有遥测，标定不了，先等 Jetson 连上\n");
            }
        }
        if (keys_.edge(VK_SHIFT)) {
            chassis_fast = !chassis_fast;
            std::printf("[chassis] 切到%s档：平移 %.0f mm/s / 转向 %.0f mrad/s\n",
                        chassis_fast ? "快速" : "普通",
                        chassis_fast ? o_.chassis_speed_fast : o_.chassis_speed,
                        chassis_fast ? o_.chassis_turn_fast  : o_.chassis_turn);
        }

        // ---------- 2b. 发射扳机：按住 F（HOLD 时不响应）----------
        fire_hold = !hold_mode && keys_.down('F');

        // ---------- 3. 云台：方向键角速度 ----------
        double target_yaw   = 0.0;
        double target_pitch = 0.0;
        if (!hold_mode) {
            if (keys_.down(VK_LEFT))  target_yaw   += o_.gimbal_rate;
            if (keys_.down(VK_RIGHT)) target_yaw   -= o_.gimbal_rate;
            if (keys_.down(VK_UP))    target_pitch += o_.gimbal_rate;
            if (keys_.down(VK_DOWN))  target_pitch -= o_.gimbal_rate;
        }
        target_yaw   = std::max(-o_.max_rate, std::min(o_.max_rate, target_yaw));
        target_pitch = std::max(-o_.max_rate, std::min(o_.max_rate, target_pitch));

        const double a = std::min(1.0, dt_target / std::max(1e-3, o_.tau));
        rate[0] += (target_yaw   - rate[0]) * a;
        rate[1] += (target_pitch - rate[1]) * a;

        // ---------- 3b. 底盘：键盘 -> 平滑限幅 ----------
        const double spd  = chassis_fast ? o_.chassis_speed_fast : o_.chassis_speed;
        const double turn = chassis_fast ? o_.chassis_turn_fast  : o_.chassis_turn;

        double tvx = 0.0, tvy = 0.0, twz = 0.0;
        if (!hold_mode && !chassis_estop) {
            if (keys_.down('W')) tvx += spd;
            if (keys_.down('S')) tvx -= spd;
            if (keys_.down('A')) tvy += spd;          // +y = 左移
            if (keys_.down('D')) tvy -= spd;
            if (!spin_mode) {                          // 小陀螺时 Q/E 让位给自旋
                if (keys_.down('Q')) twz += turn;      // +wz 逆时针
                if (keys_.down('E')) twz -= turn;
            }
            if (spin_mode) twz = o_.spin_rate_mradps;
        }
        {
            const double max_lin = 3000.0 * dt_target;   // 平移 3000 mm/s²
            const double max_ang = 6000.0 * dt_target;   // 转向 6 rad/s²
            auto slew = [](double& v, double t, double step) {
                const double d = t - v;
                v += std::max(-step, std::min(step, d));
            };
            slew(cvx, tvx, max_lin);
            slew(cvy, tvy, max_lin);
            slew(cwz, twz, max_ang);
        }

        // ---------- 3b2. 枪口坐标系 -> 车体坐标系 ----------
        // 麦轮能横移，所以可以"枪口朝哪、W 就往哪走"：先在枪口系里做限幅
        // （手感一致），再整体旋到车体系（车头 +x、左 +y）：
        //     theta = gun_sign * (yaw_deg - gun_offset)      yaw_deg = 云台相对车体的转角
        //     vx_car = u_f*cosθ - u_l*sinθ
        //     vy_car = u_f*sinθ + u_l*cosθ
        // ⚠️ 遥测断流时 theta 无从得知，退回车体系（等于 θ=0），界面会标出来。
        double ovx = cvx, ovy = cvy;
        gun_theta    = 0.0;
        gun_fellback = gun_drive && !have_telem;
        if (gun_drive && have_telem) {
            double th = o_.gun_sign * (last_telem.yaw_deg - gun_off_deg);
            th = std::fmod(th + 180.0, 360.0);
            if (th < 0.0) th += 360.0;
            th -= 180.0;                                   // 折到 [-180, 180)
            gun_theta = th;
            const double r = th * 3.14159265358979323846 / 180.0;
            const double c = std::cos(r), sn = std::sin(r);
            ovx = cvx * c - cvy * sn;
            ovy = cvx * sn + cvy * c;
        }

        // ---------- 3c. 小陀螺：yaw 反向补偿（相机在世界系站住）----------
        // 补偿基准用指令值；实测陀螺只在「同号且量级 ≥ 指令 25%」时采信，
        // 否则退回指令值 —— 陀螺坏掉恒报 0 时无脑采信会让云台跟着车一起转。
        double spin_comp_dps = 0.0;
        if (spin_mode || nowS() < spin_comp_until) {
            const bool fresh  = have_telem && (nowS() - t_last_telem < 0.3);
            const bool imu_ok = fresh &&
                ((last_telem.chassis_status & chassis_status::ImuOnline) != 0);
            double wz_src = cwz;
            if (imu_ok) {
                const double meas = last_telem.chassis_gyro_wz_mradps;
                if (meas * cwz > 0.0 && std::fabs(meas) >= 0.25 * std::fabs(cwz)) wz_src = meas;
            }
            const double alp = std::min(1.0, dt_target / 0.03);   // 30 ms 低通
            spin_wz_lp += (wz_src - spin_wz_lp) * alp;
            spin_comp_dps = -o_.spin_gain * spin_wz_lp * 0.001 * 57.29577951308232;
        } else {
            spin_wz_lp = 0.0;
        }

        // ---------- 4. 组包发送 ----------
        ControlCommand cc;
        cc.seq       = ++seq;
        cc.yaw_dps   = (hold_mode ? 0.0 : rate[0]) + spin_comp_dps;
        cc.pitch_dps = hold_mode ? 0.0 : rate[1];
        cc.flags     = oneshot;
        oneshot      = 0;
        if (en_state)      cc.flags |= ctrl_flag::Enable;
        else               cc.flags |= ctrl_flag::Disable;
        if (hold_mode)     cc.flags |= ctrl_flag::Hold;
        if (fire_hold)     cc.flags |= ctrl_flag::Fire;

        cc.chassis_vx_mmps   = ovx;   // 车体系（已经过枪口→车体旋转）
        cc.chassis_vy_mmps   = ovy;
        cc.chassis_wz_mradps = cwz;
        cc.chassis_flags     = 0;
        if (chassis_estop)   cc.chassis_flags |= chs_flag::EStop;
        else if (chassis_en) cc.chassis_flags |= chs_flag::Enable;
        if (oneshot_chassis_clear) {
            cc.chassis_flags |= chs_flag::Clear;
            oneshot_chassis_clear = false;
        }
        udp_.send(cc);

        // ---------- 5. 收遥测 ----------
        ControlTelemetry t;
        while (udp_.recv(t)) {
            if (last_echo && t.echo_seq > last_echo + 1)
                lost_pkts += static_cast<double>(t.echo_seq - last_echo - 1);
            last_echo    = t.echo_seq;
            last_telem   = t;
            have_telem   = true;
            t_last_telem = nowS();
        }

        // ---------- 6. 渲染（约 30 fps）----------
        const auto t_now = Clock::now();
        if (t_now - t_render >= std::chrono::milliseconds(kRenderMs)) {
            t_render = t_now;
            lines.clear();
            char buf[224];

            std::snprintf(buf, sizeof(buf), "gimbal_ctl (win) -> %s:%d   [arrows: gimbal]",
                          o_.host.c_str(), o_.ctrl_port);
            lines.emplace_back(buf, kWhite);

            if (::GetForegroundWindow() != hwnd_)
                lines.emplace_back("!! NOT FOCUSED - input disabled, click the window !!", kWarn);
            if (hold_mode) lines.emplace_back("** HOLD ** (space to resume)", kWarn);
            if (spin_mode) {
                std::snprintf(buf, sizeof(buf),
                              "** SPIN cmd %+.2f  act %+.2f rad/s ** yaw comp %+.0f deg/s (R)",
                              o_.spin_rate_mradps / 1000.0, spin_wz_lp / 1000.0, spin_comp_dps);
                lines.emplace_back(buf, kWarn);
            }
            if (chassis_fast) {
                std::snprintf(buf, sizeof(buf), "** FAST ** %.0f mm/s  %.1f rad/s (shift toggled)",
                              spd, turn / 1000.0);
                lines.emplace_back(buf, kWarn);
            }
            if (fire_hold) lines.emplace_back("** FIRE ** (F held)", kWarn);

            if (gun_drive) {
                if (gun_fellback) {
                    lines.emplace_back("drive GUN-REL ** NO TELEM -> fallback to chassis **", kWarn);
                } else {
                    std::snprintf(buf, sizeof(buf),
                                  "drive GUN-REL  theta %+6.1f deg  (y=set fwd, t=chassis)", gun_theta);
                    lines.emplace_back(buf, kWhite);
                }
            } else {
                lines.emplace_back("drive CHASSIS-REL  (t=gun-rel)", kWhite);
            }

            if (have_telem) {
                // 云台被失能/急停锁死时 Jetson 会把速率强制归零，必须在界面上喊出来
                if (!(last_telem.status & ctrl_status::Enabled))
                    lines.emplace_back("** GIMBAL DISABLED/LOCKED ** press g twice", kWarn);

                std::snprintf(buf, sizeof(buf),
                              "yaw    cmd %+7.1f  act %+7.1f  out %+9.2f deg",
                              rate[0], last_telem.yaw_dps, last_telem.yaw_deg);
                lines.emplace_back(buf, kWhite);
                std::snprintf(buf, sizeof(buf),
                              "pitch  cmd %+7.1f  act %+7.1f  out %+9.2f deg",
                              rate[1], last_telem.pitch_dps, last_telem.pitch_deg);
                lines.emplace_back(buf, kWhite);
                std::snprintf(buf, sizeof(buf),
                              "motor %+7.1f / %+7.1f RPM  %5.2fV  %dC  status=0x%02X",
                              last_telem.yaw_rpm, last_telem.pitch_rpm,
                              last_telem.vbus_mv / 1000.0, last_telem.temp_c,
                              last_telem.status);
                lines.emplace_back(buf, kWhite);

                const char* st   = "OK";
                uint32_t    stc  = kGood;
                if (last_telem.status & ctrl_status::EStop)         st = "ESTOP";
                else if (last_telem.status & ctrl_status::Stall)    st = "STALL!";
                else if (last_telem.status & ctrl_status::Watchdog) st = "WATCHDOG!";
                else if (!(last_telem.status & ctrl_status::YawOk) ||
                         !(last_telem.status & ctrl_status::PitchOk)) st = "MOTOR LOST!";
                if (std::strcmp(st, "OK") != 0) stc = kWarn;

                std::snprintf(buf, sizeof(buf), "link %dHz lost %.0f  RTT %.1fms  %s  rx=%dHz",
                              static_cast<int>(o_.hz), lost_pkts, udp_.rttMs(), st,
                              last_telem.rx_hz);
                lines.emplace_back(buf, stc);

                std::snprintf(buf, sizeof(buf),
                              "chassis pos %+7.0f,%+7.0f mm  yaw %+7.2f  gyro %+7.2f  %s %s",
                              last_telem.chassis_x_mm, last_telem.chassis_y_mm,
                              last_telem.chassis_yaw_deg, last_telem.chassis_gyro_yaw_deg,
                              last_telem.chassis_valid ? "ONLINE" : "OFFLINE",
                              chassis_estop ? "[ESTOP]" : (chassis_en ? "" : "[DISABLED]"));
                lines.emplace_back(buf, kWhite);
                std::snprintf(buf, sizeof(buf),
                              "        cmd %+5.0f,%+5.0f -> car %+5.0f,%+5.0f   "
                              "act %+5.0f,%+5.0f mm/s",
                              cvx, cvy, ovx, ovy,
                              last_telem.chassis_vx_mmps, last_telem.chassis_vy_mmps);
                lines.emplace_back(buf, kWhite);
                std::snprintf(buf, sizeof(buf),
                              "        wheel FL%+6.1f FR%+6.1f BL%+6.1f BR%+6.1f RPM",
                              last_telem.chassis_wheel_rpm[0], last_telem.chassis_wheel_rpm[1],
                              last_telem.chassis_wheel_rpm[2], last_telem.chassis_wheel_rpm[3]);
                lines.emplace_back(buf, kWhite);
            } else {
                lines.emplace_back("waiting for telemetry ... (gimbal running? --host right?)",
                                   kWarn);
            }

            if (o_.video) {
                std::snprintf(buf, sizeof(buf), "video %.1f fps %s", view_.fps(),
                              view_.connected() ? "connected" : "connecting...");
                lines.emplace_back(buf, view_.connected() ? kWhite : kWarn);
            } else {
                lines.emplace_back("video off (press v)", kWhite);
            }

            lines.emplace_back(
                "W/S move  A/D strafe  Q/E turn  F fire  SPACE hold  g/j enable  "
                "x estop  h home  z zero", kWhite);
            lines.emplace_back(
                "k kill-rate  c chassis-clear  R spin  SHIFT fast  v video  ESC quit  "
                "T drive-frame  Y set-fwd",
                kWhite);

            if (renderFrame(lines)) ++video_frames;

            // 每秒往控制台打一行视频统计（无头也能验证）
            const double dtv = std::chrono::duration<double>(Clock::now() - t_vstat).count();
            if (dtv >= 1.0) {
                std::printf("[video] 绘制 %.1f fps   拉流 %.1f fps\n",
                            static_cast<double>(video_frames) / dtv, view_.fps());
                video_frames = 0;
                t_vstat = Clock::now();
            }
        }

        // ---------- 7. 定频 ----------
        t_next += std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(dt_target));
        if (t_next > Clock::now()) sleepUntil(t_next);
        else                       t_next = Clock::now();
    }

    // ---- 收尾：明确发停手帧，别让云台保持上一拍速率 ----
    ControlCommand fin;
    fin.seq   = ++seq;
    fin.flags = ctrl_flag::Hold | (en_state ? ctrl_flag::Enable : 0);
    for (int i = 0; i < 3; ++i) {
        udp_.send(fin);
        ::Sleep(2);
    }

    shutdown();
    std::printf("\n已发送停手指令并退出\n");
    return 0;
}

int main(int argc, char** argv) {
    ::SetConsoleOutputCP(CP_UTF8);
    ::SetProcessDPIAware();

    Options o;
    loadIni(o);

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next  = [&](double def) { return (i + 1 < argc) ? std::atof(argv[++i]) : def; };
        auto nexti = [&](int def) { return (i + 1 < argc) ? std::atoi(argv[++i]) : def; };
        if (a == "-h" || a == "--help") { usage(argv[0]); return 0; }
        else if (a == "--host" && i + 1 < argc) o.host = argv[++i];
        else if (a == "--ctrl-port") o.ctrl_port = nexti(o.ctrl_port);
        else if (a == "--http-port") o.http_port = nexti(o.http_port);
        else if (a == "--no-video")  o.video = false;
        else if (a == "--tau")       o.tau = next(o.tau);
        else if (a == "--max-rate")  o.max_rate = next(o.max_rate);
        else if (a == "--gimbal-rate") o.gimbal_rate = next(o.gimbal_rate);
        else if (a == "--chassis-speed") o.chassis_speed = next(o.chassis_speed);
        else if (a == "--chassis-turn")  o.chassis_turn = next(o.chassis_turn);
        else if (a == "--shift-speed")   o.chassis_speed_fast = next(o.chassis_speed_fast);
        else if (a == "--shift-turn")    o.chassis_turn_fast = next(o.chassis_turn_fast);
        else if (a == "--spin-rate")     o.spin_rate_mradps = next(o.spin_rate_mradps);
        else if (a == "--spin-gain")     o.spin_gain = next(o.spin_gain);
        else if (a == "--gun-relative")  o.gun_relative = (nexti(1) != 0);
        else if (a == "--gun-offset")    o.gun_offset_deg = next(o.gun_offset_deg);
        else if (a == "--gun-sign")      o.gun_sign = (next(1.0) < 0 ? -1.0 : 1.0);
        else if (a == "--size") { o.win_w = nexti(o.win_w); o.win_h = nexti(o.win_h); }
        else if (a == "--hz")       o.hz = next(o.hz);
        else if (a == "--selftest") o.selftest = true;
        else {
            std::printf("未知参数: %s\n", a.c_str());
            usage(argv[0]);
            return 2;
        }
    }

    // ---- Winsock ----
    WSADATA wsa{};
    if (::WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        std::printf("WSAStartup 失败\n");
        return 1;
    }
    ::timeBeginPeriod(1);   // Sleep 精度 → 1 ms，否则 100 Hz 定频会被拖到 15 ms

    int rc = 0;
    if (o.selftest) {
        rc = runSelftest(o);
    } else {
        App app(o);
        rc = app.run();
    }

    ::timeEndPeriod(1);
    ::WSACleanup();
    return rc;
}

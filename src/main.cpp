// ===========================================================================
//  gimbal_ctl —— 云台/底盘控制客户端（PC 端，原生 X11）
//
//  职责: 方向键/WASD -> 换算指令 -> UDP 发给 Jetson -> 显示遥测 + 视频
//
//  云台方向键（角速度模式）:
//      按住 ←/→ = yaw 左/右   按住 ↑/↓ = pitch 抬头/低头
//      速率 --gimbal-rate（默认 120°/s），经一阶低通 --tau（默认 45ms）
//      松开 -> 目标为 0 -> 平滑衰减到 0
//      （鼠标不再参与控制，指针不再被锁定；--mode/--sens/--invert-y
//       参数保留但已不生效，仅为兼容旧命令行）
//
//  用法:
//      gimbal_ctl --host 192.168.2.2                 方向键控制 + 开视频
//      gimbal_ctl --host 192.168.2.2 --no-video       只要控制，不要视频
//      gimbal_ctl --host 192.168.2.2 --gimbal-rate 150  方向键更快
//      gimbal_ctl --selftest                          只测 UDP 收发，不开窗口
//
//  按键:
//      ↑/↓ = 云台俯仰        ←/→ = 云台左右       空格 = 停手/继续
//      W/S = 底盘前进/后退    A/D = 底盘左移/右移    Q/E = 底盘左转/右转
//      g = 云台使能开关       j = 底盘使能开关       x = 急停（云台+底盘）
//      c = 底盘里程计清零     h = 云台回中           z = 云台当前位置归零
//      k = 清空当前速率       v = 视频开关           F = 按住发射（水弹波箱）
//      R = 切换小陀螺模式（再按一下退出）
//      按一下 Shift（左/右均可）= 切换底盘「快速/普通」模式（锁存，不用一直按）：
//          普通档：平移 300 mm/s、转向 1.5 rad/s
//          快速档：平移 600 mm/s、转向 3.0 rad/s
//          再按一下 Shift 切回普通档；松开 Shift 不会退回，模式一直保持。
//          加速度斜坡仍是 3000 mm/s²（0->600 约 0.2 s），切换不会窜。
//          数值可用 --shift-speed / --shift-turn 改。
//
//  小陀螺模式（R 键切换，参考 RM 正赛）:
//      底盘按 --spin-rate（默认 2.0 rad/s）原地自旋，yaw 电机反向补偿，
//      让云台/相机在世界坐标系里保持不转；方向键照常控制云台（叠加在补偿上）。
//      补偿量优先用底盘陀螺实测角速度（chassis_gyro_wz_mradps），
//      这样地面打滑、掉速都能自动跟上；遥测断流或底盘 IMU 离线时退回指令值。
//      --spin-rate 取负值 = 反向自旋；--spin-gain -1 = 补偿方向反了时用。
// ===========================================================================
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/keysym.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "gimbal/link/chassis_link.hpp"   // chassis_status 状态位（判断底盘 IMU 是否在线）
#include "gimbal/link/control_link.hpp"
#include "mjpeg_view.hpp"

using namespace gimbal;
using namespace ctl;

// ---------------------------------------------------------------------------
static double nowS() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

/// 物理键是否按下（用 XQueryKeymap，天然免疫 X11 自动重复的 Press/Release 抖动）
static bool keyDown(Display* dpy, KeySym ks) {
    if (!dpy) return false;
    char keys[32];
    XQueryKeymap(dpy, keys);
    const KeyCode kc = XKeysymToKeycode(dpy, ks);
    if (kc == 0) return false;
    return (keys[kc >> 3] & (1 << (kc & 7))) != 0;
}

struct Options {
    std::string host      = "192.168.2.2";
    int         ctrl_port = CTRL_DEFAULT_PORT;
    int         http_port = 8080;
    bool        video     = true;
    std::string mode      = "auto";        ///< auto | grab | warp
    double      sens      = 0.09;          ///< °/s per (px/s)
    double      tau       = 0.045;         ///< 平滑时间常数 s（30→45ms：起停更顺）
    double      max_rate  = 180.0;         ///< 角速度上限 °/s
    double      gimbal_rate = 120.0;       ///< 方向键控制云台的速率 °/s
    double      deadzone  = 0.5;           ///< px/s 死区
    bool        invert_y  = false;         ///< 鼠标上下是否反向
    int         win_w     = 960;
    int         win_h     = 540;
    double      hz        = 100.0;
    bool        selftest  = false;

    // ---- 底盘（USART2 链路，坐标：vx 前进 / vy 左移 / wz 逆时针）----
    double      chassis_speed = 300.0;    ///< mm/s，WASD 满速
    double      chassis_turn  = 1500.0;   ///< mrad/s = 1.5 rad/s，←/→ 满速
    double      chassis_speed_fast = 600.0;   ///< mm/s，快速档（Shift 切换）的平移满速
    double      chassis_turn_fast  = 3000.0;  ///< mrad/s = 3.0 rad/s，快速档的转向
    bool        chassis_en    = true;     ///< j 键切换

    // ---- 小陀螺（R 键切换）：底盘原地自旋 + yaw 反向补偿让云台不转 ----
    double      spin_rate_mradps = 2000.0;  ///< 0.001 rad/s，+ 逆时针；负值 = 顺时针
    double      spin_gain        = 1.0;     ///< yaw 反向补偿倍率（方向反了填 -1）
};

static void usage(const char* a0) {
    std::printf(
        "用法: %s [选项]\n"
        "  --host <ip|名字>     Jetson 地址 (默认 192.168.2.2)\n"
        "  --ctrl-port <n>      UDP 控制口 (默认 %u)\n"
        "  --http-port <n>      MJPEG 端口 (默认 8080)\n"
        "  --no-video           不显示视频\n"
        "  --mode <模式>        [已废弃] auto | grab | warp（鼠标不再参与控制）\n"
        "  --sens <f>           [已废弃] 旧鼠标灵敏度\n"
        "  --tau <s>            平滑时间常数 (默认 0.045)\n"
        "  --max-rate <dps>     角速度上限 (默认 180)\n"
        "  --gimbal-rate <dps>  方向键云台速率 (默认 120)\n"
        "  --invert-y           [已废弃] 旧鼠标上下反向\n"
        "  --size <w> <h>       窗口尺寸 (默认 960 540)\n"
        "  --hz <n>             发送频率 (默认 100)\n"
        "  --chassis-speed <mm/s> 底盘平移满速 (默认 300)\n"
        "  --chassis-turn <mrad/s> 底盘转向满速 (默认 1500)\n"
        "  --shift-speed <mm/s> 快速档平移满速（Shift 切换，默认 600）\n"
        "  --shift-turn <mrad/s> 快速档转向满速（Shift 切换，默认 3000）\n"
        "  --spin-rate <mrad/s> 小陀螺自旋速率（+逆时针，默认 2000 = 2.0 rad/s）\n"
        "  --spin-gain <f>      小陀螺 yaw 补偿倍率（默认 1.0，方向反了填 -1）\n"
        "  --selftest           只测 UDP 收发，不开窗口\n"
        "  -h, --help\n", a0, CTRL_DEFAULT_PORT);
}

// ===========================================================================
// UDP 客户端
// ===========================================================================
class UdpClient {
public:
    bool open(const std::string& host, int port) {
        struct addrinfo hints;
        std::memset(&hints, 0, sizeof(hints));
        hints.ai_family   = AF_UNSPEC;
        hints.ai_socktype = SOCK_DGRAM;
        struct addrinfo* res = nullptr;
        if (::getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &res) != 0)
            return false;
        fd_ = ::socket(res->ai_family, res->ai_socktype, res->ai_protocol);
        if (fd_ < 0) { ::freeaddrinfo(res); return false; }
        if (::connect(fd_, res->ai_addr, res->ai_addrlen) != 0) {
            ::freeaddrinfo(res);
            ::close(fd_);
            fd_ = -1;
            return false;
        }
        ::freeaddrinfo(res);
        return true;
    }
    ~UdpClient() { if (fd_ >= 0) ::close(fd_); }
    bool ok() const { return fd_ >= 0; }

    void send(const ControlCommand& c) {
        if (fd_ < 0) return;
        const auto b = c.encode();
        ::send(fd_, b.data(), b.size(), 0);
        sent_at_[c.seq % kRing] = nowS();
        if (c.seq % kRing == 0) { /* 环形复用，够用 */ }
    }

    /// 非阻塞收遥测
    bool recv(ControlTelemetry& t) {
        if (fd_ < 0) return false;
        bool got = false;
        for (;;) {
            uint8_t buf[512];
            const ssize_t n = ::recv(fd_, buf, sizeof(buf), MSG_DONTWAIT);
            if (n <= 0) break;
            ControlTelemetry u;
            if (!ControlTelemetry::decode(buf, static_cast<size_t>(n), u)) {
                ++bad_;
                continue;
            }
            const double t0 = sent_at_[u.echo_seq % kRing];
            if (t0 > 0) {
                rtt_ms_ = (nowS() - t0) * 1000.0;
            }
            t   = u;
            got = true;
        }
        return got;
    }

    uint64_t bad() const { return bad_; }
    double   rttMs() const { return rtt_ms_; }

private:
    static constexpr size_t kRing = 256;
    int      fd_ = -1;
    double   sent_at_[kRing] = {0};
    uint64_t bad_  = 0;
    double   rtt_ms_ = -1.0;
};

// ===========================================================================
// 自检（不开窗口，只验 UDP 收发 + 协议）
// ===========================================================================
static int runSelftest(const Options& o) {
    std::printf("=== gimbal_ctl 自检 ===\n");
    int fail = 0;
    auto CHECK = [&](bool ok, const char* m) {
        std::printf("  [%s] %s\n", ok ? " OK " : "FAIL", m);
        if (!ok) ++fail;
    };

    ControlCommand c;
    c.seq = 42; c.yaw_dps = 45.0; c.pitch_dps = -30.0; c.flags = ctrl_flag::Enable;
    const auto b = c.encode();
    ControlCommand d;
    CHECK(ControlCommand::decode(b.data(), b.size(), d), "上行编解码");
    CHECK(std::fabs(d.yaw_dps - 45.0) < 1e-9 && std::fabs(d.pitch_dps + 30.0) < 1e-9,
          "角速度往返");

    // v2：底盘字段（坐标 vx 前进 / vy 左移 / wz 逆时针）
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
    cbad[16] ^= 0xFF;
    CHECK(!ControlCommand::decode(cbad.data(), cbad.size(), cd), "底盘字段篡改被拒");

    UdpClient u;
    const bool opened = u.open(o.host, o.ctrl_port);
    CHECK(opened, ("UDP socket 已建立 -> " + o.host + ":" +
                   std::to_string(o.ctrl_port)).c_str());
    if (!opened) return 1;

    std::printf("  发 50 个包并等 1 秒看回包…\n");
    for (uint32_t i = 1; i <= 50; ++i) {
        ControlCommand s;
        s.seq = i; s.yaw_dps = 0; s.pitch_dps = 0; s.flags = 0;
        u.send(s);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    ControlTelemetry t;
    bool got = false;
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
    CHECK(got, "收到 Jetson 遥测回包");
    CHECK(u.bad() == 0, "无坏包");
    std::printf("\n%s（失败 %d 项）\n", fail ? "有失败" : "全部通过", fail);
    return fail ? 1 : 0;
}

// ===========================================================================
// 主程序
// ===========================================================================
int main(int argc, char** argv) {
    Options o;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next  = [&](double def) { return (i + 1 < argc) ? std::atof(argv[++i]) : def; };
        auto nexti = [&](int def)    { return (i + 1 < argc) ? std::atoi(argv[++i]) : def; };
        if (a == "-h" || a == "--help") { usage(argv[0]); return 0; }
        else if (a == "--host" && i + 1 < argc) o.host = argv[++i];
        else if (a == "--ctrl-port") o.ctrl_port = nexti(o.ctrl_port);
        else if (a == "--http-port") o.http_port = nexti(o.http_port);
        else if (a == "--no-video")  o.video = false;
        else if (a == "--mode" && i + 1 < argc) o.mode = argv[++i];
        else if (a == "--sens")      o.sens = next(o.sens);
        else if (a == "--tau")       o.tau = next(o.tau);
        else if (a == "--max-rate")  o.max_rate = next(o.max_rate);
        else if (a == "--gimbal-rate") o.gimbal_rate = next(o.gimbal_rate);
        else if (a == "--invert-y")  o.invert_y = true;
        else if (a == "--chassis-speed") o.chassis_speed = next(o.chassis_speed);
        else if (a == "--chassis-turn")  o.chassis_turn = next(o.chassis_turn);
        else if (a == "--shift-speed")   o.chassis_speed_fast = next(o.chassis_speed_fast);
        else if (a == "--shift-turn")    o.chassis_turn_fast = next(o.chassis_turn_fast);
        else if (a == "--spin-rate")     o.spin_rate_mradps = next(o.spin_rate_mradps);
        else if (a == "--spin-gain")     o.spin_gain = next(o.spin_gain);
        else if (a == "--size")      { o.win_w = nexti(o.win_w); o.win_h = nexti(o.win_h); }
        else if (a == "--hz")        o.hz = next(o.hz);
        else if (a == "--selftest")  o.selftest = true;
        else { std::printf("未知参数: %s\n", a.c_str()); usage(argv[0]); return 2; }
    }

    if (o.selftest) return runSelftest(o);

    // ---- UDP ----
    UdpClient udp;
    if (!udp.open(o.host, o.ctrl_port)) {
        std::printf("无法建立到 %s:%d 的 UDP 通道\n", o.host.c_str(), o.ctrl_port);
        return 1;
    }

    // ---- X11 ----
    Display* dpy = XOpenDisplay(nullptr);
    if (!dpy) {
        std::printf("打不开 X display（DISPLAY=%s）\n",
                    getenv("DISPLAY") ? getenv("DISPLAY") : "(空)");
        std::printf("当前会话是 Wayland 的话，本程序走 XWayland；"
                    "若仍失败请确认 XWayland 可用\n");
        return 1;
    }
    const int scr = DefaultScreen(dpy);
    const auto vis = DefaultVisual(dpy, scr);
    const int depth = DefaultDepth(dpy, scr);

    // 检查像素格式（视频直写 ZPixmap 需要常见 TrueColor）
    const bool truecolor = (vis->c_class == TrueColor) && depth >= 24 &&
                           vis->red_mask == 0xFF0000 &&
                           vis->green_mask == 0x00FF00 &&
                           vis->blue_mask == 0x0000FF;

    std::printf("[x11] depth=%d class=%s masks R=%#lx G=%#lx B=%#lx -> %s\n",
                depth,
                vis->c_class == TrueColor ? "TrueColor" : "非TrueColor",
                vis->red_mask, vis->green_mask, vis->blue_mask,
                truecolor ? "支持直写视频" : "不支持直写视频(只显示状态文字)");

    Window win = XCreateSimpleWindow(dpy, RootWindow(dpy, scr), 0, 0,
                                     static_cast<unsigned>(o.win_w),
                                     static_cast<unsigned>(o.win_h), 0,
                                     BlackPixel(dpy, scr), BlackPixel(dpy, scr));
    XStoreName(dpy, win, "gimbal_ctl");
    XSelectInput(dpy, win, ExposureMask | KeyPressMask | ButtonPressMask |
                               PointerMotionMask | StructureNotifyMask |
                               FocusChangeMask);
    XMapWindow(dpy, win);

    Atom wm_delete = XInternAtom(dpy, "WM_DELETE_WINDOW", False);
    XSetWMProtocols(dpy, win, &wm_delete, 1);

    XGCValues gcv;
    GC gc = XCreateGC(dpy, win, 0, &gcv);
    XSetForeground(dpy, gc, WhitePixel(dpy, scr));

    XFontStruct* font = XLoadQueryFont(dpy, "fixed");
    if (font) XSetFont(dpy, gc, font->fid);

    // ---- 云台改为方向键控制：不再锁定鼠标指针 ----

    // ---- 视频 ----
    MjpegView  view;
    uint64_t   shown_seq = 0;
    bool       video_on  = o.video;
    if (video_on) {
        view.start(o.host, o.http_port);
        std::printf("[video] 拉流 http://%s:%d/stream\n", o.host.c_str(), o.http_port);
    }

    // ---- 状态 ----
    ControlTelemetry last_telem;
    bool    have_telem = false;
    double  rate[2]    = {0.0, 0.0};        // yaw, pitch（平滑后的 °/s）
    uint32_t seq       = 0;
    bool    hold_mode  = false;
    bool    en_state   = true;
    uint8_t oneshot    = 0;                 // 本拍要带的一次性 flags
    bool    fire_hold_key = false;          // 按住 F = 发射请求（本拍）

    // ---- 底盘状态（WASD 平滑后下发）----
    bool     chassis_en        = o.chassis_en;
    bool     chassis_estop     = false;
    bool     chassis_fast      = false;   // 快速档开关：按一下 Shift 切换
    bool     shift_prev        = false;   // 上一拍的 Shift 物理状态（上升沿检测用）
    bool     spin_mode         = false;   // 小陀螺：R 键切换
    double   spin_wz_lp        = 0.0;     // 实测自旋角速度的低通值 (mrad/s)
    double   spin_comp_until   = 0.0;     // 关掉小陀螺后继续补偿到这一刻（吃掉减速段）
    double   t_last_telem      = -1e9;    // 上次收到遥测的时刻（算新鲜度）
    bool     oneshot_chassis_clear = false;
    double   cvx = 0.0, cvy = 0.0, cwz = 0.0;   // 平滑后的 mm/s 与 mrad/s
    double  lost_pkts = 0;
    uint32_t last_echo = 0;

    std::printf("========================================================\n");
    std::printf("  gimbal_ctl 云台/底盘控制\n");
    std::printf("  目标    : %s  UDP:%d  MJPEG:%d\n", o.host.c_str(), o.ctrl_port,
                o.http_port);
    std::printf("  云台    : 方向键速率 %.0f°/s   平滑 tau=%.0fms   上限 %.0f°/s\n",
                o.gimbal_rate, o.tau * 1000, o.max_rate);
    std::printf("  按键    : ↑/↓=云台俯仰  ←/→=云台左右  空格=停手  g=云台使能  j=底盘使能\n");
    std::printf("            W/S=前后  A/D=左右  Q/E=底盘转向  F=按住发射  x=急停  h=回中\n");
    std::printf("            z=归零  k=清速率  v=视频开关  Esc=退出\n");
    std::printf("            按一下 Shift = 切换快速/普通模式（锁存，不用一直按）\n");
    std::printf("            R = 切换小陀螺（底盘自旋 + 云台反向站住）\n");
    std::printf("  底盘    : 普通档 平移 %.0f mm/s  转向 %.0f mrad/s\n",
                o.chassis_speed, o.chassis_turn);
    std::printf("            快速档 平移 %.0f mm/s  转向 %.0f mrad/s\n",
                o.chassis_speed_fast, o.chassis_turn_fast);
    std::printf("  小陀螺  : 自旋 %+.2f rad/s（%s）  云台补偿倍率 %+.2f\n",
                o.spin_rate_mradps / 1000.0,
                o.spin_rate_mradps >= 0 ? "逆时针" : "顺时针", o.spin_gain);
    std::printf("            vx 前进 / vy 左移 / wz 逆时针\n");
    std::printf("========================================================\n");

    const double dt_target = 1.0 / o.hz;
    auto         t_next    = std::chrono::steady_clock::now();
    auto         t_render  = std::chrono::steady_clock::now();   // 上次渲染时刻
    bool         running   = true;
    bool         video_first_done = false;
    int          video_frames = 0;
    auto         t_vstat  = std::chrono::steady_clock::now();

    while (running) {
        // ---------- 1. X 事件 ----------
        while (XPending(dpy)) {
            XEvent ev;
            XNextEvent(dpy, &ev);
            switch (ev.type) {
                case KeyPress: {
                    const KeySym ks = XLookupKeysym(&ev.xkey, 0);
                    if (ks == XK_Escape) running = false;
                    else if (ks == XK_space) hold_mode = !hold_mode;
                    else if (ks == XK_g) { en_state = !en_state;
                                           oneshot |= ctrl_flag::Enable; }
                    else if (ks == XK_j) { chassis_en = !chassis_en;
                                           if (chassis_en) chassis_estop = false;
                                           std::printf("[chassis] 使能=%d\n", chassis_en ? 1 : 0); }
                    else if (ks == XK_c) oneshot_chassis_clear = true;
                    else if (ks == XK_x) { oneshot |= ctrl_flag::EStop;
                                           chassis_estop = true;
                                           std::printf("[chassis] 急停（握手 e/f 恢复）\n"); }
                    else if (ks == XK_h) oneshot |= ctrl_flag::Home;
                    else if (ks == XK_z) oneshot |= ctrl_flag::Zero;
                    else if (ks == XK_k) { rate[0] = rate[1] = 0.0; }   // 清空当前速率（原 R 键）
                    else if (ks == XK_r) {                              // R = 小陀螺切换
                        spin_mode = !spin_mode;
                        spin_wz_lp = 0.0;                               // 复位补偿滤波器，免得上一次残留
                        // 关掉时底盘还要 0.33 s 才停转，这段时间继续补偿，否则相机会被甩一下
                        if (!spin_mode) spin_comp_until = nowS() + 0.6;
                        std::printf("[chassis] 小陀螺 %s（自旋 %+.2f rad/s，云台反向补偿）\n",
                                    spin_mode ? "开" : "关", o.spin_rate_mradps / 1000.0);
                    }
                    else if (ks == XK_v) {
                        video_on = !video_on;
                        if (video_on) view.start(o.host, o.http_port);
                        else          view.stop();
                    }
                    break;
                }
                case ClientMessage:
                    if (static_cast<Atom>(ev.xclient.data.l[0]) == wm_delete)
                        running = false;
                    break;
                default:
                    break;
            }
        }

        // ---------- 1b. 发射扳机：按住 F（空格停手时不响应）----------
        fire_hold_key = !hold_mode && keyDown(dpy, XK_f);

        // ---------- 2. 云台：方向键速率控制（↑/↓=pitch 抬头/低头，←/→=yaw 左/右）----------
        double target_yaw   = 0.0;
        double target_pitch = 0.0;
        if (!hold_mode) {
            if (keyDown(dpy, XK_Left))  target_yaw   += o.gimbal_rate;
            if (keyDown(dpy, XK_Right)) target_yaw   -= o.gimbal_rate;
            if (keyDown(dpy, XK_Up))    target_pitch += o.gimbal_rate;
            if (keyDown(dpy, XK_Down))  target_pitch -= o.gimbal_rate;
        }

        target_yaw   = std::max(-o.max_rate, std::min(o.max_rate, target_yaw));
        target_pitch = std::max(-o.max_rate, std::min(o.max_rate, target_pitch));

        const double a = std::min(1.0, dt_target / std::max(1e-3, o.tau));
        rate[0] += (target_yaw   - rate[0]) * a;
        rate[1] += (target_pitch - rate[1]) * a;

        // ---------- 3b. 底盘：键盘 -> 平滑限幅（加速度限幅防打滑/冲击）----------
        // 按一下 Shift（左右均可）= 切换快速/普通档，锁存到再按一次为止；松开不退回。
        // 用「上一拍 vs 这一拍」的上升沿做切换：XQueryKeymap 轮询的是物理按键状态，
        // 长按只会保持 true，不会像 KeyPress 事件那样被 X11 自动重复刷爆。
        const bool shift_now = keyDown(dpy, XK_Shift_L) || keyDown(dpy, XK_Shift_R);
        if (shift_now && !shift_prev) {
            chassis_fast = !chassis_fast;
            std::printf("[chassis] 切到%s档：平移 %.0f mm/s / 转向 %.0f mrad/s\n",
                        chassis_fast ? "快速" : "普通",
                        chassis_fast ? o.chassis_speed_fast : o.chassis_speed,
                        chassis_fast ? o.chassis_turn_fast  : o.chassis_turn);
        }
        shift_prev = shift_now;

        const double spd  = chassis_fast ? o.chassis_speed_fast : o.chassis_speed;
        const double turn = chassis_fast ? o.chassis_turn_fast  : o.chassis_turn;

        double tvx = 0.0, tvy = 0.0, twz = 0.0;
        if (!hold_mode && !chassis_estop) {
            if (keyDown(dpy, XK_w))      tvx += spd;
            if (keyDown(dpy, XK_s))      tvx -= spd;
            if (keyDown(dpy, XK_a))      tvy += spd;    // +y = 左移
            if (keyDown(dpy, XK_d))      tvy -= spd;
            if (!spin_mode) {                           // 小陀螺时旋转交给自旋速率，Q/E 让位
                if (keyDown(dpy, XK_q))  twz += turn;   // Q = 左转（+wz 逆时针）
                if (keyDown(dpy, XK_e))  twz -= turn;   // E = 右转
            }
            if (spin_mode) twz = o.spin_rate_mradps;    // 小陀螺：恒定自旋（WASD 平移照常）
        }
        {
            // 平移 3000 mm/s²、转向 6 rad/s²：满速约 0.1~0.25 s
            const double max_lin = 3000.0 * dt_target;
            const double max_ang = 6000.0 * dt_target;
            auto slew = [](double& v, double t, double step) {
                const double d = t - v;
                v += std::max(-step, std::min(step, d));
            };
            slew(cvx, tvx, max_lin);
            slew(cvy, tvy, max_lin);
            slew(cwz, twz, max_ang);
        }

        // ---------- 3c. 小陀螺：yaw 反向补偿，让云台在世界系里不转 ----------
        // 底盘在世界系里转 +ω，yaw 电机就必须相对底盘转 −ω，相机才站得住。
        // 补偿以**指令值 cz 为基准**，实测陀螺只用来"修正"：
        //   实测与指令同号、且量级 ≥ 指令的 25% 时才采信（能吸掉打滑/掉速的误差）；
        //   否则退回指令值 —— 陀螺坏掉/轴映射不对时会恒报 0，
        //   无脑采信会让补偿直接变成 0，云台跟着车一起转（实机踩过）。
        double spin_comp_dps = 0.0;
        const bool comp_on = spin_mode || (nowS() < spin_comp_until);
        if (comp_on) {
            const bool fresh  = have_telem && (nowS() - t_last_telem < 0.3);
            const bool imu_ok = fresh &&
                ((last_telem.chassis_status & chassis_status::ImuOnline) != 0);
            double wz_src = cwz;
            if (imu_ok) {
                const double meas = last_telem.chassis_gyro_wz_mradps;
                if (meas * cwz > 0.0 && std::fabs(meas) >= 0.25 * std::fabs(cwz)) {
                    wz_src = meas;          // 实测可信：用它抵消打滑
                }
            }
            const double alp = std::min(1.0, dt_target / 0.03);   // 30 ms 低通压陀螺噪声
            spin_wz_lp += (wz_src - spin_wz_lp) * alp;
            spin_comp_dps = -o.spin_gain * spin_wz_lp * 0.001 * 57.29577951308232;  // rad/s -> °/s
        } else {
            spin_wz_lp = 0.0;
        }

        // ---------- 4. 组包发送 ----------
        ControlCommand cc;
        cc.seq       = ++seq;
        // 小陀螺时补偿一直生效（哪怕按了空格"停手"）：底盘自旋在减速的那 0.3 s 里
        // 仍然在转，yaw 必须继续抵着，相机才不会跟着甩一下。
        cc.yaw_dps   = (hold_mode ? 0.0 : rate[0]) + spin_comp_dps;
        cc.pitch_dps = hold_mode ? 0.0 : rate[1];
        cc.flags     = oneshot;
        oneshot      = 0;
        if (en_state)        cc.flags |= ctrl_flag::Enable;
        else                 cc.flags |= ctrl_flag::Disable;
        if (hold_mode)       cc.flags |= ctrl_flag::Hold;
        if (fire_hold_key)   cc.flags |= ctrl_flag::Fire;   // 按住 F 发射

        // ---- 底盘字段（坐标：vx 前进 / vy 左移 / wz 逆时针）----
        cc.chassis_vx_mmps   = cvx;
        cc.chassis_vy_mmps   = cvy;
        cc.chassis_wz_mradps = cwz;
        cc.chassis_flags     = 0;
        if (chassis_estop)            cc.chassis_flags |= chs_flag::EStop;
        else if (chassis_en)          cc.chassis_flags |= chs_flag::Enable;
        if (oneshot_chassis_clear) {  cc.chassis_flags |= chs_flag::Clear;
                                      oneshot_chassis_clear = false; }
        udp.send(cc);

        // ---------- 5. 收遥测 ----------
        ControlTelemetry t;
        while (udp.recv(t)) {
            if (last_echo && t.echo_seq > last_echo + 1)
                lost_pkts += static_cast<double>(t.echo_seq - last_echo - 1);
            last_echo = t.echo_seq;
            last_telem = t;
            have_telem = true;
            t_last_telem = nowS();      // 小陀螺补偿要判断遥测是否新鲜
        }

        // ---------- 6. 渲染（约 30fps，别挡控制）----------
        // ⚠️ 必须写成"距上次渲染超过 33ms 才渲染"。
        //    绝不能写 t_render += 33ms 再比大小：循环每轮只过 10ms 却加 33ms，
        //    t_render 会以 3.3 倍速飞出去，那个 if 就永远不成立（踩过）。
        const auto t_now = std::chrono::steady_clock::now();
        if (t_now - t_render >= std::chrono::milliseconds(33)) {
            t_render = t_now;


            XClearWindow(dpy, win);

            // 视频居中绘制
            if (video_on && truecolor) {
                MjpegView::Frame f;
                if (view.getLatest(f, shown_seq) && f.width > 0) {
                    if (!video_first_done) {
                        video_first_done = true;
                        std::printf("[video] 首帧 %dx%d 已解码并绘制\n",
                                    f.width, f.height);
                    }
                    const int vx = std::max(0, (o.win_w - f.width) / 2);
                    const int vy = std::max(0, (o.win_h - f.height) / 2);
                    XImage* img = XCreateImage(dpy, vis, static_cast<unsigned>(depth),
                                               ZPixmap, 0,
                                               reinterpret_cast<char*>(f.bgrx.data()),
                                               static_cast<unsigned>(f.width),
                                               static_cast<unsigned>(f.height),
                                               32, f.width * 4);
                    if (img) {
                        XPutImage(dpy, win, gc, img, 0, 0, vx, vy,
                                  static_cast<unsigned>(f.width),
                                  static_cast<unsigned>(f.height));
                        ++video_frames;
                        img->data = nullptr;   // 别让 XDestroyImage 释放我们的 buffer
                        XDestroyImage(img);
                    }
                }
            }

            // 状态文字
            char line[192];
            int  ty = 16;
            auto text = [&](const char* s) {
                XDrawString(dpy, win, gc, 10, ty, s, static_cast<int>(std::strlen(s)));
                ty += 15;
            };

            std::snprintf(line, sizeof(line), "gimbal_ctl -> %s:%d   [keys: arrows]",
                          o.host.c_str(), o.ctrl_port);
            text(line);

            if (hold_mode) text("** HOLD ** (space to resume)");
            if (spin_mode) {
                std::snprintf(line, sizeof(line),
                              "** SPIN cmd %+.2f  act %+.2f rad/s ** yaw comp %+.0f deg/s (R)",
                              o.spin_rate_mradps / 1000.0, spin_wz_lp / 1000.0,
                              spin_comp_dps);
                text(line);
            }
            if (chassis_fast) {
                std::snprintf(line, sizeof(line),
                              "** FAST ** %.0f mm/s  %.1f rad/s (shift toggled)",
                              spd, turn / 1000.0);
                text(line);
            }
            if (fire_hold_key) text("** FIRE ** (F held)");

            // 云台被失能/急停锁死时，Jetson 会把速率强制归零（连 F6 帧都不发），
            // 现场表现就是"yaw 完全不动"——必须在界面上喊出来，不然只能靠猜。
            if (have_telem && !(last_telem.status & ctrl_status::Enabled)) {
                text("** GIMBAL DISABLED/LOCKED ** 按 g 关再开(两下)重新授权");
            }

            if (have_telem) {
                std::snprintf(line, sizeof(line),
                              "yaw    cmd %+7.1f  act %+7.1f  out %+9.2f deg",
                              rate[0], last_telem.yaw_dps, last_telem.yaw_deg);
                text(line);
                std::snprintf(line, sizeof(line),
                              "pitch  cmd %+7.1f  act %+7.1f  out %+9.2f deg",
                              rate[1], last_telem.pitch_dps, last_telem.pitch_deg);
                text(line);
                std::snprintf(line, sizeof(line),
                              "motor %+7.1f / %+7.1f RPM  %5.2fV  %dC  status=0x%02X",
                              last_telem.yaw_rpm, last_telem.pitch_rpm,
                              last_telem.vbus_mv / 1000.0, last_telem.temp_c,
                              last_telem.status);
                text(line);

                const char* st = "OK";
                if (last_telem.status & ctrl_status::EStop)    st = "ESTOP";
                else if (last_telem.status & ctrl_status::Stall)    st = "STALL!";
                else if (last_telem.status & ctrl_status::Watchdog) st = "WATCHDOG!";
                else if (!(last_telem.status & ctrl_status::YawOk) ||
                         !(last_telem.status & ctrl_status::PitchOk)) st = "MOTOR LOST!";

                std::snprintf(line, sizeof(line),
                              "link %dHz lost %.0f  RTT %.1fms  %s  rx=%dHz",
                              static_cast<int>(o.hz), lost_pkts, udp.rttMs(), st,
                              last_telem.rx_hz);
                text(line);

                // ---- 底盘遥测 ----
                std::snprintf(line, sizeof(line),
                              "chassis pos %+7.0f,%+7.0f mm  yaw %+7.2f  cmd %+5.0f,%+5.0f mm/s",
                              last_telem.chassis_x_mm, last_telem.chassis_y_mm,
                              last_telem.chassis_yaw_deg, cvx, cvy);
                text(line);
                std::snprintf(line, sizeof(line),
                              "        wheel FL%+6.1f FR%+6.1f BL%+6.1f BR%+6.1f RPM",
                              last_telem.chassis_wheel_rpm[0], last_telem.chassis_wheel_rpm[1],
                              last_telem.chassis_wheel_rpm[2], last_telem.chassis_wheel_rpm[3]);
                text(line);
                std::snprintf(line, sizeof(line),
                              "        act %+5.0f,%+5.0f mm/s  gyro_yaw %+7.2f  chassis %s  %s",
                              last_telem.chassis_vx_mmps, last_telem.chassis_vy_mmps,
                              last_telem.chassis_gyro_yaw_deg,
                              last_telem.chassis_valid ? "ONLINE" : "OFFLINE",
                              chassis_estop ? "[ESTOP]" : (chassis_en ? "" : "[DISABLED]"));
                text(line);
            } else {
                text("waiting for telemetry ... (check gimbal running + --host)");
            }

            if (video_on) {
                std::snprintf(line, sizeof(line), "video %.1f fps %s",
                              view.fps(), view.connected() ? "connected" : "connecting...");
            } else {
                std::snprintf(line, sizeof(line), "video off (press v)");
            }
            text(line);

            XFlush(dpy);

            // 每秒打一行视频统计到 stdout（方便无头验证）
            const auto ts = std::chrono::steady_clock::now();
            const double dtv = std::chrono::duration<double>(ts - t_vstat).count();
            if (dtv >= 1.0) {
                std::printf("[video] 绘制 %.1f fps   拉流 %.1f fps\n",
                            video_frames / dtv, view.fps());
                video_frames = 0;
                t_vstat = ts;
            }
        }

        // ---------- 7. 定频 ----------
        t_next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(dt_target));
        const auto now = std::chrono::steady_clock::now();
        if (t_next > now) std::this_thread::sleep_until(t_next);
        else              t_next = now;
    }

    // ---- 收尾：明确发一帧停手，别让云台保持上一个速率 ----
    ControlCommand fin;
    fin.seq = ++seq; fin.yaw_dps = 0; fin.pitch_dps = 0;
    fin.flags = ctrl_flag::Hold | (en_state ? ctrl_flag::Enable : 0);
    for (int i = 0; i < 3; ++i) { udp.send(fin); usleep(2000); }

    view.stop();
    if (font) XFreeFont(dpy, font);
    XFreeGC(dpy, gc);
    XDestroyWindow(dpy, win);
    XCloseDisplay(dpy);
    std::printf("\n已发送停手指令并退出\n");
    return 0;
}

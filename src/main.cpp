// ===========================================================================
//  gimbal_ctl —— 云台鼠标控制客户端（PC 端，原生 X11）
//
//  职责: 抓鼠标 -> 算角速度 -> UDP 发给 Jetson -> 显示遥测 + 视频
//
//  鼠标映射（角速度模式，像 FPS 视角）:
//      鼠标移动速度(px/s) × 灵敏度(°/px) = 目标角速度(°/s)
//      目标值经一阶低通平滑（时间常数 --tau，默认 50ms）
//      停手 -> 目标为 0 -> 平滑衰减到 0
//
//  ⚠️ 关于 Wayland:
//      在 Wayland 会话下本程序跑在 XWayland 里。XGrabPointer 的"锁定"
//      能否穿透到合成器取决于 XWayland 版本，不保证。
//      因此提供两种模式：
//          --mode grab  用 XGrabPointer 把指针限制在窗口内（X11 下最干净）
//          --mode warp  每个控制周期把指针 XWarpPointer 回窗口中心，
//                       用位置差算增量 —— **XWayland 下这个一定可用**
//          --mode auto  先试 grab，失败自动降级 warp（默认）
//
//  用法:
//      gimbal_ctl --host 192.168.2.2                 自动选模式 + 开视频
//      gimbal_ctl --host nano.local --mode warp
//      gimbal_ctl --host 192.168.2.2 --no-video       纯鼠标控制
//      gimbal_ctl --selftest                          只测 UDP 收发，不开窗口
//
//  按键:
//      移动鼠标 = 转云台      Esc/q = 退出       空格 = 停手/继续
//      W/S = 前进/后退        A/D = 左移/右移      ←/→ = 底盘左转/右转
//      e = 云台使能开关       f = 底盘使能开关      x = 急停（云台+底盘）
//      c = 底盘里程计清零     h = 云台回中          z = 云台当前位置归零
//      r = 清空当前速率       v = 视频开关
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
    double      tau       = 0.03;          ///< 平滑时间常数 s（50→30ms，降感知延迟）
    double      max_rate  = 180.0;         ///< 角速度上限 °/s
    double      deadzone  = 0.5;           ///< px/s 死区
    bool        invert_y  = false;         ///< 鼠标上下是否反向
    int         win_w     = 960;
    int         win_h     = 540;
    double      hz        = 100.0;
    bool        selftest  = false;

    // ---- 底盘（USART2 链路，坐标：vx 前进 / vy 左移 / wz 逆时针）----
    double      chassis_speed = 300.0;    ///< mm/s，WASD 满速
    double      chassis_turn  = 1500.0;   ///< mrad/s = 1.5 rad/s，←/→ 满速
    bool        chassis_en    = true;     ///< f 键切换
};

static void usage(const char* a0) {
    std::printf(
        "用法: %s [选项]\n"
        "  --host <ip|名字>     Jetson 地址 (默认 192.168.2.2)\n"
        "  --ctrl-port <n>      UDP 控制口 (默认 %u)\n"
        "  --http-port <n>      MJPEG 端口 (默认 8080)\n"
        "  --no-video           不显示视频\n"
        "  --mode <模式>        auto | grab | warp (默认 auto)\n"
        "  --sens <f>           灵敏度 °/px (默认 0.09)\n"
        "  --tau <s>            平滑时间常数 (默认 0.05)\n"
        "  --max-rate <dps>     角速度上限 (默认 180)\n"
        "  --invert-y           鼠标上下反向\n"
        "  --size <w> <h>       窗口尺寸 (默认 960 540)\n"
        "  --hz <n>             发送频率 (默认 100)\n"
        "  --chassis-speed <mm/s> 底盘平移满速 (默认 300)\n"
        "  --chassis-turn <mrad/s> 底盘转向满速 (默认 1500)\n"
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
        else if (a == "--invert-y")  o.invert_y = true;
        else if (a == "--chassis-speed") o.chassis_speed = next(o.chassis_speed);
        else if (a == "--chassis-turn")  o.chassis_turn = next(o.chassis_turn);
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

    // ---- 鼠标锁定模式 ----
    bool use_grab = (o.mode != "warp");
    bool warping  = false;

    // ⚠️ XGrabPointer 在窗口还没真正显示时会返回 GrabNotViewable(3)。
    //    XMapWindow 只是"请求映射"，必须等 map_state == IsViewable。
    {
        XWindowAttributes wa;
        for (int i = 0; i < 200; ++i) {          // 最多等 2 秒
            XSync(dpy, False);
            if (XGetWindowAttributes(dpy, win, &wa) && wa.map_state == IsViewable)
                break;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    if (use_grab) {
        XSync(dpy, False);
        const int rc = XGrabPointer(dpy, win, True,
                                    PointerMotionMask | ButtonPressMask |
                                        ButtonReleaseMask,
                                    GrabModeAsync, GrabModeAsync, win, None,
                                    CurrentTime);
        if (rc != GrabSuccess) {
            std::printf("[mouse] XGrabPointer 失败 (rc=%d)", rc);
            if (o.mode == "grab") {
                std::printf("；改用 --mode warp\n");
                use_grab = false;
            } else {
                std::printf("；自动降级为 warp 模式\n");
                use_grab = false;
            }
        } else {
            std::printf("[mouse] XGrabPointer 成功\n");
        }
    }
    if (!use_grab) {
        warping = true;
        std::printf("[mouse] 使用 warp 模式：每拍把指针拉回窗口中心\n");
    }

    // 把指针放进窗口中央作为基准
    int cx = o.win_w / 2, cy = o.win_h / 2;
    XWarpPointer(dpy, None, win, 0, 0, 0, 0, cx, cy);
    XSync(dpy, False);

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

    // ---- 底盘状态（WASD 平滑后下发）----
    bool     chassis_en        = o.chassis_en;
    bool     chassis_estop     = false;
    bool     oneshot_chassis_clear = false;
    double   cvx = 0.0, cvy = 0.0, cwz = 0.0;   // 平滑后的 mm/s 与 mrad/s
    int     last_x = cx, last_y = cy;
    double  lost_pkts = 0;
    uint32_t last_echo = 0;

    std::printf("========================================================\n");
    std::printf("  gimbal_ctl 云台鼠标控制\n");
    std::printf("  目标    : %s  UDP:%d  MJPEG:%d\n", o.host.c_str(), o.ctrl_port,
                o.http_port);
    std::printf("  灵敏度  : %.3f °/px   平滑 tau=%.0fms   上限 %.0f°/s\n",
                o.sens, o.tau * 1000, o.max_rate);
    std::printf("  按键    : 鼠标=转云台  空格=停手  e=云台使能  f=底盘使能  x=急停  h=回中\n");
    std::printf("            W/S=前后  A/D=左右  ←/→=底盘转向  c=底盘清零\n");
    std::printf("            z=归零  r=清速率  v=视频开关  Esc/q=退出\n");
    std::printf("  底盘    : 平移 %.0f mm/s  转向 %.0f mrad/s（vx 前进 / vy 左移 / wz 逆时针）\n",
                o.chassis_speed, o.chassis_turn);
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
                case MotionNotify:
                    last_x = ev.xmotion.x;
                    last_y = ev.xmotion.y;
                    break;
                case KeyPress: {
                    const KeySym ks = XLookupKeysym(&ev.xkey, 0);
                    if (ks == XK_Escape || ks == XK_q) running = false;
                    else if (ks == XK_space) hold_mode = !hold_mode;
                    else if (ks == XK_e) { en_state = !en_state;
                                           oneshot |= ctrl_flag::Enable; }
                    else if (ks == XK_f) { chassis_en = !chassis_en;
                                           if (chassis_en) chassis_estop = false;
                                           std::printf("[chassis] 使能=%d\n", chassis_en ? 1 : 0); }
                    else if (ks == XK_c) oneshot_chassis_clear = true;
                    else if (ks == XK_x) { oneshot |= ctrl_flag::EStop;
                                           chassis_estop = true;
                                           std::printf("[chassis] 急停（握手 e/f 恢复）\n"); }
                    else if (ks == XK_h) oneshot |= ctrl_flag::Home;
                    else if (ks == XK_z) oneshot |= ctrl_flag::Zero;
                    else if (ks == XK_r) { rate[0] = rate[1] = 0.0; }
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

        // ---------- 2. 算鼠标增量 ----------
        int dx = 0, dy = 0;
        if (warping) {
            // 位置差 -> 立刻拉回中心
            Window  root_ret, child_ret;
            int     rx, ry, wx, wy;
            unsigned mask;
            if (XQueryPointer(dpy, win, &root_ret, &child_ret, &rx, &ry, &wx, &wy, &mask)) {
                dx = wx - cx;
                dy = wy - cy;
                if (dx != 0 || dy != 0)
                    XWarpPointer(dpy, None, win, 0, 0, 0, 0, cx, cy);
            }
        } else {
            dx = last_x - cx;
            dy = last_y - cy;
            if (dx != 0 || dy != 0)
                XWarpPointer(dpy, None, win, 0, 0, 0, 0, cx, cy);
        }
        XSync(dpy, False);
        last_x = cx;
        last_y = cy;

        // ---------- 3. 角速度：位移速度 × 灵敏度，再一阶低通 ----------
        const double px_per_s_x = dx / dt_target;
        const double px_per_s_y = dy / dt_target;

        auto shape = [&](double pps) {
            if (std::fabs(pps) < o.deadzone) return 0.0;
            return pps * o.sens;
        };

        double target_yaw   = hold_mode ? 0.0 : shape(px_per_s_x);
        double target_pitch = hold_mode ? 0.0 : shape(-px_per_s_y * (o.invert_y ? -1.0 : 1.0));

        target_yaw   = std::max(-o.max_rate, std::min(o.max_rate, target_yaw));
        target_pitch = std::max(-o.max_rate, std::min(o.max_rate, target_pitch));

        const double a = std::min(1.0, dt_target / std::max(1e-3, o.tau));
        rate[0] += (target_yaw   - rate[0]) * a;
        rate[1] += (target_pitch - rate[1]) * a;

        // ---------- 3b. 底盘：键盘 -> 平滑限幅（加速度限幅防打滑/冲击）----------
        double tvx = 0.0, tvy = 0.0, twz = 0.0;
        if (!hold_mode && !chassis_estop) {
            if (keyDown(dpy, XK_w))      tvx += o.chassis_speed;
            if (keyDown(dpy, XK_s))      tvx -= o.chassis_speed;
            if (keyDown(dpy, XK_a))      tvy += o.chassis_speed;   // +y = 左移
            if (keyDown(dpy, XK_d))      tvy -= o.chassis_speed;
            if (keyDown(dpy, XK_Left))   twz += o.chassis_turn;    // +wz = 逆时针
            if (keyDown(dpy, XK_Right))  twz -= o.chassis_turn;
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

        // ---------- 4. 组包发送 ----------
        ControlCommand cc;
        cc.seq       = ++seq;
        cc.yaw_dps   = hold_mode ? 0.0 : rate[0];
        cc.pitch_dps = hold_mode ? 0.0 : rate[1];
        cc.flags     = oneshot;
        oneshot      = 0;
        if (en_state)        cc.flags |= ctrl_flag::Enable;
        else                 cc.flags |= ctrl_flag::Disable;
        if (hold_mode)       cc.flags |= ctrl_flag::Hold;

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

            std::snprintf(line, sizeof(line), "gimbal_ctl -> %s:%d   %s",
                          o.host.c_str(), o.ctrl_port,
                          use_grab ? "[mouse:grab]" : "[mouse:warp]");
            text(line);

            if (hold_mode) text("** HOLD ** (space to resume)");

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

    if (use_grab) XUngrabPointer(dpy, CurrentTime);
    view.stop();
    if (font) XFreeFont(dpy, font);
    XFreeGC(dpy, gc);
    XDestroyWindow(dpy, win);
    XCloseDisplay(dpy);
    std::printf("\n已发送停手指令并退出\n");
    return 0;
}

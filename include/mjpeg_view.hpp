// ===========================================================================
//  MjpegView —— 极简 MJPEG over HTTP 客户端 + libjpeg 解码
//
//  用途: 让 PC 客户端在同一窗口里看到云台摄像头的画面。
//        云台控制必须靠眼睛闭环 —— 没有画面等于盲开。
//
//  实现:
//    · 自己开 TCP，发 HTTP GET /stream
//    · 按 multipart/x-mixed-replace 的 boundary 切出 JPEG
//    · libjpeg 解成 RGB，转成 X11 ZPixmap 需要的字节序
//    · 独立线程收流，主线程只取最新帧（不阻塞控制循环）
//
//  ⚠️ 只支持常见 24/32 位 TrueColor 视觉（红=0xFF0000 绿=0xFF00 蓝=0xFF），
//     绝大多数 X11/XWayland 桌面都是这个。启动时会检查并警告。
// ===========================================================================
#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace ctl {

class MjpegView {
public:
    struct Frame {
        int                  width  = 0;
        int                  height = 0;
        std::vector<uint8_t> bgrx;   ///< X11 ZPixmap 32bpp little-endian
        uint64_t             seq    = 0;
    };

    MjpegView() = default;
    ~MjpegView();

    /// @param host      Jetson 地址（IP 或主机名）
    /// @param port      HTTP 端口（默认 8080）
    /// @param path      流路径（默认 /stream）
    bool start(const std::string& host, int port, const std::string& path = "/stream");
    void stop();

    /// 取最新一帧（拷贝）；无新帧返回 false
    bool getLatest(Frame& out, uint64_t& last_seq);

    bool        connected() const { return connected_.load(); }
    double      fps() const { return fps_.load(); }
    std::string lastError() const;

private:
    void        rxLoop();
    static bool decodeJpeg(const uint8_t* data, size_t len, Frame& out);

    std::thread              th_;
    std::atomic<bool>        run_{false};
    std::atomic<bool>        connected_{false};
    std::atomic<double>      fps_{0.0};
    mutable std::mutex       mu_;
    Frame                    latest_;
    uint64_t                 seq_ = 0;
    std::string              err_;
    std::string              host_;
    int                      port_ = 8080;
    std::string              path_ = "/stream";
};

}  // namespace ctl

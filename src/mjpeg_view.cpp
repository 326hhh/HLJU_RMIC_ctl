#include "mjpeg_view.hpp"

#include <arpa/inet.h>
#include <jpeglib.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <setjmp.h>

namespace ctl {

namespace {

// libjpeg 的默认错误处理会直接 exit()，必须替换成 setjmp 版本
struct JpegErrorMgr {
    struct jpeg_error_mgr pub;
    jmp_buf               jump;
    char                  msg[JMSG_LENGTH_MAX];
};

void onJpegError(j_common_ptr cinfo) {
    auto* e = reinterpret_cast<JpegErrorMgr*>(cinfo->err);
    (*cinfo->err->format_message)(cinfo, e->msg);
    longjmp(e->jump, 1);
}

int connectTcp(const std::string& host, int port, std::string& err) {
    struct addrinfo hints;
    std::memset(&hints, 0, sizeof(hints));
    hints.ai_family   = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    struct addrinfo* res = nullptr;
    const std::string portstr = std::to_string(port);
    if (::getaddrinfo(host.c_str(), portstr.c_str(), &hints, &res) != 0) {
        err = "DNS 解析失败: " + host;
        return -1;
    }

    int fd = -1;
    for (auto* p = res; p; p = p->ai_next) {
        fd = ::socket(p->ai_family, p->ai_socktype, p->ai_protocol);
        if (fd < 0) continue;
        if (::connect(fd, p->ai_addr, p->ai_addrlen) == 0) break;
        ::close(fd);
        fd = -1;
    }
    ::freeaddrinfo(res);
    if (fd < 0) err = "连接 " + host + ":" + portstr + " 失败";
    return fd;
}

}  // namespace

MjpegView::~MjpegView() { stop(); }

std::string MjpegView::lastError() const {
    std::lock_guard<std::mutex> lk(mu_);
    return err_;
}

bool MjpegView::start(const std::string& host, int port, const std::string& path) {
    stop();
    host_ = host;
    port_ = port;
    path_ = path;
    run_  = true;
    th_   = std::thread(&MjpegView::rxLoop, this);
    return true;
}

void MjpegView::stop() {
    run_ = false;
    if (th_.joinable()) th_.join();
    connected_ = false;
}

bool MjpegView::getLatest(Frame& out, uint64_t& last_seq) {
    std::lock_guard<std::mutex> lk(mu_);
    if (seq_ == last_seq || latest_.bgrx.empty()) return false;
    out      = latest_;
    last_seq = seq_;
    return true;
}

// ---------------------------------------------------------------------------
bool MjpegView::decodeJpeg(const uint8_t* data, size_t len, Frame& out) {
    struct jpeg_decompress_struct cinfo;
    JpegErrorMgr                  jerr;
    cinfo.err           = jpeg_std_error(&jerr.pub);
    jerr.pub.error_exit = onJpegError;

    if (setjmp(jerr.jump)) {
        jpeg_destroy_decompress(&cinfo);
        return false;
    }

    jpeg_create_decompress(&cinfo);
    jpeg_mem_src(&cinfo, data, static_cast<unsigned long>(len));
    if (jpeg_read_header(&cinfo, TRUE) != JPEG_HEADER_OK) {
        jpeg_destroy_decompress(&cinfo);
        return false;
    }
    cinfo.out_color_space = JCS_RGB;
    jpeg_start_decompress(&cinfo);

    const int w = static_cast<int>(cinfo.output_width);
    const int h = static_cast<int>(cinfo.output_height);
    if (w <= 0 || h <= 0 || w > 4096 || h > 4096) {
        jpeg_destroy_decompress(&cinfo);
        return false;
    }

    out.width  = w;
    out.height = h;
    out.bgrx.assign(static_cast<size_t>(w) * static_cast<size_t>(h) * 4, 0);

    std::vector<uint8_t> row(static_cast<size_t>(w) * 3);
    while (cinfo.output_scanline < cinfo.output_height) {
        uint8_t* rows[1] = {row.data()};
        const JDIMENSION y = cinfo.output_scanline;
        jpeg_read_scanlines(&cinfo, rows, 1);
        uint8_t* dst = out.bgrx.data() + static_cast<size_t>(y) * w * 4;
        for (int x = 0; x < w; ++x) {
            dst[x * 4 + 0] = row[static_cast<size_t>(x) * 3 + 2];  // B
            dst[x * 4 + 1] = row[static_cast<size_t>(x) * 3 + 1];  // G
            dst[x * 4 + 2] = row[static_cast<size_t>(x) * 3 + 0];  // R
            dst[x * 4 + 3] = 0;                                    // X
        }
    }

    jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);
    return true;
}

// ---------------------------------------------------------------------------
void MjpegView::rxLoop() {
    while (run_) {
        std::string err;
        int         fd = connectTcp(host_, port_, err);
        if (fd < 0) {
            {
                std::lock_guard<std::mutex> lk(mu_);
                err_ = err;
            }
            connected_ = false;
            for (int i = 0; i < 20 && run_; ++i)
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        int one = 1;
        ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

        const std::string req = "GET " + path_ +
                                " HTTP/1.0\r\nHost: " + host_ +
                                "\r\nConnection: close\r\n\r\n";
        if (::send(fd, req.data(), req.size(), 0) < 0) {
            ::close(fd);
            continue;
        }
        {
            std::lock_guard<std::mutex> lk(mu_);
            err_.clear();
        }
        connected_ = true;
        std::fprintf(stderr, "[mjpeg] 已连接 %s:%d%s\n", host_.c_str(), port_, path_.c_str());

        std::vector<uint8_t> buf;
        buf.reserve(1 << 20);
        uint8_t      chunk[16384];
        uint64_t     frame_count = 0;
        auto         t_win       = std::chrono::steady_clock::now();
        uint64_t     hdr_done    = 0;

        size_t total_rx = 0;
        while (run_) {
            const ssize_t n = ::recv(fd, chunk, sizeof(chunk), 0);
            if (n <= 0) break;
            total_rx += static_cast<size_t>(n);
            buf.insert(buf.end(), chunk, chunk + n);

            // 跳过 HTTP 头
            if (hdr_done == 0) {
                const char* pat = "\r\n\r\n";
                for (size_t i = 0; i + 4 <= buf.size(); ++i) {
                    if (std::memcmp(&buf[i], pat, 4) == 0) {
                        hdr_done = i + 4;
                        break;
                    }
                }
                if (hdr_done == 0) continue;
                buf.erase(buf.begin(), buf.begin() + static_cast<long>(hdr_done));
            }

            // 在缓冲里找每张 JPEG：SOI(FFD8) ... EOI(FFD9)
            for (;;) {
                size_t soi = 0, eoi = 0;
                bool   ok  = false;
                for (size_t i = 0; i + 1 < buf.size(); ++i) {
                    if (buf[i] == 0xFF && buf[i + 1] == 0xD8) {
                        for (size_t j = i + 2; j + 1 < buf.size(); ++j) {
                            if (buf[j] == 0xFF && buf[j + 1] == 0xD9) {
                                soi = i; eoi = j + 2; ok = true; break;
                            }
                        }
                        if (ok) break;
                        // 只有 SOI 没有 EOI：说明这帧还没收全
                        break;
                    }
                }
                if (!ok) {
                    // 丢掉 SOI 之前的填充字节，避免缓冲无限增长
                    size_t cut = 0;
                    while (cut + 1 < buf.size() &&
                           !(buf[cut] == 0xFF && buf[cut + 1] == 0xD8))
                        ++cut;
                    if (cut > 0) buf.erase(buf.begin(), buf.begin() + static_cast<long>(cut));
                    break;
                }

                Frame f;
                if (decodeJpeg(buf.data() + soi, eoi - soi, f)) {
                    std::lock_guard<std::mutex> lk(mu_);
                    latest_ = std::move(f);
                    ++seq_;
                    ++frame_count;
                    if (seq_ <= 2 || seq_ % 100 == 0)
                        std::fprintf(stderr, "[mjpeg] 第 %llu 帧 %dx%d (%zu 字节 JPEG)\n",
                                     (unsigned long long)seq_, latest_.width,
                                     latest_.height, eoi - soi);
                } else {
                    std::fprintf(stderr, "[mjpeg] JPEG 解码失败 (%zu 字节)\n", eoi - soi);
                }
                buf.erase(buf.begin(), buf.begin() + static_cast<long>(eoi));

                const auto now = std::chrono::steady_clock::now();
                const double dt =
                    std::chrono::duration<double>(now - t_win).count();
                if (dt >= 1.0) {
                    fps_    = frame_count / dt;
                    frame_count = 0;
                    t_win   = now;
                }
            }

            if (buf.size() > (4u << 20)) buf.clear();   // 异常保护
        }

        std::fprintf(stderr, "[mjpeg] 断开 (共收 %zu 字节)\n", total_rx);
        ::close(fd);
        connected_ = false;
        if (run_) {
            for (int i = 0; i < 10 && run_; ++i)
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }
}

}  // namespace ctl

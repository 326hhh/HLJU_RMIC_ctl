# include —— gimbal_ctl 公共头文件

本目录放**公共头文件**（C++，`.hpp`），`src/` 下的源码通过 `#include "<name>.hpp"` 引用。
根目录 [`../README.md`](../README.md) 是使用说明（编译/运行/按键/模式），本文只讲目录与文件本身。

## 目录结构

```
include/
├── mjpeg_view.hpp    # MjpegView 类声明：MJPEG over HTTP 拉流 + libjpeg 解码 + 最新帧缓存
└── README.md
```

## 文件说明

### `mjpeg_view.hpp` —— MJPEG 视频视图（只声明）

- **作用**：让 PC 客户端在同一个 X11 窗口里看到云台摄像头画面 ——
  云台控制必须靠眼睛闭环，没有画面等于盲开。
- **声明了什么**：
  - `struct Frame`：一帧解码后的图像
    （`width` / `height` / `bgrx`：按 **X11 ZPixmap 32bpp 小端**排好的像素 / `seq`：帧序号）。
  - `class MjpegView` 对外接口：
    | 成员 | 作用 |
    |---|---|
    | `start(host, port, path="/stream")` | 起后台线程连 `http://host:port/path` 收流 |
    | `stop()` | 停线程、断开 |
    | `getLatest(Frame&, uint64_t& last_seq)` | 取最新一帧（拷贝）；没有新帧返回 `false` |
    | `connected()` / `fps()` / `lastError()` | 连接状态 / 拉流帧率 / 最近错误 |
  - **私有实现**：`rxLoop()` 收流线程、`decodeJpeg()` 静态解码、
    线程/原子量/互斥锁/`latest_` 帧缓存（双缓冲，主线程只拿最新帧、从不阻塞）。
- **用法要点**：`start()` 后云台视频在后台线程跑，主线程按需 `getLatest()` 取帧绘制；
  模块线程与主线程之间只通过 `mutex` 保护的 `latest_` 交互。
- **限制**：只支持常见 24/32 位 TrueColor 视觉（红=`0xFF0000` 绿=`0xFF00` 蓝=`0xFF`），
  绝大多数 X11/XWayland 桌面都是这个；不匹配时调用方应退化为"只显示状态文字"。
- **实现**：见 [`../src/README.md`](../src/README.md)（`src/mjpeg_view.cpp`）。

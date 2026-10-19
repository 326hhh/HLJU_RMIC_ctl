# src —— gimbal_ctl 源码

本目录放**实现源码**（C++，`.cpp`），构建入口是根目录 `CMakeLists.txt`，
使用说明（编译/运行/按键/模式）见根目录 [`../README.md`](../README.md)。

## 目录结构

```
src/
├── main.cpp         # 主程序：X11 窗口/鼠标抓取 + UDP 控制链路 + 遥测解析 + 状态渲染 + 自检
├── mjpeg_view.cpp   # 视频实现：TCP 拉 MJPEG、按 SOI/EOI 切帧、libjpeg 解码、帧缓存
└── README.md
```

## 文件说明

### `main.cpp` —— 主程序（720 行）

程序职责：**抓鼠标 → 算角速度 → 打包 UDP 发给 Jetson → 收遥测 → 渲染**，全程单线程主循环。

- **命令行解析**：`--host`（默认 `192.168.2.2`）、`--ctrl-port`（默认 40001）、`--http-port`
  （8080）、`--no-video`、`--mode auto|grab|warp`、`--sens`（°/px）、`--tau`（平滑 s）、
  `--max-rate`（°/s 上限）、`--invert-y`、`--size w h`、`--hz`（发送频率）、
  `--chassis-speed`（底盘平移满速 mm/s）、`--chassis-turn`（底盘转向满速 mrad/s）、`--selftest`。
- **`UdpClient`**：`open()` 用 `getaddrinfo` + `connect()` 建 UDP 通道；`send()` 把
  `ControlCommand` 编码后发 UDP，同时记录 `seq` 的发送时刻用于 RTT；`recv()` 非阻塞收遥测
  （`MSG_DONTWAIT`），解码失败计入坏包数，按 `echo_seq` 算 RTT。
- **`runSelftest()`**：不开窗口的链路自检 —— 上行编解码往返、v2 底盘字段往返、篡改包被拒、
  UDP socket 建立、发 50 包等遥测回包、坏包计数。
- **`main()` 主流程**：
  1. **X11 初始化**：开 display、建 960×540 窗口、检查 TrueColor 像素格式（决定能否直写视频）。
  2. **鼠标模式**（Wayland/XWayland 兼容）：
     - `grab`：`XGrabPointer` 把指针锁在窗口内（X11 原生会话最干净）；
       会先等窗口真正 `IsViewable` 再抓（否则 `GrabNotViewable`）；
     - `warp`：每拍 `XWarpPointer` 拉回窗口中心，用位置差算增量（**XWayland 下必定可用**）；
     - `auto`（默认）：先试 grab，失败自动降级 warp。
  3. **鼠标 → 角速度**：位置增量 / 帧周期 = 像素速度（px/s）→ 死区（`--deadzone`）→
     ×灵敏度 → 一阶低通（`--tau`）→ 限幅 `--max-rate`；`空格` HOLD 时置 0。
  4. **底盘 WASD**：`W/S` 前后、`A/D` 左右、`←/→` 转向；速度目标再经**加速度限幅**
     （平移 3000 mm/s²、转向 6 rad/s²）平滑，防打滑/冲击；`HOLD` 或底盘急停时目标为 0。
  5. **组包发送**：填 `ControlCommand`（yaw/pitch 角速度、云台 flags、底盘 vx/vy/wz 与
     chassis flags、一次性 `Home/Zero/EStop/Clear`），`udp.send()`；发送频率 `--hz`（默认 100 Hz）。
  6. **收遥测**：`ControlTelemetry` 解析后更新丢包统计（`echo_seq` 跳变）、RTT、状态。
  7. **渲染**（约 30 fps，独立于控制节拍）：视频帧居中 `XPutImage`（`img->data=nullptr`
     防止 XDestroyImage 释放缓存），再叠状态文字：云台指令/实际/输出角、电机转速/电压/温度、
     status 异常标志（急停/堵转/看门狗/电机失联）、链路 Rx/RTT/丢包、底盘位姿/四轮转速/陀螺 yaw、
     视频 fps。
  8. **定频**：`sleep_until` 对齐目标周期（`t_next` 累加，落后则重置）。
  9. **收尾**：退出前连发 3 帧 `Hold` 停止指令，避免云台保持最后的速率。
- **按键**：鼠标=转云台、空格=HOLD、`e`=云台使能/失能、`f`=底盘使能、`x`=急停（云台+底盘）、
  `h`=云台回中、`z`=当前位置归零、`r`=清空速率、`c`=底盘里程计清零、`v`=视频开关、`Esc`/`q`=退出。
- **线协议**：不在本工程里定义，**直接复用** `../gimbal/include/gimbal/link/control_link.hpp`
  （上行 24 B / 下行 64 B，小端，显式字节偏移，两端逐字节一致）。

### `mjpeg_view.cpp` —— MJPEG 视频实现（267 行）

对应声明见 [`../include/README.md`](../include/README.md)（`MjpegView` 类）。

- **连接**：`connectTcp()` 用 `getaddrinfo` 遍历地址族建 TCP，`TCP_NODELAY` 关 Nagle；
  发 `GET /stream HTTP/1.0` 请求；断线自动重连（20×50ms 退避）。
- **收流线程 `rxLoop()`**：独立线程 `recv()` 进缓冲，先跳过 HTTP 响应头（找 `\r\n\r\n`），
  再按 JPEG 的 **SOI(`FFD8`) … EOI(`FFD9`)** 从字节流里切出完整帧
  （multipart/x-mixed-replace 的 boundary 不必解析，靠 SOI/EOI 更稳），
  缓冲超过 4 MB 异常保护清空。
- **解码 `decodeJpeg()`**：libjpeg 解码为 RGB；**替换默认错误处理为 `setjmp/longjmp`**
  （libjpeg 默认 error_exit 会 `exit()` 整个进程）；限制尺寸 ≤ 4096；
  逐行转成 X11 ZPixmap 需要的 **BGRX（32bpp 小端）**，写完放入 `latest_` 并递增 `seq_`。
- **帧率统计**：每秒窗口内统计解码帧数写入 `fps_`。
- **线程模型**：解码线程持锁更新 `latest_`；主线程 `getLatest()` 加锁拷贝最新帧，
  按 `seq` 判断是否有新帧，**渲染永不阻塞控制循环**。

## 构建关系

`CMakeLists.txt` 只编译 `src/main.cpp` + `src/mjpeg_view.cpp`，头文件目录加 `include/` 与
`${GIMBAL_ROOT}/include`（复用 gimbal 工程的线协议定义）；链接 X11 + libjpeg。

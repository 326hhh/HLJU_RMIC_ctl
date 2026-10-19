# gimbal_ctl —— 云台鼠标控制客户端（PC 端）

原生 X11 小程序：**抓鼠标 → 算角速度 → UDP 发给 Jetson → 显示遥测 + MJPEG 视频**。

## 目录结构

```
gimbal_ctl/
├── CMakeLists.txt   # 构建脚本：C++17、X11 + libjpeg，线协议头文件复用 ../gimbal
├── README.md        # 本文件：使用说明与目录/文件说明
├── LICENSE          # MIT License（Copyright (c) 2026 saki）
├── .gitignore       # 忽略 build/、*.o、*.log、.vscode/、.DS_Store
├── include/         # 公共头文件（mjpeg_view.hpp），见 include/README.md
├── src/             # 实现源码（main.cpp、mjpeg_view.cpp），见 src/README.md
└── build/           # CMake 构建输出目录（自动生成，已 gitignore，可删）
```

## 根目录文件说明

| 文件 | 作用 |
|---|---|
| `CMakeLists.txt` | 工程构建脚本：C++17、Release 默认；`find_package(X11)` + `pkg_check_modules(libjpeg)`；把 `include/` 与 `${GIMBAL_ROOT}/include` 加入头文件目录（`GIMBAL_ROOT` 默认 `../gimbal`，可用 `-DGIMBAL_ROOT=<路径>` 覆盖）；编译 `src/main.cpp` + `src/mjpeg_view.cpp`，链接 X11 与 libjpeg。 |
| `README.md` | 本文件。使用说明（编译/运行/按键/鼠标模式/视频/协议）见下文；目录与文件级说明见 `include/README.md`、`src/README.md`。 |
| `LICENSE` | MIT 许可证。 |
| `.gitignore` | 忽略 `build/`、`*.o`、`*.log`、`.vscode/`、`.DS_Store`。 |
| `include/` | 公共头文件（C++ `.hpp`）。 |
| `src/` | 源码实现（`.cpp`）。 |
| `build/` | 构建产物目录（CMake 生成），不入库。 |

## 编译

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
```

依赖：X11（`libx11-dev`）、libjpeg（`libjpeg-dev`）。
线协议直接复用 `../gimbal/include/gimbal/link/control_link.hpp` —— **只有一个源头，不复制**。
gimbal 工程不在 `../gimbal` 时用 `-DGIMBAL_ROOT=<路径>`。

> 💡 想用 clangd/IDE 索引不报“找不到头文件”，配置时加上
> `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON`，生成 `build/compile_commands.json` 即可。

## 运行

```bash
# 正常使用
./build/gimbal_ctl --host 192.168.2.2

# Wayland 下如果抓不住鼠标
./build/gimbal_ctl --host 192.168.2.2 --mode warp

# 只要鼠标控制，不要视频
./build/gimbal_ctl --host 192.168.2.2 --no-video

# 只验 UDP 链路（不开窗口）
./build/gimbal_ctl --selftest --host 192.168.2.2
```

## 鼠标映射（角速度模式，像 FPS 视角）

```
鼠标移动速度 (px/s) × 灵敏度 (°/px)  =  目标角速度 (°/s)
                       ↓ 一阶低通（--tau，默认 50ms）
                    实际发给 Jetson 的值
```

- **动鼠标 → 云台转；停手 → 平滑停住**
- `--sens 0.09`：鼠标以 1000 px/s 移动 → 云台 90°/s
- `--tau 0.05`：平滑时间常数，调大更"黏"，调小更"贼"
- `--invert-y`：鼠标上下反向（看你的机械装配方向）

## 按键

| 键 | 作用 |
|---|---|
| 移动鼠标 | 转云台 |
| **空格** | 停手 / 继续（HOLD 开关） |
| `e` | 云台使能 / 失能切换 |
| `f` | 底盘使能 / 失能切换 |
| `x` | **急停**（云台 + 底盘，立即停止 + 失能） |
| `h` | 云台回中 |
| `z` | 把当前位置当作 0° |
| `r` | 清空当前速率（不回中，只把速度清零） |
| `c` | 底盘里程计清零 |
| `W/S` | 底盘前进 / 后退（满速 `--chassis-speed`，默认 300 mm/s） |
| `A/D` | 底盘左移 / 右移 |
| `←/→` | 底盘左转 / 右转（满速 `--chassis-turn`，默认 1500 mrad/s） |
| `v` | 视频开关 |
| `Esc` / `q` | 退出（退出前会自动发一帧停止指令） |

> 底盘坐标约定：`vx` 前进 / `vy` 左移 / `wz` 逆时针为正。

## ⚠️ Wayland 下的鼠标抓取

当前会话是 Wayland 时，本程序跑在 **XWayland** 里，`XGrabPointer` 的"锁定"
能否穿透到合成器**取决于 XWayland 版本，不保证**。所以提供三种模式：

| 模式 | 做法 | 适用 |
|---|---|---|
| `grab` | `XGrabPointer` 把指针限制在窗口内 | X11 会话下最干净 |
| **`warp`** | 每拍把指针 `XWarpPointer` 拉回窗口中心，用位置差算增量 | **XWayland 下必定可用** |
| `auto`（默认） | 先试 grab，失败自动降级 warp | 不知道用哪个时就用它 |

启动时会打印实际用了哪种：
```
[mouse] XGrabPointer 成功
[mouse] XGrabPointer 失败 (rc=1)；自动降级为 warp 模式
```

**如果两种模式手感都不对**，说明当前合成器对 XWayland 支持有限，那就回到浏览器方案
（浏览器的 Pointer Lock 由合成器原生实现，一定可用），把 Jetson 的 UDP 链路当成后端。

按住 `Esc` 一定能退出（grab 模式下鼠标被锁住，只能靠键盘）。

## 视频

从 `http://<host>:8080/stream` 拉 MJPEG，libjpeg 解码后居中画在窗口里。
540p 窗口放 640×480 的画面刚好。

⚠️ 只支持常见 24/32 位 TrueColor 视觉（红=0xFF0000 绿=0xFF00 蓝=0xFF）。
启动时会检查，不匹配就只显示状态文字不显示画面。

## 状态显示

```
gimbal_ctl -> 192.168.2.2:40001   [鼠标:grab]
yaw   指令   +90.0   实际   +90.0   输出角   +1234.56
pitch 指令    +0.0   实际    +0.0   输出角     -12.34
电机  +30.0 /   +0.0 RPM   11.20V  27C   status=0x07
链路 100Hz 丢 0  RTT 0.8ms   正常   rx=100Hz
底盘 位姿  +1000,+500 mm  yaw +45.00°  指令 +300,+0 mm/s
     轮速 FL +100.0 FR +100.0 BL +100.0 BR +100.0 RPM
     实际 +300,+0 mm/s  陀螺yaw +45.00°  底盘在线
视频 14.2 fps 已连接
```

**status 异常时会直接标出来**：`急停` / `堵转!` / `看门狗断流!` / `电机失联!`
—— 开着车的时候你不该去数字段。

## 详细文档索引

| 想看什么 | 去哪 |
|---|---|
| 公共头文件（`MjpegView` 类）说明 | [`include/README.md`](include/README.md) |
| 源码（`main.cpp` / `mjpeg_view.cpp`）说明 | [`src/README.md`](src/README.md) |
| 线协议（上行/下行逐字节格式） | `../gimbal/include/gimbal/link/control_link.hpp` |
| 云台控制端（Jetson 侧） | `../gimbal` 工程 |

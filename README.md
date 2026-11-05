# gimbal_ctl —— 云台/底盘控制客户端（PC 端）

原生 X11 小程序：**方向键 / WASD 输入 → 换算指令 → UDP 发给 Jetson → 显示遥测 + MJPEG 视频**。

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
| `README.md` | 本文件。使用说明（编译/运行/按键/视频/协议）见下文；目录与文件级说明见 `include/README.md`、`src/README.md`。 |
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

两种连法，**只差 `--host`**：有线（网线直连 `192.168.2.2`）/ 无线（Jetson 热点 `10.42.0.1`）。

### 有线运行（笔记本 ↔ Jetson 网线直连）

笔记本 `enp8s0 192.168.2.1` ↔ Jetson `eth0 192.168.2.2`，两边都是静态地址，开机即用。

```bash
./build/gimbal_ctl --host 192.168.2.2                      # 控制 + 视频（默认）
./build/gimbal_ctl --host 192.168.2.2 --gimbal-rate 150    # 方向键云台更快（默认 120°/s）
./build/gimbal_ctl --host 192.168.2.2 --no-video           # 只要控制，不要视频
./build/gimbal_ctl --selftest --host 192.168.2.2           # 只验 UDP 链路（不开窗口）
```

### 无线运行（笔记本 ↔ Jetson 热点）

**Jetson 侧**先起热点 —— `car-ap` 是 `autoconnect=no`，每次开机要手动拉：

```bash
sudo nmcli con up car-ap        # SSID MECAR-CTRL / 密码 mecar12345 / 10.42.0.1/24
```

**笔记本侧**：

```bash
sudo nmcli device wifi connect MECAR-CTRL password mecar12345
ip -br addr show wlp14s0        # 应拿到 10.42.0.x

# Meta(Clash) 会把这个网段塞进隧道，先查：
ip route get 10.42.0.1          # 必须是 dev wlp14s0；若是 dev Meta 就加绕行规则
sudo ip rule add to 10.42.0.0/24 lookup main priority 8000

./build/gimbal_ctl --host 10.42.0.1
./build/gimbal_ctl --host 10.42.0.1 --no-video    # 无线带宽紧张时先关视频
```

- 车要走位时用无线；**开热点时 Jetson 自己就连不了别的 WiFi**（网卡只能干一件事），有线链路不受影响；
- **笔记本 Wi-Fi 省电必须关**，否则 ping 的 RTT 会到 333~488 ms、抖动很大（控车是 100 Hz UDP）：
  `nmcli con mod <MECAR-CTRL 配置名> 802-11-wireless.powersave 2`
  （这个 NetworkManager 版本 `0=default` **不会**关省电，`2` 才是 disable）；
- 图传 640x480@30 约 2 Mbps，热点够用；1280x720@30 约 8~15 Mbps 会吃紧。

> `ip rule` 是临时的，每次重连 WiFi / 重启 VPN 后都要重新加；切回校园网时删掉它：
> `sudo ip rule del to 10.42.0.0/24 lookup main priority 8000`

## 云台控制（方向键）

```
按住 ←/→ = yaw 左/右   ↑/↓ = pitch 抬头/低头
       速率 --gimbal-rate（默认 120 °/s）
              ↓ 一阶低通（--tau，默认 45ms）
           实际发给 Jetson 的角速度
```

- **按住方向键 → 云台按固定角速度转；松开 → 平滑停住**
- `--gimbal-rate 120`：方向键速率，调大更"能打"，调小更"稳"
- `--max-rate 180`：角速度上限
- `--tau 0.045`：平滑时间常数
- 鼠标不再参与云台控制（指针不再被锁定，可以正常点窗口）

## 按键

| 键 | 作用 |
|---|---|
| `←/→` | 云台 yaw 左 / 右（按住转，松开停） |
| `↑/↓` | 云台 pitch 抬头 / 低头 |
| **空格** | 停手 / 继续（HOLD 开关，云台 + 底盘 + 发射一起停） |
| `g` | 云台使能 / 失能切换 |
| `j` | 底盘使能 / 失能切换（**底盘急停后按 `j` 恢复**） |
| **`F`（按住）** | **水弹发射**（按住连发，松开停；HOLD 时不响应） |
| `x` | **急停**（云台 + 底盘，立即停止 + 失能） |
| `h` | 云台回中 |
| `z` | 把当前位置当作 0° |
| `r` | 清空当前速率（不回中，只把速度清零） |
| `c` | 底盘里程计清零 |
| `W/S` | 底盘前进 / 后退（满速 `--chassis-speed`，默认 300 mm/s） |
| `A/D` | 底盘左移 / 右移 |
| `Q/E` | 底盘左转 / 右转（满速 `--chassis-turn`，默认 1500 mrad/s） |
| `v` | 视频开关 |
| `Esc` | 退出（退出前会自动发一帧停止指令） |

> 底盘坐标约定：`vx` 前进 / `vy` 左移 / `wz` 逆时针为正。

## 关于鼠标

云台已改为方向键控制后，**鼠标不再参与控制、指针也不再被锁定**——
`--mode grab|warp` 参数保留但不再起作用（向后兼容旧命令行）。

按 `Esc` 退出（退出前会自动发一帧停止指令）。

## 视频

从 `http://<host>:8080/stream` 拉 MJPEG，libjpeg 解码后居中画在窗口里。
540p 窗口放 640×480 的画面刚好。

⚠️ 只支持常见 24/32 位 TrueColor 视觉（红=0xFF0000 绿=0xFF00 蓝=0xFF）。
启动时会检查，不匹配就只显示状态文字不显示画面。

## 状态显示

```
gimbal_ctl -> 192.168.2.2:40001   [keys: arrows]
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

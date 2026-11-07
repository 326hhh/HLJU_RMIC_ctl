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

## Windows 版（`win/`）

同一套控制逻辑的 **Windows 原生客户端**（静态链接 exe，拷过去双击就能跑，无需装运行库），
源码在 `win/`，使用说明见 [`win/readme.txt`](win/readme.txt)。

```bash
# 在 Linux 上交叉编译（默认用 zig，找不到则用 mingw-w64 g++）
cd win && ./build_win.sh                    # 产物 win/build/gimbal_ctl.exe
# 或指定 gimbal 工程路径
GIMBAL_ROOT=~/gimbal ./build_win.sh
```

- 配置：`win/gimbal_ctl.ini`（host / 速度 / 平滑 / 小陀螺速率等，都有中文注释）；
- 启动：双击 `start.cmd`（或 `gimbal_ctl.exe --selftest` 先自检）；
- 键位与 Linux 版一致（云台方向键、WASD、Shift 快速档、R 小陀螺、F 发射）。
- Windows 下没有 X11，窗口/键鼠用 Win32 API，视频解码用 `stb_image`。

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
| `k` | 清空当前速率（不回中，只把速度清零；**原 `r` 键，已被小陀螺占用**） |
| `c` | 底盘里程计清零 |
| `W/S` | 底盘前进 / 后退（满速 `--chassis-speed`，默认 300 mm/s） |
| `A/D` | 底盘左移 / 右移 |
| `Q/E` | 底盘左转 / 右转（满速 `--chassis-turn`，默认 1500 mrad/s；小陀螺模式下让位给自旋） |
| **Shift** | **按一下切换「快速/普通」档（锁存，不用按住）**，详见下文 |
| **`R`** | **切换小陀螺模式**（底盘自旋 + 云台反向站住），详见下文 |
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
** SPIN cmd +2.00  act +1.85 rad/s ** yaw comp -106 deg/s (R)   <- 小陀螺开着才有
** FAST ** 600 mm/s  3.0 rad/s (shift toggled)                 <- 快速档才有
yaw    cmd   +90.0  act   +88.5  out   +1234.56 deg
pitch  cmd    +0.0  act    +0.0  out     -12.34 deg
motor  +30.0 /  +0.0 RPM  24.10V  27C  status=0x07
link 100Hz lost 0  RTT 0.8ms  OK  rx=100Hz
chassis pos +1000,   +500 mm  yaw  +45.00  cmd  +300,   +0 mm/s
        wheel FL+100.0 FR+100.0 BL+100.0 BR+100.0 RPM
        act  +300,    +0 mm/s  gyro_yaw  +45.00  chassis ONLINE
video 14.2 fps connected
```

**status 异常时会直接标出来**：`ESTOP` / `STALL!` / `WATCHDOG!` / `MOTOR LOST!`，
以及 `** GIMBAL DISABLED/LOCKED **`（云台被失能/锁死，按 `g` 两下重新授权）
—— 开着车的时候你不该去数字段。

## 快速模式（Shift 键）

**按一下 Shift（左右均可）切换，锁存**；再按一下切回普通档。松开 Shift 不会退回。

| 档位 | 平移满速 | 转向满速 |
|---|---|---|
| 普通 | `--chassis-speed` 默认 300 mm/s | `--chassis-turn` 默认 1500 mrad/s |
| 快速 | `--shift-speed` 默认 600 mm/s | `--shift-turn` 默认 3000 mrad/s |

- 加速度斜坡不变（平移 3000 mm/s²、转向 6000 mrad/s²），所以切换瞬间不会窜；
- 快速档生效时状态栏多一行 `** FAST ** 600 mm/s  3.0 rad/s (shift toggled)`；
- 终端会打一行 `[chassis] 切到快速档：平移 600 mm/s / 转向 3000 mrad/s`。

> 实现用 `XQueryKeymap` 轮询做**上升沿检测**，不是靠 `KeyPress` 事件 ——
> X11 对长按的键会连发 KeyPress，用事件法会在按住期间疯狂来回切档。

## 小陀螺模式（R 键）

参考 RM 正赛：**底盘原地自旋，云台（相机）在世界坐标系里站住不转**，方向键照常控制云台。

```
按一下 R -> 底盘按 --spin-rate 自旋，yaw 电机反向补偿
方向键   -> 叠加在补偿之上，仍然控制云台（世界系）
再按 R   -> 退出，底盘平滑停转（减速段继续补偿，相机不会被甩）
```

| 参数 | 默认 | 说明 |
|---|---|---|
| `--spin-rate <mrad/s>` | `2000`（2.0 rad/s ≈ 115°/s） | 自旋速率，**正 = 逆时针**；取负值 = 反向自旋 |
| `--spin-gain <f>` | `1.0` | yaw 补偿倍率；实机发现相机"转得更快"（补偿同向叠加）时填 `-1` 翻转 |

### 补偿量怎么算的

```cpp
comp（°/s） = -spin_gain × wz（rad/s → °/s）
cc.yaw_dps  = 方向键速率 + comp
```

`wz` 的取法（**这里是踩过坑的地方**）：

1. **以指令值 `cwz` 为基准**；
2. 只有底盘陀螺**实测** wz 满足「与指令同号 且 ≥ 指令的 25%」时才改用实测值 ——
   这样地面打滑、掉速能自动吸掉；
3. 否则退回指令值。**原因**：陀螺坏掉/轴映射不对时会恒报 0，
   无脑采信实测会让补偿直接变成 0，表现就是"云台跟着车一起转"。

实测值还会过 **30 ms 一阶低通**压噪声；关掉小陀螺后 **0.6 s 内继续补偿**，
因为底盘还要约 0.33 s 才停转，不补的话相机会被甩约 19°。
按空格 HOLD 时底盘自旋会停，但补偿仍然生效（相机继续站住）。

状态栏（`act` 是低通后的实测自旋速率，`comp` 是补偿量）：

```
** SPIN cmd +2.00  act +1.85 rad/s ** yaw comp -106 deg/s (R)
```

### 已知限制

| 限制 | 数值 / 原因 |
|---|---|
| **yaw 必须能无限连续旋转** | 小陀螺期间 yaw 相对底盘一直转。若电机线/电源线没走导电滑环，会被绞紧 → 转半圈到一圈后**堵转保护**（电机状态位 `0x6B`），云台直接停住。**这是硬性条件** |
| 轮速天花板 | 自旋 2 rad/s 约占 105 RPM，快速档平移 600 mm/s 约占 143 RPM，最坏轮 ≈248 RPM 顶到 250 上限 → **小陀螺时用普通档平移** |
| 云台转速上限 | Jetson 云台默认 `--yaw-rate 180°/s`。自旋 3.0 rad/s 时补偿就吃掉 172°/s，方向键几乎没余量；要边转边瞄，Jetson 端起时加 `--yaw-rate 300` |
| 打滑补偿依赖实测陀螺 | 底盘反馈帧里的 `gyro_wz` 若恒为 0（IMU 故障），只能按指令值补偿，打滑和掉速补偿不了 |

## 现场排障手册

小陀螺/云台出问题时，按这张表查（**都是实际踩过的**）：

| 现象 | 检查 | 处理 |
|---|---|---|
| 窗口显示 `** GIMBAL DISABLED/LOCKED **` | 遥测 `status` 的 bit0=0，云台被失能/急停锁死 | **按 `g` 关再开（两下）**重新授权 |
| yaw 完全不动，但底盘正常 | Jetson 会话卡在 `kLocked`（等使能/位置反馈/启动抬升）时**整段不下发速率** | 先按 `g` 两下；仍不行看 Jetson 的 `~/gimbal_loop.log` |
| 电机状态 `0x6B` | bit3 = **堵转保护**触发，电机会拒绝执行速度命令 | `cansend can0 00000100#0E526B`（yaw）/ `...00000200#0E526B`（pitch）清除；客户端重新使能时也会自动清一次 |
| yaw 转半圈就停、状态 `0x6B` | 线缆被绞紧（没走滑环） | **机械问题**，刷软件没用，得改走线 |
| 底盘陀螺 `act` 一直 +0.00 | 底盘反馈帧 `gyro_wz` 恒 0（IMU 故障/轴映射不对） | 查底盘 LSM6DSV16X 的 I2C 与零偏，与本工程无关 |

### 直接查电机（Jetson 上）

```bash
# 查状态：返回 3A 01 6B = 使能正常；3A 08 6B 之类含 bit3 = 堵转保护
cansend can0 00000100#3A6B      # yaw（地址 1）
cansend can0 00000200#3A6B      # pitch（地址 2）

# 抓总线看谁在发命令（ZDT 电机帧是扩展帧，ID = 地址<<8，如 0x100/0x200）
candump -L can0 | grep -E "00000100|00000200"

# 观察器（本机临时工具，可能已被清理）：每 2 秒打印
#   底盘 wz 命令 / 陀螺实测 / yaw 速度命令 / 电机状态 / 四轮转速
bash ~/turbo_selftest/spin_watch.sh 90
```

> ⚠️ 抓包诊断两个坑：① ZDT 的 CAN ID（如 `0x100`）**数值小于 0x7FF**，
> 不能按"ID > 0x7FF 才是扩展帧"判断，要看 `candump -L` 打印成 8 位还是 3 位；
> ② 只在"数值变化"时打印会骗人——进程死了和空闲看起来一样，**必须打心跳/帧龄**。

## 详细文档索引

| 想看什么 | 去哪 |
|---|---|
| 公共头文件（`MjpegView` 类）说明 | [`include/README.md`](include/README.md) |
| 源码（`main.cpp` / `mjpeg_view.cpp`）说明 | [`src/README.md`](src/README.md) |
| 线协议（上行/下行逐字节格式） | `../gimbal/include/gimbal/link/control_link.hpp` |
| 云台控制端（Jetson 侧） | `../gimbal` 工程 |

# 08 · IPC、robotctl 与你的改造切入点

> 目的：本讲收官，做两件事。**(1) 从"外面"看这台机器人**——`duck-ipc-proto` 定义了哪些
> 上线消息，`robotctl` 怎么通过 unix socket 跟 `robotd` 说话，`monitor`/`health` 显示什么；
> **(2) 把你在 `~/microduck_rl` 里评估过的两件改造——14 个 XL330 换飞特 HL-2909、IMU 从
> 伺服总线上拿下来直连——落成 `~/microduck` 的"下刀清单"**。前 7 讲教你读懂 20 ms 那一拍，
> 这一讲回答：改那一拍要从哪几个文件动刀，哪些是搬运、哪些是重写。

---

## 1. duck-ipc-proto：机器人与外面唯一的"线协议"

### 1.0 这是什么 crate

`duck-ipc-proto` 是**只定义线格式的 crate**——顶注释写得很硬：依赖只有 `serde` /
`serde_json` / `semver`，**不碰 http、不碰 crypto、不碰 async runtime**
（`duck-ipc-proto/src/lib.rs:1`）。因为它在恢复路径上：`updaterd` 坏了也要能答话的
那个 crate，依赖越少越安全。

线格式一句话（`lib.rs:9`）：

> **Wire format: JSON-RPC 2.0, one object per line (NDJSON), over a unix socket.**
> Framing is a single newline.

所以从 socket 上读到的**不是二进制、不是 WebSocket，而是每行一个 JSON 对象**。通知
（notification，没有 `id`）与请求（request，带 `id`）两种消息靠 `id` 有无区分。协议版本
`API_VERSION` 现在到 17（`lib.rs:172`）——每次不兼容改动 bump 一次，而对不上版本的 peer
**不在握手时拒绝**，而是让"它够不到的 route / 变了形状的 params"自己按名字拒绝
（`lib.rs:50` 那段注释是整个设计决策的坦白）。

### 1.1 关节号是"裸数组"——印证第 2 讲

`lib.rs:247` 有一张 15 格表，它就是**线上按位置索引的真相源**：

```rust
pub const JOINT_NAMES: [&str; 15] = [
    "left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle", // 左腿 5
    "neck_pitch", "head_pitch", "head_yaw", "head_roll", "mouth",                  // 颈/头/嘴 5
    "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle", // 右腿 5
];
```

`RobotState`（`lib.rs:2568`）里 `joints` 和 `targets` 都是 `Vec<f64>`——**光秃秃的数组**，
字段注释写着 *indexed as [`JOINT_NAMES`]*。这就是第 2 讲强调的"线上按位置索引"：索引 3 是谁？
`JOINT_NAMES[3]` = `left_knee`。客户端**必须带着这张表才能显示**，所以这张表活在线协议里，
而 `duck-control` 用 `pub use duck_ipc_proto::JOINT_NAMES` 把它 re-export 回来，再用一个
编译期断言钉死长度（`duck-control/src/model.rs:24`）：

```rust
pub use duck_ipc_proto::JOINT_NAMES;
const _: () = assert!(JOINT_NAMES.len() == NUM_JOINTS);
```

> 一个 `const` 断言让"两条表对不上"成为编译错误。`model.rs` 里还有三个测试把同一件事在
> 运行期再钉一遍：`tables_agree_on_length`、`ids_are_unique`、`imu_id_does_not_collide_with_a_joint`。
> 你换飞特、把 IMU 请下总线之后，**这些测试正是你第一个要改、且改了仍必须绿的**——见第 5 节。

`RobotState` 其余字段也值得扫一眼，因为它们回答了"外面能看见这台机器人的什么"：

| 字段（`lib.rs`） | 内容 |
|---|---|
| `t` | 单调时钟秒，用于对样、不是报时 |
| `move: { requested, applied, limited_by }` | 客户端要的 vs 实际放行的；被限速时 `limited_by` 具名（如 `max_velocity`） |
| `head: [f64; 4]` | 头/颈目标 |
| `policy` | **这一拍**由哪个网驱动（`walk`/`stand`/`held`） |
| `safety: { fallen, limp, gravity, gain }` | 裁决 + 它依据的 gravity；`gain` 是"舵机实际被设成的 P 增益"（不是请求值） |
| `loop: { hz, missed }` | 实际达成频率、漏拍累计——`unhealthy: control loop at 43.9 Hz` 就是看它 |
| `joints` / `targets` | 量到 vs 指令的 15 关节角（裸数组，按 `JOINT_NAMES` 索引） |
| `odom` | 接触里程计：`{position, yaw}`，相对、会漂 |
| `theremin` / `chorale` | 可选块，乐器/合唱在不在才出现 |

`SafetyState.gravity` 有一句很关键的话（`lib.rs:2649`）：*Projected gravity in the trunk
frame, the input `fallen` is decided from. Upright is about `[0, 0, -1]`.* —— 这就是第 3/5 讲
那个 gravity 出了 crate 之后的形态；`robotctl monitor` 屏幕上那只鸭子的倾斜、`fallen` 判定、
`fall.rs` 的预测，全部从这一个数来。

### 1.2 两个词汇表：intents in / state out

`duck-ipc-proto` 把 method 分成 namespace（`lib.rs:267` 起）。控制面相关的核心是两组：

- **连续 intent**：`robot.move`、`robot.head`——以 notification 发送（无 `id`、无回包），
  50 Hz、last-writer-wins、带时间戳（deadman 靠"它多老"而不是"它是几"）。有回包的就是错。
- **离散 intent**：`robot.stop`、`robot.enable`、`robot.init`、`robot.relax`、`robot.do`——
  以 request 发送、必须被回答，因为"被拒了 + 为什么"是调用方要的答案。
- **状态流**：`robot.subscribe` 把这条连接变成 `robot.state` 通知流。服务端按订阅者各自
  decimate（`SubscribeParams { hz }`，`lib.rs:1766`），发布进一个有界广播，慢客户端拿到的
  是**空洞而不是背压**——控制线程永远不等客户端。

这就是 robotd-design §1.2 那张图的接线：`padd`、`robotctl monitor`、以后的 app，全说这同一
个词汇表；`mediad`/`btd` 只是**中继**而不是翻译。

### 1.3 健康是"发布"出来的，不是"问"出来的

`robot.health` 的答案里有个 `ImuHealth`（`lib.rs:2469`）：

```rust
pub struct ImuHealth {
    pub ready: bool,                  // 姿态滤波是否已收敛
    pub stale_blocks: u64,            // 累计读回相同块的次数
    pub consecutive_stale_blocks: u64, // 当前连续相同块数——这才是该告警的
}
```

`ImuHealth::FROZEN_RUN = 25`（`lib.rs:2496`）：连续 25 块相同 = 半秒 = "orientation frozen"，
而**不是**小 hiccup。注意 `duck-control/src/bus.rs:59` 的 `STALE_RUN_WARN: u64 = 25` 是**同一
个数字在硬件层的另一份拷贝**——两边注释互相点名，说这是故意的（硬件层不依赖 IPC 词汇表）。

> 这段对第 4 节的改造是伏笔：**"25 次 = 半秒"这个等式建立在 IMU 和舵机同一条总线、同一拍、
> 50 Hz 读一次的前提上。IMU 一旦离开总线，时基变了，"25"要不要跟着重算？**

---

## 2. robotctl：从终端走 unix socket 看它

### 2.0 它是个"瘦客户端"

`robotctl` 顶注释（`robotctl/src/main.rs:1`）自称 *A thin client*：解析 argv → 连 unix socket →
发一个 JSON-RPC → 打印流式通知和结果 → 把结果映射成退出码。**它里面没有任何"懂机器人"的
逻辑**——逻辑全在 daemon。它用 `std::os::unix::net::UnixStream`（`robotctl/src/main.rs:34`），
不是 tokio——短命 CLI，不需要。

socket 路径：`robotctl` 全局参数 `--robot-socket` 默认 `/run/robotd.sock`
（`robotctl/src/main.rs:82`），而 `robotd` 的 `--socket` 默认值也是 `/run/robotd.sock`
（`robotd/src/main.rs:208`）；真正把这些路径收口的是 `duck_ipc_proto::socket` 模块
（`lib.rs:196`，`ROBOT = "/run/robotd.sock"`），因为 `robotctl` 和 `btd` 都要连它，路径
每客户端抄一份就会各漂各的。

权限只有一条规则（cheatsheet 开头）：**只读命令不需要特权；任何"改变机器人"的命令要
`sudo`**。这也写进了协议——`Call::is_mutating()`（`lib.rs:805`）是 `updaterd` 授权用的清单。

### 2.1 monitor：把一个 NDJSON 流画成屏幕

`monitor` 订阅的就是上一节的 `robot.state`。`robotctl/src/monitor.rs` 顶注释说它是
**同一个流的两种渲染，由 stdout 去哪决定**：

- 终端：一帧原地重绘的 TUI——15 个"实测角度 vs 指令角度"的偏差条、IMU 投影重力、fall 裁决、
  loop 达成频率的 trace、底部健康行、右侧按实测关节姿势画的那只鸭子、最底下 odometry 的
  braille 轨迹地图。`q` 退出，`d` 关鸭子，`[`/`]` 旋转视角，`u` 切度/弧度，`t` 开 ToF 矩阵，
  `p` 开手柄原始事件流。
- 管道/文件/`--json`：每拍一行。落盘最常用这条（cheatsheet "Watching the loop"）：

```
robotctl monitor --json --hz 50 > run.jsonl
```

这一行给你的是**整份 `RobotState`、一行一个 JSON 对象**——这就是为什么说 monitor 的背脊是
NDJSON 流：机器人侧 `robot.subscribe → robot.state` 是 NDJSON，`monitor` 只是挑了个终端画法。

屏幕上有三样 cheatsheet 特意点名的东西，值得记住：

1. **帧上唯一 IMU 量是 projected gravity**（直立约 `[0,0,-1]`），`fallen` 由它判；而
   **stale 计数器和比值在 `robotctl health` 里**，不在 monitor 帧上。
2. 健康行的最后一行是"机器人的状态而不是行为"：电量伏特与百分比、最热舵机、板温，以及
   `unhealthy: control loop at 43.9 Hz` / `degraded: no robot on the motor bus after 3
   attempts` / **`orientation frozen — 25 stale reads`**。最后那句只在 health 行出现、别处没有
   ——"板子已停止融合但还在答总线，所以什么都不报错、gravity 永远是一个貌似合理的姿态"
   正是第 1.3 节 `consecutive_stale_blocks` 想抓的事。
3. 底边框点名**当前加载的 policy 文件名**——`walk` 是"两个 release 不同步态都会报的名字"，
   想知道"到底是哪个网络"得看这里，而不是看帧上的 `policy: walk`。

### 2.2 一张表：我想从外面看 X

全部出自 `~/microduck/docs/robot/cheatsheet.md`，命令原样照抄。除注明外都【需真机】——
`robotctl` 走 unix socket 找 daemon，没有 `robotd` 就没有 socket（这正是 00 讲"无鸭可跑矩阵"
里 `robotctl` 那格写"是"的原因）。

| 我想从外面看 … | 敲这条 | 出处（cheatsheet 小节） |
|---|---|---|
| 先确认每个 daemon 跑的 vs 装的是不是同一个 release | `robotctl version` | The first thing to run |
| 硬件+软件一页报告；脚本可拿退出码判健康 | `robotctl health` · `robotctl health --json` | The first thing to run |
| 这一拍实况：请求 vs 实际、限速原因、15 关节跟踪、gravity、loop hz | `robotctl monitor` | Watching the loop |
| 同一流落盘，每行一个完整 state 对象 | `robotctl monitor --json --hz 50 > run.jsonl` | Watching the loop |
| 关节上电并 ~2 s 爬到 home（会动所有关节，上支架！） | `sudo robotctl robot init` | Power to the joints (`robotd`) |
| 切断电源松弛（没人扶会塌，所以要 `--yes`） | `sudo robotctl robot relax --yes` | Power to the joints (`robotd`) |
| 手柄连没连 / `padd` 活没活（两件事分开答） | `robotctl pad status` | Gamepad (`configd`) |
| 配新手柄 | `sudo robotctl pad pair` | Gamepad (`configd`) |
| 当前 wifi：SSID/信号/地址 | `robotctl net status` | Wifi (`configd`) |
| 名字/序列号/运行时长 | `robotctl system info` | Identity and power (`configd`) |
| 有没有可更新的 release | `robotctl update status` · `robotctl update check daemon` | Updates (`updaterd`) |
| 最近一次更新干了什么 | `robotctl update log` · `robotctl update show 42` | Updates (`updaterd`) |
| 逐个键改 `/etc/robot/robotd.toml` | `sudo robotctl configure` | Configuring the robot |
| robotd 自己打的日志 | `journalctl -u robotd -f` | Logs |
| **谁占着总线**（排查第二串口时你会需要它） | `fuser -v /dev/ttyS2` | robotd-design §1.1（不是 cheatsheet） |

> 别急着背。这张表对第 8 讲真正的作用是：**它是你改完之后的验证手段**。你换完飞特、
> 把 IMU 请下总线之后，`robotctl version`、`robotctl health`、`robotctl monitor --json`、
> `sudo robotctl robot init` 应该照常工作——如果有一条命令读出来的语义变了而你事先不知道，
> 那是改动漏了线协议那一层。

---

## 3. 改造切入点 A：14 个 XL330 → 飞特 HL-2909

你在 `~/microduck_rl` 的评估结论（`feetech_hls2915_servo_swap.md` §0/§3.5）已经写清：
**动作空间不变**——15 关节 / 14 动作 / 61 维 obs / ONNX 契约全不动；换的是**协议层、寄存器
语义、增益刻度、物理参数**。这里只把它们落到 `~/microduck` 的文件。

### 3.1 model.rs：常数表整段换语义

`duck-control/src/model.rs` 是"机器人=数据"的第一站，飞特换装在这里动四块：

1. **`JOINT_IDS`（`model.rs:15`）**——现在是一串 Dynamixel ID `20-24 / 30-34 / 10-14`。飞特
   HLS 出厂 ID=1（`feetech_hls_memtable.md` 地址 5），你要给 15 个舵机逐一烧录唯一 ID，然后把
   这张表**内容**换成新 ID；长度仍 15、仍按 `JOINT_NAMES` 索引。第 2 讲"线上按位置索引"不因
   换舵机而变——`JOINT_NAMES` 来自协议 crate，`joint_index("mouth")` 照旧。
2. **`EXPECTED_REGISTERS`（`model.rs:89`）**——这张表是"启动时断言并纠正的 EEPROM 寄存器"，
   现在全是 Dynamixel 语义：`return_delay_time`/`baud_rate`/`pwm_slope`/`shutdown`。飞特没有
   这些名字。第 2 讲点名的 **return_delay_time 时序注释就在这张表头上**（`model.rs:84`）：
   XL330 出厂 250 → 每设备 500 µs 往返 → 16 设备 8 ms/拍、占 20 ms 预算的 40%。飞特协议里
   对应的"每帧应答开销"概念不同（帧头 0xFF 0xFF + 校验和、应答状态级别地址 8），**启动检查
   的哲学保留，但表内容换成飞特 EPROM**：ID(5)、波特率(6)、EPROM P/D/I(21/22/23)、运行
   模式(33)=0 位置伺服、保护电流(28)、卸载/报警条件(19/20)。**这是一张新表，不是改数值。**
3. **`BAUD_RATE`（`model.rs:80`）**——`1_000_000` 是波特率数值；飞特那边波特率是**寄存器索引**
   （地址 6：`0=1M … 7=38400`），默认 1 Mbps 恰好也是 `0`。`DynamixelIo::open` 现在把
   `BAUD_RATE` 直接喂给 `serialport::new`，换协议后这个常数要么仍作串口波特率、要么新开一个
   索引常量，别让两个含义混在一个名字里。
4. **电池常数（`model.rs:109`）**——`BATTERY_FULL_V = 8.2` / `BATTERY_EMPTY_V = 6.6` 是按
   NP-F550 2S Li-ion 标定的，**而飞特是 12 V 系统（9–14 V）**。`battery_percent` 与
   `[safety] battery_empty_shutdown` 的"坐下去断电"联动都吃这两个数——不改的话 12 V 读数
   `12.4 V` 会被 `battery_percent` 直接 clamp 成 100%，而欠压保护永远不触发。你的换装文档
   §3.2 里 `vin_range=(10.5,12.6)` 是 **sim 侧**的电压域，`model.rs` 这两个常数是**真机侧**
   的同一决策，两侧要一致。

### 3.2 bus.rs：协议整层替换

`duck-control/src/bus.rs` 的 `DynamixelIo` 是"唯一碰总线的实现"，飞特换装在这里改动最大：

- **依赖与类型**：`use rustypot::servo::dynamixel::xl330::Xl330Controller`
  （`bus.rs:18`）和 `Xl330Controller::new().with_protocol_v2().with_serial_port(serial)`
  （`bus.rs:110`）——Dynamixel Protocol 2.0 的控制器，**整个换**成说 FT-SCS 协议的控制器
  （自己写一个薄后端，或 vendor 一个支持 HLS 的开源 SDK；你的 `ftscs_protocol.md` 已存档）。
- **每一拍读**：`sync_read_raw_data(&ids, READ_ADDR=124, READ_LEN=12)`（`bus.rs:236`）。飞特
  反馈区在**地址 56–70**（当前位置 56/57、当前速度 58/59、电压 62、温度 63、电流 69/70，
  `feetech_hls_memtable.md` §2.4），不是 124。`READ_ADDR`/`READ_LEN` 和 `read()` 里逐字节
  解码那段（`bus.rs:274`）都要按飞特反馈表重写。
- **好消息：一次事务的哲学能搬。** FT-SCS 有 SYNC READ(0x82)/SYNC WRITE(0x83)
  （`ftscs_protocol.md` §4.6/§4.7），且写明"所有查询长度与首地址必须相同、应答按指令包中 ID
  顺序返回"——这正是 `sync_read` 的语义。但协议注明"此指令针对部分串行总线舵机开放"，你的
  G 阶段（`actuator_physics_swap_plan.md` §4 G4）要先拿单舵机验证 **HLS 磁编码款支不支持同步
  读**；不支持就得退化成逐 id 轮询，20 ms 预算要重算。
- **单位换算**：`RAD_PER_SEC_PER_COUNT = 0.229 rpm/count`（`bus.rs:31`）是 Dynamixel 速度反馈
  单位；飞特反馈速度单位是 **0.732 RPM/LSB**（memtable 58/59）。位置 `2π·pos/4096 − π`
  （`bus.rs:287`）里 4096/圈分辨率两边相同，但飞特反馈位置是 **BIT15 方向的多圈 ±32767**、
  单圈 0.087°/LSB，且有一个"位置偏移"寄存器(31/32)定义中位——**换算回 `[-π, π]` 的那一行要按
  飞特语义重写，并和 `DEFAULT_POSITION` 的零点对齐**。`slow_sensors` 的电压/温度同理：Dynamixel
  `u16 0.1V @144 + u8 °C @146` → 飞特 `u8 0.1V @62 + u8 °C @63`。
- **写目标**：`sync_write_goal_position`（`bus.rs:293`）→ 飞特 SYNC WRITE 目标位置地址 42/43
  （0.087°/LSB、多圈 BIT15 方向）。

### 3.3 增益：不是同一个单位

`set_gain(kp: u16)`（`bus.rs:304`）现在写 Dynamixel 的 `position_p_gain`、并把 I 和 D 写 0——
那段注释（`bus.rs:305`）说得很清楚：Dynamixel 的 RAM 增益每次上电回出厂、出厂 D 不是 0，所以
**写 0 是刻意的**。飞特这边：

- 位置环增益是 **Kp@50（1/8 缩放）、Kd@51（1/4 缩放）、Ki@52（位置模式无效）**，上电由 EPROM
  21/22/23 赋值（memtable §2.3）。"每次上电要重写 SRAM 增益"这个**时序逻辑保留**，但"写哪些
  寄存器、值域 0–254、带缩放"全是新的。
- `safety` 里"limp = 把增益降到 50"、正常跑 200（`io.rs:122` 附近）——这两个数字是 **Dynamixel
  刻度**；换到飞特 0–254 + 1/8 缩放的刻度，`gain_limp`/默认增益的**数值要重定**，而且换装文档
  要求 **Kd 显式写、且与 sim 建模一致**（`feetech_hls2915_servo_swap.md` §1.1：BAM 是 P-only+
  反电动势，真机却带 D）。这不是搬数值，是把"软/硬"的刻度整个翻译一遍。
- **第 2/4 讲讲过的低通与"下发即到"假设要重新审视**：飞特固件有目标速度+加速度"缓起缓停"
  （ACC@41，0=最大加速度；SPEED@46/47），50 Hz 下发的目标在真机内部按斜坡追——策略训练假设
  "20 ms 内目标基本到位"，慢舵机 + 固件限幅会引入**额外的相位滞后**。sim 侧建模是
  `~/microduck_rl` 的活（你的 §3.1 HLS2909Actuator），运行时侧要意识到：同一份 obs 布局，喂给
  策略的 `joint_pos`/`joint_vel` 的时间语义变了。

### 3.4 不改的部分，也列出来（免得下错刀）

`NUM_JOINTS=15` / `MOUTH_INDEX=9` / `ACTION_LEN=14` / 61 维 obs 布局 / `scatter_action` /
`policy.rs` 加载期校验 / `safety.rs` 的裁决逻辑——**全部不动**。`robotd/src/control.rs` 的
"home + action_scale × action + 低通"链（`control.rs:432`）也不动。动作空间不变意味着：**协议层
(`duck-ipc-proto`) 完全不用动**——线上 `joints`/`targets` 还是 15 个 f64，`JOINT_NAMES` 一个
字节不改。

---

## 4. 改造切入点 B：IMU 从伺服总线上拿下来直连

### 4.0 现在的结构

`imu_to_dxl` v2 板（LSM6DSV16X）以 **id 200** 骑在 Dynamixel 总线上，和舵机共享同一根 UART，
在**同一个 `sync_read`** 里读回（robotd-design §1.1 那张图）。`io.rs:6` 的模块注释把这件事说成
了 trait 的形状：

> `[Sensors]` carries joints *and* IMU together because that is what the hardware does: the IMU
> board sits on the Dynamixel bus and is fetched in the same transaction as the servos. A trait
> that split them would invent a distinction the bus does not have, and would double the bus
> traffic to honour it.

第 3 讲讲过"为什么拆它现在才正当"——当时**不正当**，因为总线没给理由。你把它请下总线，
这句话的前提就没了：**"硬件就是这么做的"不再是事实，拆 IMU 源才是诚实的建模**。

### 4.1 下刀清单：id 200 相关的每一处

| 位置 | 现在 | 直连之后 |
|---|---|---|
| `model.rs:78` `IMU_DXL_ID = 200` + 测试 `imu_id_does_not_collide_with_a_joint` | IMU 在总线上要有 ID | IMU 不在总线上，常量可删；测试改成"总线 id 列表无冲突"即可 |
| `bus.rs:114` `ids = [IMU_DXL_ID, JOINT_IDS…]` | 16 设备一包读，IMU 排第一 | ids 只剩 15 个舵机 |
| `bus.rs:250` `read()` 里 slot 0 分支 | 从 124 号块拷 12 字节喂 `SflpDecoder` | 这个分支整段搬走 |
| `bus.rs:67` `StaleImuTracker` + `bus.rs:258` 告警 | 挂在 `DynamixelIo` 上、按 tick 数 | 要迁到新 IMU 源（见 4.3 ready/stale） |
| `bus.rs:389` `imu_ready()` / `bus.rs:385` `imu_stale()` | 从 `self.imu`/`self.stale_imu` 取 | 改从新源取；`RobotIo` trait 的默认实现保留 |
| `imu.rs` `SflpDecoder` 全家 | decode 的是"总线块里那 12 字节" | 见 4.2：芯片不变则**整体复用** |
| `robotd` 打开端口 | `DynamixelIo::open(&params.bus.port)`（`main.rs:743` `run_init`、`main.rs:883` `open_bus`）——只有一个串口 | 新增第二端口打开点，见下 |
| `robotd-params` | `bus.port` 默认 `/dev/ttyS2`（`robotd-params/src/lib.rs:848`） | 加一个如 `imu.port` 的落点；`robotctl configure` 会因为它出现而自动认识它（schema 同源） |
| `robotd/main.rs:1867` tick 里 `let imu_warm = safety.imu_ready();` | 策略启动前等 IMU 收敛 | 语义保留，但要等的是**新源**的 ready |

"第二端口开在哪"要拆两层：**打开与权限**、**时序**。

- 打开：现在 `robotd` 以 root 跑（design §1.1），已有字符设备访问权；若 systemd unit 对
  `/dev` 有 `DeviceAllow`/`PrivateDevices` 类限制，新串口节点要加进 unit。而且 Armbian 默认在
  UART 上跑登录 console——`setup-board.sh` mask 掉 `serial-getty@ttyS2` 的**做法对第二个串口
  同样适用**，否则 `agetty` 会抢你的新 IMU。
- 时序：`Sensors` 数据类**保留**（它只是个快照），但"一次事务同时拿到 joints+imu"的**原子性
  没了**。两个端口各有自己的读时刻，拼成同一份 `Sensors` 时 joints 和 imu 天然差几毫秒甚至一
  拍。这就是第 3/6 讲反复说的"时间对齐"——以前硬件帮你对齐，以后要你自己对齐（见 4.3）。

### 4.2 两条路线，一条明确推荐

**路线①：保留 LSM6DSV16X + SFLP 芯片，只换传输。** 芯片仍在片内跑 SFLP，输出 fused
quaternion + gyro；你只是把"这 12 字节从哪来"从"总线块 124"改成"第二个串口/另一路"。`imu.rs`
的 `SflpDecoder`（`half()` 解码、`rotate`/`median3_each`、`DEFAULT_MOUNT`、`ready()` 门限）
**整体搬运、几乎不改**。obs 第 0..6 格（gyro 3 + projected gravity 3）的单位和坐标系不变，
`safety`/`fall` 直接续命。真正工作集中在传输层 + 时序。

**路线②：换普通串口 IMU（裸 accel/gyro）。** 现在片上的 SFLP 融合要搬到主机端——要么自己写
互补滤波/Madgwick，要么找带自带四元数的模块；gyro bias、温度、安装角全部重新标定；
`ready()` 的语义（"融合已收敛"）要重新定义。obs 的前 6 格、fall 的解析导数前提
（`ġ = −ω×g`，gyro 与 gravity 同源同拍）都可能被滤波滞后破坏。**这是风险最大的一条。**

> **明确建议：走路线①。** 你的 RL 侧两个文档都只重舵机、没碰 IMU/obs 语义——路线① 完美贴合
> "架构不动"的前提：decode、mount、obs 布局、安全裁决全不动，只把 id-200 那一支从伺服总线挪到
> 一个新源。路线② 是把"别人做好的融合"变成"你要在 50 Hz 主循环里养的另一个滤波器"，那是
> 一整类新风险，和换舵机叠在一起会让故障无法归因。

### 4.3 真正的坑：ready / stale 语义迁移，而不是解码

三个数字现在互相咬合，全建立在"50 Hz、同一总线、同一拍"上：

- `SflpDecoder::ready()` = `quat_samples >= 25`（`imu.rs:99`），注释说约 0.25 s；
- `bus.rs` `STALE_RUN_WARN = 25`（=半秒）；
- `duck_ipc_proto` `ImuHealth::FROZEN_RUN = 25`（=半秒，`lib.rs:2496`）。

IMU 离开总线后：新源的采样率可能不是 50 Hz（SFLP 板现在约 100 Hz 内部刷新、你 50 Hz 采样），
读时刻和 tick 不再锁相，"连续 25 块相同"的含义、`ready()` 要等多久、stale 计数在哪个线程更新
——**这三个数/三处代码要一起搬、一起重定**，否则 monitor 的 `orientation frozen — 25 stale
reads` 会在新硬件上要么永远不响、要么开机就误报。这正是你 `~/microduck_rl/docs/` 换装文档的
**空白**：`feetech_hls2915_servo_swap.md` 和 `actuator_physics_swap_plan.md` 都只在运行时一行
带过"协议层全换"，**没有任何一节写 IMU 直连**。建议补一节，标题就叫
**「直连 IMU + 复用 SflpDecoder + obs/安全语义不变」**，内容至少覆盖：路线① vs ② 的取舍、
`SflpDecoder`/`StaleImuTracker`/`imu_ready` 三件套迁到新源、25 这个数在新时基下的重算、第二
端口的 systemd/权限。

---

## 5. 下刀顺序与规模评估

一次把话说清：**官方代码改动面在 4 个文件 + 1 个新协议后端**，不是全仓。

| 文件 | 现状 | 飞特 + IMU 直连后 | 搬运 or 重写 |
|---|---|---|---|
| `duck-control/src/model.rs` | 常数表（Dynamixel 语义） | `JOINT_IDS` 内容、`EXPECTED_REGISTERS` 整表、`BAUD_RATE` 含义、电池常数(8.2/6.6→12 V 域) | 重写（表换语义），但 `JOINT_NAMES`/`NUM_JOINTS`/`MOUTH_INDEX` 是搬运 |
| `duck-control/src/bus.rs` | `Xl330Controller`+Protocol2.0+124/136 块 | FT-SCS 后端、反馈表 56–70、单位换算、增益刻度、`sync_write` 目标地址 | **重写**（协议整层）；但"一次读 N 个"哲学、stale 计数结构是搬运 |
| `duck-control/src/imu.rs` | `SflpDecoder`（总线块解码） | 路线①：原样复用，换数据来源 | **搬运** |
| `duck-control/src/io.rs` | `RobotIo::read` 一事务拿 joints+imu | 拆源后 trait 语义要改（读只回关节，IMU 走新源），`Sensors` 保留 | 半搬半写 |
| 新（或 vendor） | — | FT-SCS 协议后端（读写/寄存器 helper） | 新写，参考 `ftscs_protocol.md` + memtable |
| `robotd/src/main.rs` + `robotd-params` | 单端口 `bus.port=/dev/ttyS2` | 第二 IMU 端口的打开、权限、`imu.port` 参数 | 小改（新增一个打开点） |
| `obs.rs` / `policy.rs` / `safety.rs` / `fall.rs` / `duck-ipc-proto` | — | **完全不动** | 搬运（零改动） |

**哪些是搬运**：`JOINT_NAMES` 表、61 维 obs 布局、`scatter_action` 嘴跳过、`safety` 的裁决
逻辑、`fall` 的解析导数、`SflpDecoder` 解码、整个线协议。**哪些是重写**：`bus.rs` 的协议读写、
启动寄存器检查、增益刻度、`model.rs` 的常数语义、电池电压域、IMU 的传输层。

**真正的风险不在 diff，在"时间对齐与 ready/stale 语义迁移"**：joints 与 IMU 分属两端口后的
原子快照丢失、`25` 这个数在新时基下的重定义、飞特固件缓起缓停给"下发即到"假设引入的滞后。
这三件事都不产生大 diff，却决定了真机是"走起来像训练里的鸭子"还是"看起来对、跑起来摔"。

**验收思路（无鸭即可执行）**：第 1 讲那批 `duck-control` 单测就是你的**改造前基线**：

```bash
cd ~/microduck && cargo test -p duck-control
```

下刀之后，"语义没被破坏"的机器读法是：**同一批测试（更新到新常数表后）依然全绿**。要区分两类
测试——凡是写死"XL330 刻度 / id 200 在总线上 / Dynamixel 寄存器名 / 124 号块布局"的，必须
**改写**成新契约（这不是作弊，是用测试把新契约钉下来）；凡是保护**不变量**的——obs 偏移、
嘴跳过、`scatter` 回填、安全裁决、stale 计数结构——必须**原样绿**。一批测试里哪些"应该变"、
哪些"必须不能变"，本身就是改造设计的一部分。

---

## 6. 自己动手

### 【无鸭可跑】先把基线跑绿、读三处注释

```bash
cd ~/microduck
cargo test -p duck-control                        # 全绿 = 你的改造前基线

# 挑几个"换舵机后必须仍然绿"的不变量：
cargo test -p duck-control the_layout_widths_sum_to_the_declared_input
cargo test -p duck-control every_block_lands_at_its_documented_offset
cargo test -p duck-control scattering_an_action_skips_the_mouth
cargo test -p duck-control gravity_is_a_unit_vector_in_steady_state

# 挑几个"写死了 XL330 / id200 / 124 块 / 0.229"，下刀时你预期它们要改写：
cargo test -p duck-control velocity_scale_matches_the_datasheet_figure
cargo test -p duck-control position_conversion_round_trips_through_rustypot
cargo test -p duck-control imu_id_does_not_collide_with_a_joint
cargo test -p duck-control a_dead_board_runs_past_the_warning_threshold
```

再读三处注释，它们是改造要保住的"为什么"：`model.rs:84`（return_delay_time 时序）、
`io.rs:6`（Sensors 为什么连体——你拆它的理由）、`imu.rs:95`（`ready()` 为什么等 25 个样本）。
读 `bus.rs` 的 `StaleImuTracker` 测试夹具（`bus.rs:394` 起），想象它从"总线块"换成"新 IMU 流"
后哪些测试要改、哪些不用。

### 【需真机】robotctl 命令是改完之后的验证手段

以下都标【需真机】，且**建议在动手改之前先跑一遍**，把"现在的正常长相"存进记忆：

```bash
robotctl version                                  # 每只 daemon 跑 vs 装 是否一致
robotctl health                                   # 硬件+软件一页；退出码可判
robotctl monitor                                  # 终端 TUI；q 退出，u 切弧度
robotctl monitor --json --hz 50 > run.jsonl       # NDJSON 落盘，先存一份"改前真机流"
journalctl -u robotd -f                           # daemon 日志（改 bus/imu 时盯着它）
```

跑不了真机时，这一节唯一能离线做的是对照 2.2 那张表，把每条命令的"出处小节"翻到 cheatsheet
原文读一遍——它们是你在真机上确认"换装后语义没变"的核对清单。

---

## 7. 收尾检查

- [ ] `RobotState.joints`/`targets` 为什么是裸 `Vec<f64>`？它靠谁才有名字？（答：`JOINT_NAMES` 在线协议里；`model.rs` re-export + 编译期断言钉长度）
- [ ] `robotctl monitor` 的背脊是什么？终端渲染和 `--json` 落盘是什么关系？（答：都是 `robot.subscribe → robot.state` 的 NDJSON 流；终端画 TUI，`--json` 一行一个对象）
- [ ] 换飞特后，哪些"常数/刻度"换了语义、哪些"数"原样保留？（答：ID 表、寄存器表、波特率含义、电池电压域、0.229→0.732 RPM/LSB、增益 0–254 缩放换掉；`NUM_JOINTS`/`MOUTH_INDEX`/61 维 obs/`scatter` 不动）
- [ ] IMU 直连的两条路线分岔在哪？为什么推荐保留 SFLP 芯片？（答：解码与 ready/mount/obs/safety 语义可复用；换裸 IMU = 主机端重做融合 + 全重新标定）
- [ ] "25"这个数字出现在哪三处？IMU 离开总线后它为什么是第一个要重算的？（答：`imu.rs` ready 门限、`bus.rs` STALE_RUN_WARN、proto `FROZEN_RUN`；三者都假设 50 Hz 同拍）

**至此八讲闭环**。回到 00 讲那句"20 ms 一拍"——你现在能指出每一环在哪个文件：**敲拍的时钟**
在 `robotd/src/main.rs` 的 tokio interval（第 6 讲）；**读一次总线**是 `duck-control/src/bus.rs`
的 `sync_read`；**快照长什么样**是 `io.rs` 的 `Sensors`；**IMU 那一格**由 `imu.rs` 的
`SflpDecoder` 喂进来；**61 维 obs** 在 `obs.rs` 拼装；**选网与推理**在 `robotd/src/control.rs`
+ `policy.rs`；**14→15 散开再缩放**在 `obs.rs::scatter_action` + `control.rs` 的
`home + scale + 低通`；**安全裁决**是 `safety.rs`（唯一握着 `RobotIo` 的门卫）；**写回总线**是
`bus.rs` 的 `sync_write_goal_position`；**门外的人怎么看它**，是 `duck-ipc-proto` 的 NDJSON +
`robotctl`。改造下刀前，让 `cargo test -p duck-control` 全绿；下刀后，让同一批测试（更新到新
常数表后）再绿——那就是"语义没被破坏"的机器读法。前 7 讲让你读懂这只鸭子，第 8 讲给你的是
改它的手术图谱。

# 02 · 常数表与一次事务：model.rs 与 io.rs

> 目的：把"鸭子只是一张数据表"这件事吃透——15 个关节怎么编号、那排 EEPROM 常数为什么
> 动不得、`RobotIo` 为什么设计成"一次事务"。改造飞特/IMU 时，这两份文件是主战场。

## 1. model.rs：把机器人当成"数据"的常数表

`duck-control/src/model.rs:1` 顶部注释先立规矩：

> *The numeric values here are lifted from `microduck_runtime`'s `motor.rs`, where they were
> **measured against hardware** rather than derived. Re-deriving them from a datasheet is exactly
> the kind of change that looks right and walks wrong.*

记住这句话：**这文件里的数不是查手册算出来的，是对着真鸭子量出来的**。改它们（尤其做飞特
换装时想"按数据手册重推一套"）要三思——这是全仓最像"到处引用、一动全歪"的地方。

### 1.1 关节：15 个、分三组、顺序即协议

```rust
pub const NUM_JOINTS: usize = 15;
pub const JOINT_IDS: [u8; NUM_JOINTS] = [
    20, 21, 22, 23, 24, // left leg     ← 左腿 5
    30, 31, 32, 33, 34, // neck/head/mouth ← 颈/头/嘴 5（mouth 在 30–34 组内）
    10, 11, 12, 13, 14, // right leg    ← 右腿 5
];
```

`JOINT_NAMES` 不是在这里定义的——它是从 IPC 协议 crate 再导出的：

```rust
pub use duck_ipc_proto::JOINT_NAMES;     // 线上的 joints/targets 按位置索引
const _: () = assert!(JOINT_NAMES.len() == NUM_JOINTS);  // 编译期防漂移
```

为什么要"借用"而不是自己再抄一份？因为 `Sensors`/`JointTargets` 在线上是**裸数组**（按位置
对应关节），名字只给人和工具看。两处各存一份就会漂移，所以让编译期断言（`const _: () = assert!`）
把"长度必须一致"钉死。这是全仓贯穿的风格：**用类型/断言把约定变成编译错误**。

三组顺序不是随便排的——`sync_read` 按 ID 顺序返回块，而 `bus.rs` 拼 `Sensors` 时按 `JOINT_IDS`
顺序填位置，所以**这个数组的下标就是关节的全局序号**。索引 9 是谁？

```rust
pub const MOUTH_INDEX: usize = 9;   // = JOINT_NAMES[9] == "mouth"（有测试钉着）
```

鸭嘴不在任何 alpha 策略里（动作只有 14 维），所以要有个"显式的洞"让跳过 mouth 是故意的、
而不是某个 off-by-one。单测 `mouth_index_names_the_mouth` 保证它指向的真是 mouth。

### 1.2 初始姿势：和训练共用同一张表

```rust
pub const DEFAULT_POSITION: [f64; NUM_JOINTS] = [ ... 0.0, -0.0873, -0.4579, ... ];
```

每个值旁的注释标了是哪条腿/哪个节。要点在 doc 注释里：

> *Must match `HOME_FRAME` in the training env — a policy observes joint positions **relative
> to home**, so a discrepancy here is a constant offset on 14 observation slots.*

即：策略训练时看到的是"相对 HOME_FRAME 的角度差"，真机上 `obs.rs` 也先减 `DEFAULT_POSITION`。
两边不一致 = 观测整体偏一个常数 = 鸭子看起来"站对了却莫名往一边倒"。
（`microduck_rl` 侧对应文件：`src/mjlab_microduck/robot/microduck_constants.py` 的 `HOME_FRAME`。
你的换装分支若改关节配置，这里和训练侧必须同一次改。）

### 1.3 IMU id 与那排"动不得"的寄存器

```rust
pub const IMU_DXL_ID: u8 = 200;
pub const BAUD_RATE: u32 = 1_000_000;

pub const EXPECTED_REGISTERS: &[(&str, u8)] = &[
    ("return_delay_time", 0),
    ("baud_rate", 3),        // 3 = 1 Mbps，必须和 BAUD_RATE 一致
    ("pwm_slope", 255),
    ("shutdown", 52),        // 过载/过热/欠压的错误锁存掩码
];
```

`return_delay_time` 是**最关键**的一行，doc 注释给了完整的算术：

> *The XL330 ships at 250, which is 500 µs of turnaround **per device**. Across 16 devices that
> is 8 ms per tick — **40% of a 20 ms budget** — spent waiting for servos to get around to
> answering.*

看：16 个设备 = **15 舵机 + IMU（id 200）**，每个省 500 µs，一拍省 8 ms。**IMU 不只是被"顺带"
读的——它是这 16 设备时序预算里的一员**。你把 IMU 挪下总线，这行数学从 16 变 15，`check_registers`
（`bus.rs:132`）本来也只遍历 `JOINT_IDS`，所以那部分不受影响——但你要意识到这套时序是围着
"16 设备同包"调的。

`shutdown = 52`：舵机内部的错误锁存掩码（过载/过热/欠压 各占一个 bit）。这是"Dynamixel 特色"——
飞特舵机的错误寄存器体系不同，换装时 `EXPECTED_REGISTERS` 整张表要换、而不是改数字。

### 1.4 电量：没有电量计，只能看总线电压

```rust
pub const BATTERY_FULL_V: f64 = 8.2;    // NP-F550 2S，满电带载
pub const BATTERY_EMPTY_V: f64 = 6.6;   // 低到开始吃力，还没到电芯保护
```

doc 注释讲清楚了：机器人**没有 fuel gauge 也没有 ADC**，唯一的电量读数来自舵机回报的供电电压
（"穿过总线看到的电池"），所以 8.2/6.6 是**带载可用区间**，不是电芯化学区间。`battery_percent`
必须 clamp、对 NaN/负数返回 0——单测 `battery_percent_*` 一排都是钉这个的。

> 飞特视角：恒力舵机对电压的回报、以及它的欠压/过压点，未必与 XL330 的 `shutdown=52`
> 语义一致——这套"电压=电量"映射到新舵机上要重新对着硬件标定（对应 `microduck_rl` 的
> BAM `vin=(10.5,12.6)` 那套电压参数，见你的换装文档）。

## 2. io.rs：控制循环与物理世界的唯一边界

（第 1 讲已给 `RobotIo` trait 全貌，这里只补"为什么一次事务"的论证，它是你拆 IMU 的钥匙。）

`io.rs:6` 模块注释：

> *`Sensors` carries joints AND IMU together because that is what the hardware does: the IMU
> board sits on the Dynamixel bus and is fetched in the same transaction as the servos. A trait
> that split them would invent a distinction the bus does not have, and would double the bus
> traffic to honour it.*

翻译成大白话：**不是设计者喜欢把 IMU 塞进 `Sensors`，而是硬件把 IMU 放上了伺服总线**。
trait 是照着硬件长的。所以当你决定"IMU 不上总线、直连主机"，你是在**让硬件长出总线没有的
区别**——这时拆 trait 才是正当的。第 8 讲会把它落成具体改动清单，这里先种下因果。

## 3. 自己动手【无鸭可跑】

```bash
cd ~/microduck
cargo test -p duck-control tables_agree_on_length
cargo test -p duck-control ids_are_unique
cargo test -p duck-control imu_id_does_not_collide_with_a_joint
cargo test -p duck-control mouth_index_names_the_mouth
cargo test -p duck-control battery_percent_spans_the_usable_range
cargo test -p duck-control home_pose_legs_are_mirrored
# io 假件行为：
cargo test -p duck-control fake_io_tracks_what_was_written
cargo test -p duck-control frozen_fake_io_ignores_writes
cargo test -p duck-control simulated_read_failure_clears_itself
```

挑一个测试，故意改坏常数看它怎么红（例如把 `IMU_DXL_ID` 改成 20，跑
`imu_id_does_not_collide_with_a_joint`），体会"用单测把协议约定钉死"的做法。

## 4. 收尾检查

- [ ] `JOINT_NAMES` 为什么从 `duck_ipc_proto` 再导出而不是自己写？（答：线上数组按位置索引，防漂移 + 编译期断言）
- [ ] `return_delay_time=0` 值多少钱？（答：16 设备 × 500 µs = 8 ms，20 ms 预算的 40%）
- [ ] 为什么"IMU 与舵机同包读"是 trait 设计使然，而不是偶然？（答：硬件如此，trait 照硬件长）

**下一步**：第 3 讲进 `bus.rs` 看那"一次事务"到底怎么读 16 个设备、字节怎么变成弧度；
再进 `imu.rs` 看 SFLP 姿态块怎么变成 `gravity`。

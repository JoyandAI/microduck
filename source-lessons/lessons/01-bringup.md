# 01 · 先把仓库跑起来：workspace、duck-control 与"无鸭能测"

> 目的：先建立**能编译、能测、能假跑**的基座，并看清 19 个 crate 谁是谁。
> 这一讲不深入任何逻辑，只解决"地图 + 跑通"两件事。

## 1. 先看 workspace 布局

打开 `~/microduck/Cargo.toml` 顶部：

```toml
[workspace]
resolver = "3"
members = ["btd", "configd", "duck-control", "duck-detect", "duck-ipc-proto",
  "duckctl", "kinematics", "mediad", "odometry", "padd", "pet-detect", "sounds",
  "tof", "updater", "robotctl", "robotd", "robotd-params", "test-support", "xtask"]
```

19 个 crate。别被吓到——用"开机后谁在跑"来分组立刻清晰：

| 组 | crate | 一句话 |
|---|---|---|
| **控制心脏（你要改的）** | `duck-control` | 模型/总线/IMU/观测/策略/安全，**纯逻辑，非 daemon** |
| **守门进程（你要改的）** | `robotd`、`robotd-params` | 50Hz 主循环、IPC、systemd、参数注册表 |
| 你从外面敲的命令 | `robotctl`（机器人上）、`duckctl`（PC 上经蓝牙） | 读状态/下发意图 |
| 系统级服务 | `configd`（网络/电源）、`updater`（OTA）、`btd`（蓝牙）、`sounds`（发声）、`padd`（摇杆）、`mediad`（相机）、`tof`、`duck-detect`/`pet-detect` | 各管一段外围 |
| 工具 | `xtask`、`test-support` | 构建运维、测试假件 |
| 库 | `duck-ipc-proto`（IPC 协议）、`kinematics`（正解/逆解）、`odometry` | 被上面引用 |

先只盯住加粗的三个 crate。其余外围（相机/ToF/蓝牙/发声）**在你能让鸭子走之前都不必读**。

## 2. duck-control：为什么它是"无鸭可测"的心脏

`duck-control/src/lib.rs` 第一段注释就是设计声明（值得背下来）：

> *Deliberately not a daemon. There is no tokio here, no socket, no systemd — `robotd`
> owns all of that. The boundary is enforced by the compiler rather than by discipline.*

即：**`duck-control` 刻意不碰任何进程/IO 外围**，`robotd` 才管定时器和 socket。好处正是你要的——
这颗"心脏"可以在没有鸭子、没有 Linux、没有 socket 的普通电脑上单独编译、单独跑全部单测。

它只有 8 个源文件，每个都短，按数据流排：

```
model.rs  常数表（关节编号/寄存器/默认姿势）
io.rs     RobotIo trait + Sensors + FakeIo   ← 控制循环碰物理世界的唯一边界
bus.rs    DynamixelIo：总线读/写（含 IMU 那一路）
imu.rs    SFLP 姿态解码（字节 → gyro/gravity/quat）
obs.rs    61 维观测拼装
policy.rs ONNX 策略会话与推理
safety.rs 电压/温度/摔倒 → 限幅、松力
fall.rs   摔倒预测（防"硬着陆"）
```

## 3. 关键架构缝：`RobotIo`（这一讲唯一要记住的概念）

`duck-control/src/io.rs:113`：

```rust
pub trait RobotIo {
    fn read(&mut self) -> Result<Sensors>;           // 一次事务：15 舵机 + IMU
    fn write(&mut self, targets: &JointTargets) -> Result<()>;
    fn set_gain(&mut self, kp: u16) -> Result<()>;   // "go limp"靠降增益实现
    fn set_torque(&mut self, on: bool) -> Result<()>;
    fn slow_sensors(&mut self) -> Result<SlowSensors>; // 电压/温度，约 1 Hz
    fn imu_ready(&self) -> bool { true }             // 默认认为 IMU 已收敛
    ...
}
```

`io.rs:6` 的模块注释解释了为什么 `read()` 把**关节和 IMU 一起返回**：

> *`Sensors` carries joints AND IMU together because that is what the hardware does:
> the IMU board sits on the Dynamixel bus and is fetched in the same transaction.*

**这就是你"IMU 不上伺服总线"改造要动的那条缝**：现在 trait 不给 IMU 单开读口，是因为总线
本来就是一包读。等第 3 讲看完 `bus.rs` 你再回来看这句话，会知道为什么拆它要动哪些地方。

`Sensors`（`io.rs:15`）就是一个原子快照：

```rust
pub struct Sensors {
    pub positions: [f64; NUM_JOINTS],     // 弧度
    pub velocities: [f64; NUM_JOINTS],    // rad/s
    pub currents_ma: [f64; NUM_JOINTS],   // 只取幅值（看负载，不看方向）
    pub imu: ImuData,                     // gyro + gravity + quat
}
```

`FakeIo`（`io.rs:166`）是"假鸭子"：`read()` 把你上次 `write()` 的关节角度原样读回，
好像每个舵机都精确跟上了指令。单测和 `robotd --fake` 都用它——**这就是"无鸭可测"的物理基础**。

## 4. 自己动手【无鸭可跑】

先装 Rust（本机当前没有）：

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
# 重开 shell 后：
cargo --version
```

然后只编译并测试控制心脏（不用编译整个 workspace，避免 ort/gstreamer 等重依赖）：

```bash
cd ~/microduck
cargo test -p duck-control
```

**预期**：60 个 `#[test]` 全部通过。它们分成几簇，正好是后续几讲的预览——

| 簇 | 文件 | 例（`cargo test -p duck-control <子串>`） |
|---|---|---|
| 常数表自洽 | `model.rs` | `tables_agree_on_length` / `imu_id_does_not_collide` / `mouth_index_names_the_mouth` |
| 事务/总线换算 | `bus.rs` | `position_conversion_round_trips` / `the_first_block_is_never_stale` |
| IMU 解码 | `imu.rs` | `half_precision_decodes_known_values` / `all_zero_quaternion_bytes_hold_the_last_good_value` / `not_ready_until_the_chip_has_produced_output` |
| 观测布局 | `obs.rs` | `every_block_lands_at_its_documented_offset` |
| 摔倒预测 | `fall.rs` | `the_rate_is_the_analytic_derivative` / `a_fall_fires_once_after_the_debounce` |
| 假 IO 行为 | `io.rs` | `fake_io_tracks_what_was_written` / `simulated_read_failure_clears_itself` |
| 安全裁决 | `safety.rs` | `a_brief_tilt_is_not_a_fall` |

> 说明：策略会话（`policy.rs`）用 `ort` 的 `load-dynamic` 在**运行时**才 `dlopen` 系统里的
> `libonnxruntime`，编译期不需要、多数测试也不触会话。若个别 policy 测试因找不到该库而失败，
> 属预期，不影响核心路径（这正是 `load-dynamic` 的注释在讲的事）。

## 5. 收尾检查：你能说清

- [ ] `Sensors` 里为什么同时装着关节和 IMU？（答：硬件就是同一条总线同一包读的）
- [ ] `FakeIo` 让"无鸭测试"成为可能的机制是什么？（答：写什么读什么，假装舵机完美跟随）
- [ ] `robotd` 和 `duck-control` 的边界由谁保证？（答：编译器——控制核心里没有 tokio/socket）

**下一步**：第 2 讲去 `model.rs` 看那 15 个关节和那排寄存器常数——它们是"为什么总线要 1 Mbps、
为什么 IMU id 是 200"的答案。

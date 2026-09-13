# HD1910-C001 舵机 + I²C IMU 迁移方案（microduck 运行时）

> Status: **已实施，并在 2026-09-13 按实机/BAM 回填修订**。
> 2026-09-04/07 的 §2 是当时的契约；**当前 HD1910 通讯以 §6 为准**（mode 4、WritePosEx、
> BAM 固件表、`FeetechIo` 命名、`[platform].variant`）。§2 未改写，以免把历史实施记录
> 伪装成一直如此。
>
> **实施注记（Phase B · 集成，已落地）**：§2.5（`robotd-params` 的 `Bus.driver` + 新 `[imu]`
> 段 + registry、`robotd/src/main.rs` 的按 driver 选总线 + `ImuIo` 构造 + `Safety` 双泛型 +
> IMU 合成/降级、`deploy/robotd.toml`）已实现。`duck-control/src/imu.rs` 的解码契约未改；
> §2.5 之前的 Phase A 与 §2.6 部署物/§2.7 文档由各自 agent 落地。
>
> **验收记录（2026-09-07，Phase C 由主持 agent 执行）**：
> `duck-control` 73 tests、`robotd-params` 25 tests、`robotd` 97+7 tests 全绿；
> `clippy -D warnings`（dc/params/robotd）0 告警；`cargo fmt --all --check` 通过；
> `cargo check --target aarch64-unknown-linux-gnu`（dc/robotd）通过；
> `imu.rs` / `obs.rs` / `fall.rs` / `odometry` 的 git diff 为空（§3-2/§3-3 满足）；
> 部署物（`i2c4-imu.dts`/`99-robot-i2c-imu` udev/`provision-hls.py`）存在且语法检查通过；
> 唯一新依赖处理：`bisync`（lsm6dsv16x-rs 传递依赖）在 crates.io 全版本 yanked，
> 采用 `vendor/bisync` + `[patch.crates-io]`（注释已说明）。
>
> **修订记录（2026-09-13）**：OpenMicroDuck 与原版 MicroDuck 并存。HD1910 固件写入对齐
> BAM `hd1910/m1.json`（commit `b8b7721`）与真机回读（固件 3.46）：mode=4、Kp/Kd/Ki=32/40/0、
> 保护电流 500 LSB（3.25 A）、电压窗 4.0–8.4 V（**2S 包 7.4 V**；BAM 辨识时 `vin=6.0`）。每拍写改为 HLSCL
> `WritePosEx`（从 ACCEL 起 7 字节），不再只 poke 目标位置 2 字节。类型名 `HlsBusIo` →
> `FeetechIo`。硬件开关是 `[platform].variant`，不是单独改 `bus.driver`。原版 MicroDuck
> （XL330 + 总线 ID 200）相对 `2c61dcc` **不改设计**，`DynamixelIo` 已按当时路径接回。
> O3（Kp/Kd）、O5（电压窗）关闭；O1/O2/O4 仍开。控制环形状与部署改 toml 的步骤见
> [`robotd-design.md`](robotd-design.md) §1.1 / §2.1 / §4.2。

## 0. 背景与资料（已验证）

- **实施环境**：本会话已安装 Rust 工具链（rustup stable 1.98.1 + rustfmt + clippy，
  位于 `~/.cargo`；另装 `aarch64-unknown-linux-gnu` 检查目标）。基线已记录（修改前）：
  `cargo check -p {duck-control,robotd,robotd-params,tof,odometry,kinematics,duck-ipc-proto}`
  全部通过（x86 与 aarch64 均验证）；`cargo test -p duck-control` = 60 passed、
  `cargo test -p robotd` = 7 passed；`cargo fmt --all --check` 通过；ci 门禁为
  fmt+clippy+test（见 CI 工作流），**每个 agent 交付前必须跑 `cargo fmt`**。
  **注意**：宿主无 gstreamer 开发库，`cargo check --workspace` 会因 `mediad`
  （gstreamer-sys）失败——属既有环境限制，本地验证以"本方案涉及的各包"为范围，
  mediad 不在改动范围内；CI 会覆盖全量。
- 舵机：**Feetech HD1910-C001**（12V 9 kg·cm 铁芯恒力双轴 TTL 总线舵机，4096/圈、
  0.087°/unit、中位 2048、77 RPM、22.5 g）。权威资料在 **microduck_rl** 仓库：
  - `docs/feetech_hls_memtable.md` —— 官方内存表（寄存器地址 = 唯一定义）
  - `docs/ftscs_protocol.md` —— FT-HLS 协议（帧格式、0x82 同步读、0x83 同步写、
    0x0B 位置校准；帧格式 = Dynamixel v1 风格：`FF FF ID LEN INST PARAMS CHK`，
    `CHK = ~(ID+LEN+INST+PARAMS) & 0xFF`，多字节小端）
  - `docs/feetech_hls2915_servo_swap.md` —— RL 侧完整分析（BAM/重训，**不在本仓库范围**，
    但其中"运行时寄存器写入值"一节是本方案 §2.1 必须对齐的）
  - `docs/mechanical_swap_requirement.md` —— 机械/供电需求（12V 系统，轴位保持）
- IMU：**LSM6DSV16X + 片上 SFLP**（现 `imu_to_dxl` v2 板同款）。I²C 直驱路径已被验证：
  ST 官方 `lsm6dsv16x-rs`（crates.io v2.0，BSD-3，blocking feature）+ 其
  `bsp/examples/sensor_fusion.rs` 给出完整初始化与 FIFO 解析参考；数据手册
  （DS13510）确认四元数只从 FIFO 出（tag 0x13），组件为 IEEE half —— 与现有
  `imu.rs` 的 `half()` 解码一致。器件路线二选一（接口同一）：
  a) 现有 `imu_to_dxl` 板改固件为 I²C 从机（服务同样的 12 字节块）；
  b) SparkFun SEN-21325（LSM6DSV16X Qwiic，0x6A/0x6B）直驱裸芯片。
- 现状（本仓库，2026-09-13）：**两个变体**。OpenMicroDuck（默认）= 15×HD1910（`FeetechIo`）+
  I²C IMU；原版 MicroDuck = 15×XL330 + IMU ID 200 同总线（`DynamixelIo`，对齐 `2c61dcc`）。
  解码仍在 `duck-control/src/imu.rs`（`SflpDecoder`，12 字节块 → `ImuData`）。
  2026-09-04 动手前的基线是单一 XL330+ID200 路径。

## 1. 硬件前提（不在本仓库改，但决定代码的占位默认）

1. **供电 12 V（9–14 V）**。电池测量语义：现 `model.rs::battery_percent`（8.2/6.6 V）
   假定"舵机上报电压 = 电池"。12 V 系统若为稳压 BEC 直供，该假设失效。
   **开放问题 O1**：代码侧先保留映射并加注释、参数可调；机械/电气侧定案后回填。
2. **IMU 总线 i2c4（引脚 27/28，板载上拉）**，别名 `/dev/i2c-imu`；地址 0x6A。
   备选（配置即支持）：HAT i2c3（注意 tofd 启动时 550 KB 固件上传占用总线）。
3. **mouth（第 15 个）舵机**默认也换 HD1910（同一总线只能一种协议；XL330 是 v2，
   HLS 是 v1 风格帧，不能混）。**开放问题 O2**：若 mouth 保留 XL330，需单独一条总线
   + 单独代码路径，不在本方案内。

## 2. 代码修改清单（文件粒度）

> 目标架构：`RobotIo`（舵机）与 `ImuIo`（IMU）分离；`Sensors` 结构与所有消费者
> （obs / safety / fall / odometry / 状态流）**不变**；`imu.rs` 的 `SflpDecoder`
> **零行改动**（12 字节块契约不变）。

### 2.1 `duck-control/src/ftbus.rs`（新）—— HD1910 总线驱动

> **§6 已取代本节里的写入值与每拍写格式。** 下面保留 2026-09-04 的契约原文（`HlsBusIo`、
> mode=0、写 `0x2A`×2、Kp/Kd=32/32、12 V 电压窗），便于对照当时为什么那样写。落地代码是
> `FeetechIo`，见 §6。

FT-HLS 帧原语 + `HlsBusIo`（实现现有 `RobotIo` trait）。

- **帧/校验和**：按 `ftscs_protocol.md`；指令：PING 0x01、READ 0x02、WRITE 0x03、
  SYNC_READ 0x82、SYNC_WRITE 0x83、位置校准 0x0B（无参数=校到中位；HLS ≥3.43 支持，
  **不支持**扭矩开关写 128）。广播 ID 0xFE。
- **传输层（已定案，源码核实过 rustypot 1.6.0）**：直接用 rustypot 的
  `Sts3215Controller` + `with_protocol_v1()` **只当纯传输层**（其命名寄存器表与 HLS
  在 34/35/36 处不同，地址一律自定常量），调用宏生成的
  `sync_read_raw_data(&ids, addr, len)` / `sync_write_raw_data(&ids, addr, &data)`。
  核实结论：rustypot proto1 的 `sync_read` 就是"发 0x82 包 → 按 ID 顺序逐个读取应答帧"，
  `StatusPacketV1` 的校验和算法与 FT-HLS 一致（`~(ID+LEN+ERR+DATA) == CHK`），
  params 提取（`data[5..3+LEN]`）与 FT-HLS 应答帧对齐，且**非零状态字不视为错误**
  （以 `errors()` 暴露，帧仍解析成功）——完全兼容；`sync_write`（0x83，广播，
  读回无应答）同样兼容。30 ms 超时照抄 `bus.rs`。若上板发现兼容性问题，回退自实现
  （参照 `microduck_rl/scripts/read_hls_registers.py`）。
- **寄存器常量**（DEC，来自内存表）：mode 33、torque_enable 40、accel 41、
  goal_position 42(2B)、goal_current 44(2B)、goal_speed 46(2B)、torque_limit 48(2B)、
  Kp 50 / Kd 51 / Ki 52、lock 55、present_position 56(2B)、present_speed 58(2B)、
  present_load 60(2B)、present_voltage 62(1B)、present_temperature 63(1B)、
  moving 66、present_current 69(2B)、current_bias 73(2B)、offset 31(2B)、
  angle limits 9/11、max_temperature 13、min/max_voltage 15/14、max_torque 16。
- **每拍读**：SYNC READ `(0x38, 15)` = pos/speed/load/voltage/temp/async/status/
  moving/target/current 共 15 字节（56..70）。
- **单位换算**（冻结，与 RL 侧一致）：
  - 位置：`rad = (2π·p / 4096) − π`（p 为 i16，单圈 0..4095；BIT15 为方向位，
    多圈禁用下无效，转换前按 0..4095 校验）；
  - 速度：`rad/s = v_raw × 0.732 × 2π / 60`（0.732 RPM/LSB，BIT15 方向；i16）；
  - 电流：`mA = |raw| × 6.5`（BIT15 方向；bias 73 暂不扣除，**O4**）；
  - 电压：`V = u8 × 0.1`；温度：`°C = u8`；负载：`% = u16 × 0.1`（BIT10，仅监控）。
- **每拍写**：SYNC WRITE `(0x2A, 2)` 目标位置（i16 小端，`(rad+π)·4096/2π`，
  与位置读换算互逆；沿用现有 round-trip 测试风格）。
- **RobotIo 方法**：`read`（解析上表，`currents_ma`/`velocities`/`positions`）、
  `write`、`set_torque`（40 写 0/1，可选 2=阻尼给 limp 用——**先按 0/1**）、
  `set_gain(kp)`（映射语义：**kp 参数即固件 50 号寄存器原始值**；Kd/Ki 写 51/52 = 0？
  按 RL 侧："位置模式 Ki 无效"→ 写 0；Kd 写与 sim 一致值。**默认值 O3**：
  先 `Kp=32, Kd=32`（STS3215 参考量级）并注释"标定后回填"）、
  `slow_sensors`（**直接返回 tick 缓存**，不再发额外事务；volts/temps 来自本 tick 块）、
  `present_positions`、`interpolate_to`、`check_registers`（见下）。
- **启动断言 & 校正（`check_registers`，仿 `EXPECTED_REGISTERS`）**：
  baud 6=0(1M)、应答级别 8=1、angle limits 9/11=0/4095、max_temp 13=70、
  min_voltage 15 / max_voltage 14（12V 系统：min=100(10.0V)、max=140(14.0V)，
  **O5** 标定前不动，仅列出）、max_torque 16=980、mode 33=0、accel 41=0（最大加速度）、
  goal_speed 46=32767、torque_limit 48=980、lock 55=1。EEPROM 写入须先解 lock（55=0）
  再回锁——只在 `check_registers` 与 ID 配置流程里做。
  - 注意：mode/accel/speed 都是 SRAM（40-52 区），掉电恢复 —— 每个开机都要写，
    放 `open()`/`check_registers`；EEPROM（5-39 区）仅在需要校正时写。

### 2.2 `duck-control/src/io.rs` + `safety.rs` —— 接口拆分（IMU 分离）

- `Sensors` **结构不变**；`RobotIo`：
  - `read()` 语义改为"舵机反馈；`imu` 字段由调用方填充"（模块注释重写；
    原"同一次事务"论证作废，改为"IMU 走独立总线，由调用方同拍合成"）；
  - **移除** `imu_stale()` / `imu_ready()` 默认方法（或保留默认但语义改为
    "无 IMU"，**倾向移除**——由 `ImuIo` 承担，避免双真相）；
  - 其余方法不变。
- 新增 trait `ImuIo`：
  ```rust
  pub trait ImuIo {
      /// 读一个 IMU 样本（50 Hz，与 servo read 同拍调用）。
      fn read(&mut self) -> Result<ImuData>;
      /// 融合是否已收敛（闸门：fall 检测/odometry 播种）。
      fn ready(&self) -> bool;
      /// 字节相同重复计数（健康上报）。
      fn stale(&self) -> ImuStale;
  }
  ```
- `Safety<T: RobotIo, U: ImuIo>`：`new(io, imu, config)`；`imu_ready()`/`imu_stale()`
  委托给 `U`；`read()`/`slow_sensors()`/`set_torque()` 不变。`safety.rs` 内测试
  （`FakeIo`）全部同步改为双泛型，更新 `Safety::new` 调用点。
- `FakeIo` 不动（Sensors 字段仍可 set_imu 给测试直接用）；新增 `FakeImu`
  （`ready` 可控、可注入 ImuData、可注入错误/陈旧计数），供 robotd 测试与
  `--fake` 路径使用。
- `lib.rs` 导出：`ImuIo`, `FakeImu`（如通用）。

### 2.3 `duck-control/src/imui2c.rs`（新）—— LSM6DSV16X I²C 直驱（linux-only）

- **直接复用官方 `lsm6dsv16x-rs`（crates.io，v2.1.0，MSRV 1.85 < 仓库 rust-version 1.89 ✓）
  blocking 特性**：`default-features = false, features = ["blocking"]`；它依赖
  `embedded-hal 1.0` + `st-mems-bus`，`Lsm6dsv16x::from_bus(bus, delay)` 需要
  `BusOperation`（对实现了 `embedded_hal::i2c::{Read,Write,WriteRead}` + `DelayNs`
  的任意类型自动可用）。因此本模块只实现一个 **`I2cDev`（Linux i2c-dev，ioctl
  `I2C_RDWR`，新增依赖 `libc`）** + `StdDelay`（`std::thread::sleep`）即可，芯片
  寄存器/页切换（DSV16X 嵌入式函数页）全部由官方驱动处理——**不要手写寄存器序列**。
- 初始化序列（官方 `sensor_fusion` 示例 + 已核实枚举）：WHO_AM_I（0x0F=0x70）→
  复位 → BDU → `XlFullScale::_4g`、**`GyFullScale::_500dps`**、ODR `Odr::_120hz`、
  `SflpDataRate::_120hz` → `fifo_watermark_set(8)` → `fifo_sflp_batch_set(
  game_rotation=1)`（可选 gravity=1 供调试）→ `fifo_mode_set(StreamMode)` →
  `sflp_game_rotation_set(1)` → `sflp_game_gbias_set(0)`。
- 每拍读（ImuIo::read）：`fifo_status_get()`（`fifo_th`/`fifo_level`）→ 循环
  `fifo_out_raw_get()`（`FifoOutRaw { tag, data[6] }`）取**最新**：
  - tag **0x01 = Gyroscope NC**（注意 DSV16X 与老 LSM6DS 相反：0x01=陀螺、0x02=加速度！
    已从 DS13510 FIFO tag 表核实）、tag **0x13 = SFLP game rotation**（3×half LE）；
  - 组 12 字节块 `[gyro 3×i16 LE][quat x/y/z half LE]` → `SflpDecoder::decode`（复用）；
  - 陀螺原始计数与 `imu.rs::GYRO_RAD_PER_LSB`（±500 @ 17.5 mdps/LSB）匹配。
- `SflpDecoder`/`StaleImuTracker`/`ready()` 由 `ImuIo` 实现持有（stale 对组好的块比较）。
- `#[cfg(target_os = "linux")]` 实现，其他平台 `open` 返回明确错误（参照 `tof::sensor`）；
  设备路径/地址来自 `[imu]` 配置（`/dev/i2c-imu`、0x6A）。

### 2.4 `duck-control/src/model.rs` + `bus.rs`

- `model.rs`：2026-09-04 方案要求删除 `IMU_DXL_ID`。**未删** — MicroDuck 仍用 200，
  OpenMicroDuck 预留给以后的飞特 IMU 节点。`JOINT_IDS` / `DEFAULT_POSITION` 不动。
  HLS EEPROM 表在 `ftbus.rs`（`HLS_EEPROM_EXPECTED`），不与 XL330 `EXPECTED_REGISTERS` 混表。
- `bus.rs`（`DynamixelIo`）：2026-09-04 方案要求去掉 ID 200。**2026-09-13 已撤回该条**：
  MicroDuck 必须保持 `2c61dcc` 的 combined `sync_read`。OpenMicroDuck 不使用本类型。

### 2.5 `robotd-params` + `robotd` 集成

- `robotd-params/src/lib.rs`：
  - `Bus { port, driver }`，`driver: "hd1910" | "xl330"`（默认 `"hd1910"`；
    默认值即迁移目标，旧机器人用 `driver="xl330"` 回退）；
  - 新增 `Imu { bus: PathBuf("/dev/i2c-imu" 默认), address: u8(0x6A) }` 段；
  - `registry.rs` 同步新增 key（说明/默认值/枚举），并满足其一致性测试。
- `robotd/src/main.rs`：
  - 按 `params.bus.driver` 构造 `HlsBusIo` 或 `DynamixelIo`；构造 `ImuIo`
    （真实现 / `FakeImu` in fake); `Safety::new(io, imu, cfg)`；
  - 控制循环（~1339 行）：`let s = safety.read()?;` 后
    `match imu.read() { Ok(d) => s.imu = d, Err(e) => { 保留上次 ImuData；计数；按 1/10 打日志 } }`
    —— **IMU 读失败不再让整拍失败**（本次迁移的核心收益）；
  - 健康上报：`imu_stale_*`/`imu_ready` 从 imu source 取；`adopt_startup_pose`
    走 bus（不变）；`publish_slow_sensors` 不变（内部走新 slow_sensors 缓存路径）；
  - `--fake` 路径：`FakeIo` + `FakeImu`；
  - robotd 内测试全部更新（`Safety` 双泛型、`control_loop` 签名）。
- `deploy/robotd.toml`：`[bus] port` 注释更新 + `driver`；新增 `[imu]` 段注释。

### 2.6 部署物 & 脚本

- `deploy/overlays/i2c4-imu.dts`（新）：仿 `i2c3-pihat.dts`；`&i2c4` `status="okay"`
  + `i2c4m0_xfer` pinctrl（名字以 vendor DTB 为准，上板确认）+ 400 kHz；
  （i2c4 与 fusb302 无关，无需 disable 子节点；仍要上板验证无占用。
  注：RK3566 i2c3=fe5c0000（udev 规则实测），按序 i2c4=fe5d0000，以 vendor DTB 为准。）
- udev 规则：仿 `scripts/setup-board.sh` §702-713 的
  `/etc/udev/rules.d/99-robot-i2c-pihat.rules` 写法，新规则
  `SUBSYSTEM=="i2c-dev", KERNELS=="fe5d0000.i2c", SYMLINK+="i2c-imu"`；
  `scripts/setup-board.sh` 中 `REQUIRED_OVERLAY` 逻辑不动（uart2-m0 保持），
  但 §109 附近注释"our IMU rides the Dynamixel bus rather than I²C"要改写；
  `i2c4-imu` overlay 加入 overlays 列表（沿用 `configure_overlay`）。
- `scripts/provision-hls.py`（新，pyserial，参照 microduck_rl `read_hls_registers.py`）：
  逐台：PING → 校验 → id=目标值、baud=0、lock=1（先解 lock）；可选 0x0B 位置校准
  （装到机械位后"校到中位"）。文档写明用法与风险（单台连接，广播禁止）。
- `scripts/board-test.sh` / `scripts/setup-board.sh`：i2c4 overlay、`/dev/i2c-imu`
  存在性检查；串口 getty 掩码逻辑不变。

### 2.7 文档

- `docs/design/robotd-design.md`：§1.1 双变体总线图、§2.1 HD1910/XL330 事务、§4.2
  `[platform].variant` 与部署时改 `robotd.toml` 的步骤；
- 本文件 §6：当前 HD1910 通讯（取代 §2.1 中已过时的写入值）；
- `docs/design/architecture.md` 相应图示/文字；`docs/robot/cheatsheet.md`、
  `docs/robot/install-by-hand.md`；`docs/microduck-fact-check.md`；
- `source-lessons/lessons/03-bus-imu.md`（明确标注为本评估依据，需修订）；
- `deploy/robotd.toml` 注释（§2.5 / `robotd-design.md` §4.2）。

### 2.8 测试（每文件随附带，全部跑 `cargo test -p duck-control` 与 workspace）

- ftbus：帧校验和/长度；**协议手册示例帧直接做测试向量**（PING 例 1、READ 例 2、
  校验和例 7/8 的字节序列）；位置/速度/电流/电压换算（含符号/BIT15）；
  round-trip 位置互逆；stale/块解析虚设备（新 `FakeBus`? 或对解析函数单测——不要
  引入串口 mock，仿现有做法：解析与换算函数独立可测）。
- imui2c：12B 块组装（合成 FIFO tag 数据 → 块 → `SflpDecoder` 输出值断言）；
  half 解码复用现有测试。
- safety/io：双泛型、`FakeImu`、`imu_ready` 闸门测试更新。
- robotd：`driver` 选择、IMU 失败降级（整拍不失败、保留上次值、错误计数）、
  健康上报来源。

## 3. 验收标准（完成后逐条核对）

1. `cargo test --workspace` 全绿（x86 无硬件环境）；`cargo clippy` 无新增警告；
2. `duck-control/src/imu.rs` **零改动**（git diff 验证）；
3. `obs.rs` / `fall.rs` / `odometry` **零改动**；
4. IMU 故障只降级不 fail tick（有测试证明）；
5. ftbus 全部换算/帧行为有单测（含协议手册向量）；
6. 新部署物（overlay/udev/脚本）存在且 install.sh 引用；
7. 文档段落更新（§2.7 清单逐项完成）；
8. 本方案文件本身更新为"已实施"（strike 或注记）。

## 4. 多 agent 执行计划（≥18:00 开始）

- **契约冻结**：本文件 §2.1–§2.8 为契约；各 agent 先读本文件与
  `microduck_rl/docs/{feetech_hls_memtable,ftscs_protocol,feetech_hls2915_servo_swap}.md`。
- **Phase A（并行）**：
  - A1 舵机总线：`ftbus.rs`（含换算/常量/测试向量）+ `model.rs`/`bus.rs` 调整；
  - A2 接口拆分：`io.rs`（RobotIo 摘 IMU + ImuIo trait + FakeImu）+ `safety.rs`
    （双泛型）+ `lib.rs` 导出 + 相关单测更新；
  - A3 IMU I²C：`imui2c.rs` + `Cargo.toml`(libc) + 解码契约测试；
  - A4 部署物/脚本/文档：overlay、udev、provision 脚本、board/setup 脚本、
    docs（§2.6/§2.7）。
- **Phase B（串行集成）**：合并 A1–A4；`robotd-params`（Bus.driver + Imu 段 +
  registry）+ `robotd/src/main.rs` 集成 + robotd 测试更新；`deploy/robotd.toml`；
  全量 `cargo test --workspace` + clippy，修到绿。
- **Phase C（复核）**：对照 §3 验收清单逐条核对（独立 review agent）；
  产出"未决 O1–O5 清单"与建议回填值。

## 5. 未决项（O）与风险

- O1 电池测量来源（HD1910 真机电压窗是 4.0–8.4 V / **2S 包 7.4 V**，
  `battery_percent` 的 8.2/6.6 V 映射是否仍表示「包电压」待电气定案。若替换为HL3915/HL2915/HL2909这种，电压为9~12V）。
- O2 mouth 是否换装 HD1910（同一 UART 不能混 XL330）。
- ~~O3 Kp/Kd 默认值~~ **已关闭（2026-09-13）**：BAM / 真机回读 Kp=32、Kd=40、Ki=0。
- O4 电流偏置（reg 73）仍未从 `present_current` 扣除。
- ~~O5 min/max 电压寄存器~~ **已关闭（2026-09-13）**：EEPROM 写 40/84（4.0 V / 8.4 V）。
- 风险 R1：无实机时寄存器/时序只能按文档 + 单测保证；上板后 `hls_control` 已证明
  WritePosEx 与 mode 4 是 HD1910 能走步的组合。
  R2（已消除）：rustypot proto1 帧/校验和/应答解析与 FT-HLS 协议源码级核对一致
  （见 §2.1 传输层），无需自实现帧层；
  R3：`Sensors.imu` 由调用方填充后，任何新消费者可能忘记填充 —— 用
  `FakeIo`/文档醒目注释 + robotd 合成点唯一化缓解；MicroDuck 的 `DynamixelIo` 在
  `RobotIo::read` 里已经填好 imu，合成点仍调用 `ImuIo::read` 以免两套循环。
  R4：与 BAM / microduck_rl 的 sim 参数必须一致，否则 sim2real 错位 —— 运行时写入值
  现以 `HD1910_FIRMWARE` 为单一来源；六项 M1 识别参数不写舵机。

## 6. 落地后的 HD1910 通讯设计（2026-09-13，权威）

OpenMicroDuck 的舵机路径。实现：`duck-control/src/ftbus.rs`（`FeetechIo`）+
`duck-control/src/hardware.rs`（`HD1910_FIRMWARE` / `HD1910_BAM_M1`）。
原版 MicroDuck 不走本节，见 [`robotd-design.md`](robotd-design.md) §2.1「MicroDuck」。

### 6.1 帧与传输

与 §2.1 传输层定案相同，未改：

- 帧：`FF FF ID LEN INST PARAMS CHK`，`CHK = ~(ID+LEN+INST+PARAMS) & 0xFF`，多字节小端。
- 指令：PING 0x01、READ 0x02、WRITE 0x03、SYNC_READ 0x82、SYNC_WRITE 0x83、
  位置校准 0x0B；广播 0xFE。
- 传输：`rustypot` `Sts3215Controller` + `with_protocol_v1()` **只当纯传输**。命名寄存器表
  在 34/35/36 与 HLS 不同，**地址一律用 `ftbus::reg`**，只调用
  `sync_read_raw_data` / `sync_write_raw_data` / `read_raw_data` / `write_raw_data` / `ping`。
- 超时 30 ms。1 Mbps（baud 寄存器 = 0）。ID 表仍是 `JOINT_IDS`（20–24 / 30–34 / 10–14）。
  `IMU_DXL_ID`（200）留给以后的飞特 IMU 节点，**现在不 ping**。

### 6.2 每拍读

`SYNC READ` 起始地址 **56（0x38）**、长度 **15**：

`pos(2) speed(2) load(2) voltage(1) temp(1) async(1) status(1) moving(1) target(1) current(2)`

即 56..70。一次事务覆盖关节反馈和 volts/temps；`slow_sensors()` 返回本拍缓存，不再另发。

换算（冻结）：

| 量 | 公式 |
|---|---|
| 位置 | `rad = (2π · p / 4096) − π`，`p` 为 u16、单圈 0..4095；BIT15 置位视为多圈，拒绝 |
| 速度 | `rad/s = ±(raw & 0x7FFF) × 0.732 × 2π / 60`，BIT15 为方向 |
| 电流 | `mA = (raw & 0x7FFF) × 6.5`（丢掉方向；bias 73 仍不扣，O4） |
| 电压 | `V = u8 × 0.1` |
| 温度 | `°C = u8` |
| 负载 | `% = (raw & 0x3FF) × 0.1`（BIT10 方向，仅监控） |

### 6.3 每拍写：WritePosEx，不是 2 字节 goal position

只写寄存器 42（goal position）会把 goal-torque（44）留在 0，HD1910 **不会按预期迈步**
（`hls_control` 真机结论）。因此 `FeetechIo::write` 对每个关节发一块 HLSCL **WritePosEx**：

```text
SYNC WRITE 起始地址 = ACCEL (41)，每关节 7 字节：
  [ acc | pos_l pos_h | torque_l torque_h | speed_l speed_h ]
```

生产字段：

| 字段 | 值 | 含义 |
|---|---|---|
| acc | 0 | 最大加速度 |
| pos | `(rad+π)·4096/2π`，钳位到 0..4095 | 与位置读互逆 |
| torque | 980 | 0.1% LSB，与 `max_torque` / `torque_limit` 同一上限 |
| speed | **0** | 最大 **steps/s**（不是 present-speed 的 0.732 RPM/LSB，也不是旧稿的 32767） |

编码函数：`ftbus::encode_write_pos_ex`。

### 6.4 固件写入值（BAM / 真机回读）

单一来源：`duck_control::HD1910_FIRMWARE`。来自 `bam/params/hd1910/m1.json`
（commit `b8b7721`）配套的执行器设定，以及 FT-SCS 固件 3.46 真机回读。
**不是** 2026-09-04 草稿里的 STS3215/12 V 占位。

| 寄存器 | 值 | 何时写 |
|---|---|---|
| 33 mode | **4**（纯位置 PD；不限目标斜率） | `open()` 解 lock 后写，EEPROM；`check_registers` 再断言 |
| 50 Kp | 32（可由 `set_gain` 覆盖，钳位 0..254） | RAM，控制环按 variant 默认 32 |
| 51 Kd | **40**（厂默 D，不是 32） | 随 `set_gain` 每次写 |
| 52 Ki | 0（位置模式无效） | 随 `set_gain` |
| 28 保护电流 | 500 LSB = **3.25 A**（6.5 mA/LSB） | EEPROM，`check_registers` |
| 14 / 15 电压窗 | 84 / 40 → **8.4 V / 4.0 V** | EEPROM |
| 16 / 48 扭矩上限 | 980 | EEPROM / `open()` 写 48 |
| 6 baud | 0（1 Mbps） | EEPROM |
| 8 应答级别 | 1 | EEPROM |
| 9 / 11 角度限制 | 0 / 4095 | EEPROM |
| 13 最高温度 | 70 | EEPROM |

Mode 4 与 mode 0（HLS「带力限的角度伺服」）不是一回事。BAM 在 mode 4、
`use_rate_limiting = false` 下辨识，所以运行时必须写 4。寄存器 33 在 EEPROM 窗
（5..=39）：锁定时写是静默丢弃，`open()` / `hls_control` 都必须先 `LOCK=0`。

六项 BAM M1 识别参数（`HD1910_BAM_M1`）**不写舵机**：

`kt=0.736` N·m/A，`R=3.75` Ω，`armature=0.0008` kg·m²，`q_offset=0`，
`friction_base=0.05` N·m，`friction_viscous=0.008` N·m/(rad/s)。

典型供电 **7.4 V**（2S 包，`HD1910_VIN`），对应 `policy.nominal_voltage` 两变体的未设置默认。
BAM 辨识用过 `vin=6.0`，那是模型条件，不是这块电池。

### 6.5 启动顺序

1. 打开 `/dev/ttyS2`，30 ms 超时。
2. 对 `JOINT_IDS` 逐个 PING；有一个不应答则 `open` 失败（上层等待上电）。
3. `apply_startup`：扭矩关 → 解 lock → **mode 4** → 上 lock → `torque_limit=980`。
   不因进程启动而开扭矩。
4. `check_registers`：解 lock，按 `HLS_EEPROM_EXPECTED` 比对/校正，再上 lock。
5. 控制环第一拍才 `set_gain(32)`（或 toml 覆盖）并按意图 `set_torque`。

Limp-fall 仍是降 kP（默认 8），不是 torque-enable=2 阻尼模式。

### 6.6 IMU（本变体）

舵机 UART 上没有 IMU。LSM6DSV16X 走 I²C（`[imu].bus` 默认 `/dev/i2c-imu`，地址 0x6A；
打不开则按 `imu_probe` 扫描 `/dev/i2c-*` 的 WHO_AM_I=0x70）。FIFO → 12 字节块 →
`SflpDecoder`（与 v2 `imu_to_dxl` 块布局相同）。I²C 读失败只降级本拍姿态，不让整拍失败。

安装朝向必须对齐 `SflpDecoder::DEFAULT_MOUNT`（绕 Y +90°：`trunk = [+raw_z, +raw_y, −raw_x]`）：
芯片 **+X 朝下、+Y 朝左、+Z 朝前**。装好后把鸭子**直立站稳**，`imu_probe` 的 `gravity`
应接近 **`[0, 0, -1]`**（Z ≈ −1，XY 接近 0）。这是策略观测和摔倒检测吃的量，不是加速度计原始轴。
芯片封装朝上平放在桌上时，同一套旋转会报约 `[-1, 0, 0]`——那是未按设计安装时的姿态，不是坏芯片。
`ready=false` 且 gravity 停在 `[0, 0, -1]` 才是 decoder 默认值（FIFO 还没组出 SFLP 块）。

以后若飞特 IMU 节点上总线，复用 ID 200，形状对齐原版 MicroDuck；那是后续工作，不是现在。

### 6.7 与原版 MicroDuck 的边界

| | OpenMicroDuck（本节） | MicroDuck |
|---|---|---|
| 代码 | `FeetechIo` | `DynamixelIo`（`2c61dcc` 路径，不要改） |
| 协议 | FT-HLS v1 风格 | Dynamixel v2 |
| 每拍读 | 15× HD1910，0x38×15 | **16 设备**：ID 200 + 15× XL330，124×12 |
| 每拍写 | WritePosEx 7 B | XL330 goal position |
| IMU | I²C `ImuIo` | 同一次 `sync_read` 的 slot 0 |
| toml | `variant = "open_microduck"` | `variant = "microduck"`（§4.2） |

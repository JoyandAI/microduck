# 03 · 总线读与姿态解码：bus.rs 与 imu.rs

> 目的：看清一次事务的物理实现——一条 `sync_read` 怎么把 15 个舵机的字节变成弧度，以及
> 姿态怎么从芯片里的 SFLP 融合块一路变到 `gravity`；再看 IMU 改成独立 I²C 总线后，
> 谁把 12 字节块递给 `SflpDecoder`。
> **这一讲最初是你"IMU 不上伺服总线"评估的直接依据；该评估 2026-09-04 已按迁移方案落地。**
>
> **迁移注记（2026-09-04，`hd1910-i2c-imu-migration.md`）。** 本讲描述的是**迁移后**结构：
> 舵机换 Feetech HD1910-C001（FT-HLS），IMU **不在舵机总线上**，改走 `i2c4` →
> `/dev/i2c-imu`（LSM6DSV16X，0x6A），由 `duck-control/src/imui2c.rs` 的 `ImuIo` 实现读取；
> `SflpDecoder` **零行改动**（12 字节块契约不变）。§1 的"同一包读"是旧 XL330 时代的做法，
> 保留作对照，标注哪几行已失效。

## 1. bus.rs：同一条总线、同一包读（—— XL330 时代的旧结构）

> **迁移后：这条"同一包读"已被拆开（2026-09-04）。** 舵机总线改由
> `duck-control/src/ftbus.rs`（`HlsBusIo`）驱动，用 rustypot 只当 FT-HLS 纯传输层；
> IMU 由 `duck-control/src/imui2c.rs` 的 `ImuIo` 在 `i2c4` 上单读。下面 §1 逐行描述的是
> **旧 `bus.rs`（DynamixelIo/XL330）**，保留作对照——`Sensors` 结构与换算方向不变，
> 只是 IMU 不再排在 `sync_read` 里；已失效的"和舵机同包读 IMU"相关行已在此标注。

`duck-control/src/bus.rs:3` 模块注释把每拍结构说死（旧）：

> *One combined `sync_read` per tick covering the IMU board and all 15 servos, and one
> `sync_write` of goal positions. **The IMU is listed first so it answers before the servo***
> …（总线读慢速传感器是单独事务）

### 1.1 设备清单与一块"恰好 12 字节"的公共区

```rust
const READ_ADDR: u8 = 124;
const READ_LEN: u8 = 12;
const READ_TIMEOUT: Duration = Duration::from_millis(30);
```

`bus.rs:114` 组 ID 清单：

```rust
let mut ids = Vec::with_capacity(NUM_JOINTS + 1);
ids.push(IMU_DXL_ID);              // id 200 排第一
ids.extend_from_slice(&JOINT_IDS); // 再跟 15 个舵机
```

`open()`（`bus.rs:101`）用一个 `serialport` 打开 `/dev/ttyS2`，波特率 `BAUD_RATE`（1 Mbps），
超时 30 ms，协议 `Xl330Controller` 走 Dynamixel **v2**。Dynamixel 的 `sync_read` 允许**同一地址、
同一长度、多个 id 一次读出**——官方利用这点，让 IMU 板和舵机都"服务"在地址 124、各回 12 字节：
舵机的那 12 字节正好是 `present_pwm/current/velocity/position` 连续区；IMU 板的 12 字节是
SFLP 姿态块（见下）。一次事务 16 块，全部到手。**这是"一次快照"的物理真相。**

### 1.2 read()：块怎么变成 `Sensors`

`bus.rs:233`：

```rust
fn read(&mut self) -> Result<Sensors> {
    let blocks = self.controller
        .sync_read_raw_data(&self.ids, READ_ADDR, READ_LEN) ...;
    if blocks.len() != self.ids.len() { return Err(IoError::ShortRead{...}); }
```

先校验块数——少一块 = 有设备没应答，直接报错而不是把旧值留一半（注释：*a silently short
read would leave stale values in half the joint array*）。

- `blocks[0]` 是 IMU（len 必须 `== IMU_BLOCK_LEN` 12）→ 喂给 `SflpDecoder.decode()`，同时过一个
  `StaleImuTracker`：连续 `STALE_RUN_WARN = 25` 拍（0.5 秒）收到同一字节 → 报 "orientation is
  frozen"（`bus.rs:257` 那段，rate-limit 到每 500 拍一次免得刷爆日志）。
- `blocks[1..]` 逐个拆舵机（`bus.rs:269` 附近）：`[2..4]` 电流取幅值 mA、`[4..8]` 速度 ×
  `RAD_PER_SEC_PER_COUNT`（= 0.229 × 2π/60，datasheet 系数）、`[8..12]` 位置 `(2π·pos/4096) − π`
  → 弧度。

### 1.3 剩下的口

- `write()`：`sync_write_goal_position(&JOINT_IDS, &targets.positions)` —— 只写位置。
- `check_registers()`（`bus.rs:132`）：启动时只对 `JOINT_IDS` 断言并纠正那排 EEPROM 常数
  （第 2 讲那张表），IMU 不在内。它一次 register 一读，`rustypot` 对单 id 也返回 Vec。
- `slow_sensors()`：约 1 Hz、**不在** tick 关键路径上另开事务读 144–146（电压/温度）。
  `io.rs` 注释：pack 不会在 20 ms 内耗尽、电机不会在 20 ms 内烧起来，所以不值得每拍为它付一次事务。
  电压是"穿过总线看到的电池"——`model.rs` 那张 8.2/6.6 电量表吃的就是它。
- 启动路径另有 `present_positions()`（只读当前位置），让 `robotd init` 采纳机器人当前的姿势。

## 2. imu.rs：从 SFLP 块到 gravity

`imu.rs:1` 顶部注释点题，是你改造时绕不开的一句话：

> *One IMU, one code path. … **no separate sensor to poll and no fusion to run on the host** —
> the chip's SFLP block ships a game-rotation quaternion and estimates its own gyro bias.*

**融合在芯片里，不在主机上。** 这是后面一切的前提。

### 2.1 块布局（地址 124，前 12 字节）

| bytes | 内容 |
|---|---|
| 0..6 | gyro x/y/z，`i16` LE，±500 dps |
| 6..12 | SFLP 四元数 x/y/z（IEEE half），`w = √(1−x²−y²−z²)` |

完整诊断块其实是 20 字节（还带裸加速度计、采样计数、状态位），但控制循环**只消费前 12**。
（旧 XL330 时代为了塞进和舵机同一次事务才只读前 12；迁移后 IMU 走独立 I²C 总线，块布局
不变，12 字节契约照旧——"IMU 在总线上"带来的这个取舍已经不存在，2026-09-04。）

### 2.2 解码：五步，每步都有注释讲"为什么"

```rust
pub const GYRO_RAD_PER_LSB: f64 = 0.0175 * PI / 180.0;  // ±500dps @ 17.5 mdps/LSB
```

`SflpDecoder::decode()`（`imu.rs:103`）：

1. **gyro**：3× `i16` LE × 量纲 → rad/s，然后用 `mount`（安装旋转）从**传感器系**转到**躯干系**。
2. **quat 防御**：三个 half 全 0 = SFLP 还没写表（刚上电/初始化失败）→ **保留上次好值**
   （`last_quat`），绝不回落到 identity——否则摔倒检测会收到"明明姿势未知却报直立"这种最坏的谎。
   校验 `norm_sq ≤ 1.02` 防芯片没产出的垃圾数。
3. 把芯片四元数乘上 `mount` 的逆 → 躯干→世界；单位化后更新 `last_quat`、`quat_samples += 1`。
4. **gravity**：`gravity = normalise(rotate_inverse(last_quat, [0, 0, -1]))` —— 把"世界系竖直向下"
   反旋回躯干系。静止直立 = `[0, 0, -1]`，这正是观测与摔倒检测阈值所吃的量（`ImuData.gravity`）。
5. **去尖峰**：gyro 与 gravity 各自过一个 **median-of-3**（单拍野值被中值滤掉，比均值少滞后）。

`mount` 常数（`imu.rs:78`）：

```rust
pub const DEFAULT_MOUNT: [f64; 4] = [FRAC_1_SQRT_2, 0.0, FRAC_1_SQRT_2, 0.0]; // +90° 绕 Y
```

注释：*board is mounted so that trunk = [+raw_z, +raw_y, −raw_x]*。**你直连 IMU 时，安装朝向
必须和这张表一致，或换一张新的 `mount`**——装歪了 gravity 就错，策略与安全一起错。

`ready()`（`imu.rs:99`）：

```rust
pub fn ready(&self) -> bool { self.quat_samples >= 25 }  // ~0.25 s @ 100 Hz
```

**直到芯片证明自己产出过融合输出之前，方向都是默认值而不是测量值**——摔倒检测不许在它之前跑。
（这个 25 拍到第 5 讲会再出现：它是 odometry 播种、limp-fall 的闸门。）

### 2.3 迁移后：IMU 走独立 I²C 总线（已按评估落地）

结论即当初评估所指的第一条路（2026-09-04）：

- **保留 LSM6DSV16X + SFLP**，直连 `i2c4` → `/dev/i2c-imu`（地址 0x6A）：`SflpDecoder`
  原封复用，`imu.rs` 一行不改，只换"谁把 12 字节递给它"——现在是
  `imui2c.rs` 的 `ImuIo::read()`，用官方 `lsm6dsv16x-rs`（blocking 特性）直驱裸芯片，
  组 12 字节块 `[gyro 3×i16 LE][quat x/y/z half LE]` 后喂给 `SflpDecoder`。
  注意 DSV16X 的 FIFO tag 与老 LSM6DS **相反**：0x01=陀螺、0x13=SFLP game rotation。
- **不再需要**"若换成普通串口 9 轴 IMU"那条路（那会把第 2/4 步融合挪到主机）。
- `RobotIo` 摘掉 `imu_stale()/imu_ready()`（迁到 `ImuIo`，避免双真相）；`Safety<T, U>`
  双泛型持有 `RobotIo` + `ImuIo`；`Sensors` 结构与全部消费者（obs/fall/odometry/状态流）不变。
- 验收：IMU 读失败只降级不 fail tick；`imu.rs`、`obs.rs`、`fall.rs`、`odometry` **零改动**。

### 2.4 迁移后的寄存器对照（ftbus.rs）

| 旧 (Dynamixel XL330) | 新 (Feetech HLS, FT-HLS) |
|---|---|
| 每拍 `sync_read` @ 124–136 (12 B) | 每拍 `sync_read` @ `0x38` (15 B = 位置/速度/负载/电压/温度/状态/moving/目标/电流) |
| IMU 也在 124、各回 12 B | 无 IMU；IMU 走 `/dev/i2c-imu` |
| 慢速传感器 144–146 单独事务 | `slow_sensors()` 直接返回本 tick 缓存块 |
| 读写 `0x2A` 目标位置 | 同 `0x2A`（位置伺服 mode 0） |
| 校验和 Dynamixel v2 | FT-HLS `~(ID+LEN+INST+PARAM)&0xFF` |

## 3. 自己动手【无鸭可跑】

```bash
cd ~/microduck
# 姿态解码：
cargo test -p duck-control half_precision_decodes_known_values
cargo test -p duck-control all_zero_quaternion_bytes_hold_the_last_good_value
cargo test -p duck-control not_ready_until_the_chip_has_produced_output
cargo test -p duck-control gravity_is_a_unit_vector_in_steady_state
# 总线块换算（无串口也能测——换算函数与 rustypot 字节互相印证）：
cargo test -p duck-control position_conversion_round_trips_through_rustypot
cargo test -p duck-control velocity_scale_matches_the_datasheet_figure
cargo test -p duck-control the_first_block_is_never_stale
```

> `bus.rs` 单测为什么能无串口跑？因为 `RobotIo` 是 trait，测试直接对换算函数/字节块做断言；
> 真串口行为只在 `DynamixelIo`（`bus.rs` 的真实现）里。这就是第 1 讲说的"纯逻辑心脏"。

## 4. 收尾检查

- [ ] 旧 XL330 时代 IMU 板为什么能和舵机同包读？（答：Dynamixel `sync_read` 同地址同长度多 id；两者都服务 124 地址 12 字节。**迁移后已拆开**：IMU 走 `/dev/i2c-imu`，舵机走 FT-HLS。）
- [ ] 一个全 0 的 quat 块意味着什么？解码器会怎么做？（答：SFLP 还没写表；保留上次好值，绝不回落 identity）
- [ ] `gravity` 怎么来的？（答：把世界系 `[0,0,-1]` 用 `last_quat` 反旋回躯干系并单位化）
- [ ] 迁移后 IMU 读失败会怎样？（答：只降级——保留上次 `ImuData`、错误计数、按 1/10 记日志，整拍不 fail；`imu.rs` 零改动）
- [ ] 换了非 SFLP 的 IMU，第几、几步要在主机重做？（答：第 2/4 步——出 quat 与估 gyro bias 的融合）

**下一步**：第 4 讲看 gravity/gyro 怎么被放进 61 维观测，以及策略动作怎么缩回 15 个关节目标。

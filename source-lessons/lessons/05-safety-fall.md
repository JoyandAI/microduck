# 05 · 安全网：摔倒预测与 limp-fall（safety.rs + fall.rs）

> 目的：看懂"读到的数字"如何变成"松力/保持/断电"的裁决。安全逻辑在 `duck-control` 里分两层：
> `Safety`（慢裁决：是不是已经倒了）与 `FallPredictor`（快预测：正在往下倒吗）。
> 这两层都吃 IMU 的 gravity/gyro——所以 IMU 怎么接、多新鲜，直接决定这层网的效果。

## 1. Safety：一个"门卫"对象

`safety.rs` 的结构一句话：`Safety<T: RobotIo>` 是**唯一握着 `RobotIo` 句柄**的门卫——
主循环不直接碰总线，而是 `safety.read()` 读、`safety.observe()` 喂、`safety.gate()`/`apply()`
放行目标。doc 注释（`safety.rs:17` 附近）点明它的三件事：**电压/温度/摔倒 → 限幅或松力，
外加对指令本身的 deadman**。

### 1.1 配置与"先讲清它不保护什么"

```rust
pub const ACTUATOR_MIN: f64 = -PI;
pub const ACTUATOR_MAX: f64 =  PI;
```

注意这条注释的诚实：

> *This is the **actuator's** travel, not a per-joint anatomical limit … it catches a policy
> emitting `NaN`, an absurd action scale, or a garbage tensor; it will not stop a joint being
> driven somewhere mechanically unwise.*

`SafetyConfig` 默认值（`safety.rs:51`）就是第 7 讲 `robotd.toml` 里 `[safety]` 的默认出处：

| 字段 | 默认 | 含义 |
|---|---|---|
| `fall_gravity_z` | `-0.5` | 投影 gravity 的 z 高于它就"算倒"（直立≈−1.0，侧躺≈0，≈60° 处触发） |
| `fall_debounce` | `200 ms` | 要持续这么久才算数（硬跺脚不是摔） |
| `deadman` | `500 ms` | 指令超过这么久没更新，速度清零 |
| `gain_running` | `200` | 运行增益 |
| `gain_limp` | `50` | 松力增益：低到不再和地板较劲的那个值 |

### 1.2 裁决怎么下来

- `read()`（`safety.rs:127`）透传给 `io.read()`——主循环第一件事就是它（第 6 讲）。
- `observe()`（`safety.rs:181`）累计"gravity 越过阈值多久"：

  ```rust
  let down = sensors.imu.gravity[2] > self.config.fall_gravity_z;
  // falling_for += dt（down 时）；任何直立样本都清零
  ```

  `fall_debounce` 到点 → `fallen = true`。单测 `a_brief_tilt_is_not_a_fall` 就是钉这个
  （一瞬的倾斜不算）。
- `gate()`（`safety.rs:218`）上 deadman：`intent_age > deadman` → 速度归零并报 `Limit::Deadman`。
  注释/测试强调：**deadman 只清 twist，别的不动**（`the_deadman_zeroes_the_twist_only`）。
- `apply()` 对每个目标做范围/有限性检查，命中就报 `Limit::Range` / `Limit::NotFinite`
  （枚举在 `safety.rs` 里，见 `Limit`）。

`imu_ready()`（`safety.rs:145`）透传给 bus 的 `SflpDecoder::ready()`（第 3 讲的 ≥25 拍）——
**IMU 还没收敛前，摔倒裁决与里程计播种都不该运行**。

## 2. FallPredictor：专门用来"提前半拍"的检测器

`fall.rs:1` 模块注释把为什么要第二层讲得极清楚：

> [`Safety`] has a fall verdict already … projected gravity past `fall_gravity_z` (about 60°
> from upright) held for `fall_debounce` (200 ms). **By the time that latches the robot is on
> the floor, or a few tens of milliseconds from it.** That is fine for "refuse to enable a
> robot lying on its side" and useless for "drop the gains before it lands" — the whole value
> of **limping is spent in the window this predictor exists to find.**

`Safety` 回答"已经倒了吗"（用于拒绝在侧躺时使能）；`FallPredictor` 回答"**正在往下倒吗**"
（用于落地前把增益降到 `gain_limp`，让鸭子软着陆）。要"提前"，就得看**变化率**而不是位置。

### 2.1 数学：一行叉积

躯干系里 gravity 随躯干转，所以它的导数恰好是 `ġ = −ω × g`，只取 z 分量就是一次乘减：

```rust
pub fn gravity_z_rate(imu: &ImuData) -> f64 {
    -(imu.gyro[0] * imu.gravity[1] - imu.gyro[1] * imu.gravity[0])
}
```

> *No filtering, no differentiation of the quaternion: the gyro is a direct measurement
> arriving in the **same 12-byte block**.* —— 不去微分 SFLP quat，因为那样会把滤波器自己的
> 滞后加进"本该最早"的那个数里。

### 2.2 三个门槛 + 一小段防抖（全部满足才触发）

| 门槛 | 默认 | 为什么 |
|---|---|---|
| 已经倾斜过 `tilt_z` | `-0.90`（≈26°） | 直立时的 gyro 尖峰是跺脚/被推/被抱起，都不是摔；没有位置门槛会把三者全报成摔 |
| 还在继续倒（`ġz > 0`） | — | 在往下倒，而不是从倾斜里恢复 |
| 外推到 `lookahead` 后超过 `predicted_z` | `predicted_z=-0.5`、`lookahead=300 ms` | 问的是"**0.3 秒后 gravity 在哪**"，过了不归点才算 |

然后防抖 60 ms（50 Hz 三拍）——**一个噪点不能让鸭子在中途失去增益**。

`FallPredictorConfig` 完整默认（`fall.rs:70` 附近）就是上面那行的出处：
`tilt_z = -0.90`、`predicted_z = -0.5`（与 `SafetyConfig.fall_gravity_z` 同号同值）、
`lookahead = 300 ms`、`debounce = 60 ms`。另外 `observe()` 是**边沿触发**：防抖到点那拍返回一次
`true`，之后保持 `false` 直到机器人不再像在倒——limp 该做多久、做完做什么由调用方（robotd）决定。

注释把调参哲学写成一句话（这是理解"为什么默认偏晚"的钥匙）：

> *Too early and the robot limps out of leans it would have walked off — every false positive
> is a fall the robot **caused**, which is strictly worse than the fall being dampened.*

**宁晚勿早**：一次误触发 = 鸭子自己把自己吓倒，比摔倒本身还糟。所以 `tilt_z=-0.90` 平时走路
到不了，60 ms 防抖长于任何跺脚冲量、又短到能把大部分坠落留给 limp 去缓冲。

### 2.3 触发后：limp-fall 序列（第 6 讲在主循环里见）

`robotd` 在 `main.rs` 里看到 `falling.observe()` 返回真，就进入 `limp_fall` 状态机：
把增益降到 `gain_limp`、让鸭子软着陆、落地后再慢慢回到站立姿势——**把"起立"这件策略最擅长的
事还给它**（`main.rs` 那段注释："The standing policy is a good stand-up-er and a bad faller"）。

## 3. 对改造的含义（结合你的 IMU 直连）

这两层都假设：**gyro 与 gravity 在同一份、够新鲜、够同拍的 `ImuData` 里到达**。

- `gravity_z_rate` 用 `gyro × gravity` 的叉积当"倒下速度"。若直连 IMU 在**同拍内**、只比舵机
  晚几十 µs 到一两 ms 到达，这个语义基本不变；
- 若新 IMU **刷新率比 50 Hz 低**、或你改读 raw accel/gyro 自己在主机融合（滞后 + 滤波），
  这层"靠变化率提前半拍"的网会退化——`ready()` 的 25 拍门槛、`fall_debounce`、limp 窗口
  全都以"IMU 足够新鲜"为前提。这是评估"不上伺服总线"时最容易低估的一环。

## 4. 自己动手【无鸭可跑】

```bash
cd ~/microduck
# Safety 裁决：
cargo test -p duck-control a_brief_tilt_is_not_a_fall
cargo test -p duck-control the_deadman_zeroes_the_twist_only
# FallPredictor：
cargo test -p duck-control the_rate_is_the_analytic_derivative   # 叉积就是解析导数
cargo test -p duck-control a_footfall_on_an_upright_robot_never_fires
cargo test -p duck-control recovering_from_a_lean_never_fires
cargo test -p duck-control a_fall_fires_once_after_the_debounce
```

读 `fall.rs` 里 `tipping()` / `leaning()` 那几个测试夹具——它们把"直立被推 / 从倾斜恢复 /
真要倒"三种情况的 gravity 形状摆出来，比注释更直观。

## 5. 收尾检查

- [ ] `Safety` 与 `FallPredictor` 各答什么问题？（答：已倒吗 vs 正在倒吗；前者慢而稳，后者快而早）
- [ ] 为什么预测用叉积 `gyro×gravity` 而不是去微分 quat？（答：gyro 是同一块里的直接测量，微分 quat 会引入滤波滞后）
- [ ] 误触发为什么"比摔倒本身更糟"？（答：鸭子会自己把自己吓倒——它造成的摔，比被缓冲的摔严重）

**下一步**：第 6 讲进 `robotd/src/main.rs`，看这一切怎么被 50 Hz 的 tick 串起来。

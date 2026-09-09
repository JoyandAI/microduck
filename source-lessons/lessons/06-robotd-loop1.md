# 06 · robotd：50 Hz 主循环骨架（main.rs）

> 目的：前 5 讲在 `duck-control` 里把"纯逻辑链"讲完了——读总线、拼观测、跑策略、过 safety。
> 但那条链自己不会跑：是 `robotd/src/main.rs` 里一个 50 Hz 的 tick 在驱动它。
> 本讲把 `main.rs` 里"一拍"的完整路径走一遍：ticker 从哪来、读失败怎么办、
> intent 与 safety 怎么插进来、torque 什么时候才打开。第 7 讲再进 controller 内部。

## 1. 50 Hz 从哪来：period 与 ticker

`main.rs` 顶部模块注释（`repo:robotd/src/main.rs:1`）一句话交代这个进程的三块：

> *`robotd` — the robot control daemon. The 50 Hz control loop, the `robot.*` socket, and
> the health the updater gates on.*

控制循环跑在**自己的 OS 线程 + 自己的 current-thread tokio runtime** 上
（`repo:robotd/src/main.rs:785` 起，`spawn_control_thread`）。注释给了理由：
总线读是**阻塞串口 I/O**，放在共享 runtime 上会占住 worker；给它独享 runtime，
IPC 的活儿就永远排不到一拍前面。这是整个"循环从不被客户端卡住"的地基。

### 1.1 period 的来处

`spawn_control_thread` 开头一行 `let period = params.period();`（`repo:robotd/src/main.rs:798`）。
`period()` 实现在 `robotd-params`：

```rust
pub fn period(&self) -> std::time::Duration {
    std::time::Duration::from_secs_f64(1.0 / self.control.hz as f64)
}
```

（`repo:robotd-params/src/lib.rs:995`）。`control.hz` 默认 `50`（`repo:robotd-params/src/lib.rs:856`），
所以一拍 **20 ms**。部署文件里注释得很直白：50 Hz 是从树莓派 Zero 2W 时代继承的，
在 Radxa 上还没重新测量过（`repo:deploy/robotd.toml:30`）。第 7 讲细讲 params 落点。

### 1.2 ticker：`interval(period)` + `MissedTickBehavior::Skip`

```rust
let mut ticker = tokio::time::interval(period);
ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
```

（`repo:robotd/src/main.rs:1180`、`repo:robotd/src/main.rs:1195`）。`Skip` 的选择是讲稿级的细节，
代码注释（`repo:robotd/src/main.rs:1181`）给了测量依据：

- `Burst` 把积压补发、马达指令叠在一起——显然错；
- `Delay` 每拍都按 `now + period` 重排下一次，调度抖动全被加进周期——**永久性掉速**。
  注释记录：用 `Delay` 实测 43.1 Hz 对 50 Hz 目标、`missed = 0`，看起来像硬件问题；
- `Skip` 保持原节奏、丢弃错过的拍——控制循环要的就是这个：不积压、不漂移。

## 2. 两个 io：`--fake` 与真总线

一拍要读要写，但 io 有两个实现。`spawn_control_thread` 里按 `args.fake` 分叉
（`repo:robotd/src/main.rs:817`）：

```rust
if fake {
    tracing::warn!("--fake: no bus, no robot");
    runtime.block_on(control_loop(
        FakeIo::at(DEFAULT_POSITION),
        state, intents, params, period, poweroff,
    ));
    return;
}
```

**fake 时没有总线**：不给 `robotd` 开任何串口，`FakeIo::at(DEFAULT_POSITION)` 直接当 io
（`repo:robotd/src/main.rs:820`）——每拍读回的都是"站在 home pose"的一份样本。
`FakeIo` 的定义注释（`repo:duck-control/src/io.rs:161`）说得清楚：*a robot made of nothing*，
读回上次写入的位置，所以驱动它的循环表现得像"舵机完美跟手"。`--fake` 是设计文档钦定的
笔记本路径，而且**必须显式要求**，不会自动降级（design §1.2 那句 *must be asked for explicitly*）。

真机路径在 `open_bus_waiting` → `open_bus` 里（`repo:robotd/src/main.rs:857`）：

```rust
let mut io = match duck_control::bus::DynamixelIo::open(port) { ... };
```

（`repo:robotd/src/main.rs:887`）。打开后还做 `check_registers()`——把舵机 EEPROM 里
`return_delay_time/baud/pwm_slope/shutdown` 校正到出厂期望值。`run_init`（`robotd init`
子命令）也直接 `DynamixelIo::open(&params.bus.port)`（`repo:robotd/src/main.rs:743`）。
类型层面用 `BusIo` 别名统一两种平台（`repo:robotd/src/main.rs:844`）：
Linux 上是 `DynamixelIo`，非 Linux 上是 `FakeIo`（所以 Mac 能编译整仓、但没有 `--fake`
就只能得到一句 *no bus on this platform; use --fake*，`repo:robotd/src/main.rs:915`）。

## 3. 一拍：主 while 循环逐段读

`control_loop` 的真正主体是

```rust
while !state.shutdown.load(Ordering::Relaxed) {
    ticker.tick().await;
    let tick_start = Instant::now();
    ...
}
```

（`repo:robotd/src/main.rs:1339`）。函数头的文档注释先把一拍画成一行：
`read → observe (fall) → gate (deadman) → policy → safety.apply`
（`repo:robotd/src/main.rs:1037`）。按代码顺序拆开：

| # | 做什么 | 代码 | 一句话 |
|---|---|---|---|
| 1 | 等一拍、记 `tick_start` | `repo:robotd/src/main.rs:1340` | 所有超时都以 `tick_start` 计时，不累加漂移 |
| 2 | `safety.read()` 拿 `fresh` | `repo:robotd/src/main.rs:1343` | 通过 `Safety` 读——它握着唯一的 `RobotIo` |
| 3 | 失败计数 | `repo:robotd/src/main.rs:1349` | `consecutive_errors` 第 1、第 10…次才 warn（下节细讲） |
| 4 | `coast.sample(fresh)` | `repo:robotd/src/main.rs:1365` | 掉读时用上一份好样本续命 ≤3 拍 |
| 5 | 只在 `fresh` 上 `safety.observe` | `repo:robotd/src/main.rs:1367` | 重放的样本不能喂摔倒防抖 |
| 6 | `imu_ready()` 才 `odometry.update` | `repo:robotd/src/main.rs:1373` | IMU 未收敛/样本是重放，都不播种里程计 |
| 7 | `intents.snapshot()` + `safety.gate` | `repo:robotd/src/main.rs:1379` | 拿 intent、上 deadman（05 讲的 `deadman_ms`） |
| 8 | 一次性请求：init/relax/skills/mode/shutdown | `repo:robotd/src/main.rs:1388`–`1623` | "取一次"而非"电平"，见 §5 |
| 9 | limp-fall 观测与状态机 | `repo:robotd/src/main.rs:1638`–`1743` | 05 讲的第二层网在这里挂 |
| 10 | 指令 EMA 平滑 | `repo:robotd/src/main.rs:1746` | 摇杆阶跃变斜坡；limp-fall 期间 twist 强行置零 |
| 11 | bringup：enable → torque → Homing | `repo:robotd/src/main.rs:1795` | 只有这里才开 torque（§5） |
| 12 | 算 `driving` | `repo:robotd/src/main.rs:1881` | 见下，七条闸门缺一不可 |
| 13 | 目标来源分派 | `repo:robotd/src/main.rs:1936` | limp-fall > controller.step > homing > hold |
| 14 | `safety.apply(targets, hold, gain)` | `repo:robotd/src/main.rs:2180` | 范围/非有限检查 + 真正 `sync_write` |
| 15 | 发布状态帧 | `repo:robotd/src/main.rs:2188` | 只有订阅者存在才拼帧（避免每拍分配） |
| 16 | 记 tick/missed/achieved_hz | `repo:robotd/src/main.rs:2224`–`2241` | 健康门卫的原料；每秒顺带读慢传感器 |

### 3.1 `fresh` 与"策略步进的样本"是两个概念

第 2/4 步的区别很关键，注释（`repo:robotd/src/main.rs:1360`）点了两处后果：

- **safety 只 observe fresh 样本**——重放同一份读数三遍，等于让防抖把一次坏 tick 数三次；
- **`known_positions` 只随 fresh 前进**——coast 的一拍不能假装知道关节已经走到哪了。

`driving` 的第 12 步是整拍的总闸（`repo:robotd/src/main.rs:1881`）：

```rust
let driving = snapshot.enabled
    && bringup == Bringup::Ready
    && controller.is_some()
    && !in_limp_fall
    && sensors.is_some()
    && imu_warm
    && !powered_off;
```

每个条件都承重：`sensors.is_some()` 保证"读失败的那拍没有观测可拼，不拿旧数硬造一只不存在的鸭子"；
`!in_limp_fall` 是唯一一个"不是拒绝"的条件——limp-fall 序列期间策略**故意**不驱动；
`imu_warm` 挡掉"IMU 还没收敛就开步"的 *crazy start*。

### 3.2 目标来源：谁有资格说话

`driving` 之后是一大段 `match (driving, sensors.as_ref())`（`repo:robotd/src/main.rs:1936`），
决定这一拍的 15 个目标 `targets`、增益 `gain`、是否 `moving`：

| 条件 | 目标 | 增益 | 说明 |
|---|---|---|---|
| `in_limp_fall` | 序列自带的 target / pose 斜坡 | `gain_limp` / `limp_fall_pose_gain` | `driving=false`，若没有这一支会掉进 hold、以行走增益命令摔倒前的姿势——正是本模式要禁的 |
| `driving && Some(sensors)` | `controller.step(...)` | 策略增益 | **第 7 讲的主角**，这里只留入口（`repo:robotd/src/main.rs:1971`） |
| bringup 仍在 Homing | `bringup.homing_target(...)` | 策略增益 | 线性斜坡，逐拍插值而非阻塞 |
| 其余（hold） | `hold` | 策略增益 | 没使能/读不到/IMU 冷：就地站住 |

第 14 步 `safety.apply` 是唯一的写出口（`repo:robotd/src/main.rs:2180`）——`Safety` 握着唯一
`RobotIo`，策略、controller、客户端都只能"提议"目标，谁也绕不开范围钳位与非有限拒绝。

## 4. 掉一拍怎么办：COAST_TICKS、连续错误与自愈

### 4.1 "一次掉读是寻常事"，所以必须隐形

`COAST_TICKS = 3` 的文档注释（`repo:robotd/src/main.rs:105`）讲了一个真实 bug 史：
**一次失败的 Dynamixel 事务很正常**——bench 实测约每分钟 8 次——但旧实现会让策略停一拍、
去命令 hold 姿势、还把 controller 重置了，于是每次掉读都变成一次可见的抽搐
（"random tiny spasms"）。3 拍 = 60 ms 覆盖"一次掉读 + 重试 + 一次慢拍"。

`Coast::sample`（`repo:robotd/src/main.rs:2282` 起）把这个语义做成一格：

```rust
Some(sensors) => { self.ticks = 0; self.last = Some(sensors); Some(sensors) }
None if self.ticks < COAST_TICKS => { self.ticks += 1; self.last }
None => None,
```

读到好样本就清零；连续掉读的前 3 拍吐回最后一份好样本，第 4 拍起诚实地说 `None`。
配套的 `known_positions`（`repo:robotd/src/main.rs:2319`）在需要"关节现在大概在哪"的场合
（hold、limp-fall 起坡）给最后读到的那组位置。**coast 之所以安全**，是因为观测本来就
"旧一拍"：50 Hz 下策略训练用的就是这种年龄的数据，多一拍仍在训练分布内。

### 4.2 连续错误：第 1 次和第 10 次才出声

```rust
let n = state.consecutive_errors.fetch_add(1, Ordering::Relaxed) + 1;
if n == 1 || n.is_multiple_of(10) {
    tracing::warn!(error = %e, consecutive = n, "bus read failed");
}
```

（`repo:robotd/src/main.rs:1349`）。掉读那拍**照常跑完**（coast 或 hold），日志只在第 1 次
与每第 10 次出现——持久故障可见，又不刷墙。`consecutive_errors` 也是健康门卫的输入：
超过 `update_gate.max_consecutive_errors`（默认 10）就报不健康（`repo:robotd/src/main.rs:545`）。

### 4.3 自愈：RESET_AFTER_PAUSE 决定要不要重置 controller

重新开始驱动时，controller 要不要 `reset()`（清空策略观测到的 action 历史、丢掉低通锚点）？
重置本身也是个不连续。`reset_on_resume`（`repo:robotd/src/main.rs:2331`）用 `RESET_AFTER_PAUSE`
（默认 **200 ms**，`repo:robotd/src/main.rs:125`）分界：

- 暂停 < 200 ms（比如一次 3–5 拍的掉读）：**不重置**——不然会把刚想消除的抖动又引回来；
- 真停过 200 ms 以上：重置，因为机器人可能已在别处。

注意两个常数配合：`COAST_TICKS`(60 ms) < `RESET_AFTER_PAUSE`(200 ms)，所以短暂掉读既被
coast 吸收、又够不到重置线；超过 200 ms 的停顿才当作"真的换了个世界"。

## 5. bringup 状态机：torque 何时开

`Safety::set_torque` 的 trait 注释是这个 daemon 最重要的一条不变量
（`repo:duck-control/src/io.rs:132`）：

> ***Nothing calls this at startup, and that part has not changed.** A `robotd` restarted by an
> update must leave a standing robot standing: torque comes on when someone enables the policy,
> never because a process began.*

update 重启、systemd 拉起、进程崩溃重启——都不该让鸭子动一下。舵机在进程死后仍保持
最后一条目标位置指令，所以 `robotd` 起来后 `adopt_startup_pose`（`repo:robotd/src/main.rs:995`）
**读回当前位置当作 `hold`，不开 torque**。老设计里"开力"藏在 `robotd init` 子命令里
（要停 daemon、自己开总线、还没文档），于是按 Start 什么都不会发生。现在 bringup 是循环里
的一个状态机（`repo:robotd/src/main.rs:959`）：

```
Limp ──enable（策略已加载 + 有新鲜样本）──▶ Homing（torque 开，2 s 斜坡）──▶ Ready ──▶ 策略驱动
```

`set_torque`/`set_gain` 是**每关节一笔总线事务**，所以 bringup 不能是"每拍都去 apply 的
电平"，必须是"取一次请求、推进状态机"。第 11 步（`repo:robotd/src/main.rs:1795`）的条件很关键：

- 从 `Limp` 出发；
- `snapshot.enabled`（有人按了 Start / 发了 `robot.enable`）；
- `controller.is_some()`（"enable 策略"得真有策略可 enable——否则会把坏 release 的鸭子
  立起来然后傻站着）；
- `Some(sensors)`（斜坡从"关节实际在哪"起步，没人读过的位置起步就是那个要避免的 lurch）。

`robot.init` / `robot.relax` 是同一个状态机的另外两个入口（`repo:robotd/src/main.rs:1388`）：
`init` 不需要策略（"站起来"对没有行走网络的鸭子也合理），`relax` 清 `enabled` 并把状态机
退回 `Limp`。挂在主循环上的收尾状态机同理，本讲只认挂点：

| 挂点 | 位置 | 一句话 |
|---|---|---|
| limp-fall（Limp→Posing→交还） | `repo:robotd/src/main.rs:1638` | 用 `was_driving` 而非 `enabled` 判定资格，防"倒地起立摇一下又触发" |
| shutdown-sit（坐下→断电） | `repo:robotd/src/main.rs:1580` | 电池到空（EMA）或 `robot.shutdown`：能坐就坐 4 s 再断 torque |
| mode 切换（先回 home 再换策略包） | `repo:robotd/src/main.rs:1538` | 换网络不能发生在步态中途 |
| 技能一次性请求 | `repo:robotd/src/main.rs:1447` | kick/roulade/ground-pick/sit，需要 `Ready` + `enabled` |

## 6. 对改造的含义：你在 duck-control 改的，正是这里被驱动的

这讲把前 5 讲的"纯逻辑"和进程缝在了一起：`duck-control` 不碰 tokio、不碰 socket，
所以那一侧的东西（`Sensors` 布局、obs、safety、FakeIo）可以脱离 daemon 单测
（01 讲 `cargo test -p duck-control` 无鸭可跑）。改飞特换装/IMU 直连时，先分清改在哪一侧：

- 改**读什么**（总线协议、寄存器、IMU 融合）→ `duck-control`，这里不用动；
- 改**什么频率、谁先谁后、失败怎么办**（coast 拍数、`RESET_AFTER_PAUSE`、bringup 门槛、
  把新外设挂进第 8/9/16 步而不许它阻塞一拍）→ `main.rs`，本讲的骨架就是下刀点。

## 7. 自己动手【无鸭可跑】

> 本机还没有 Rust 工具链，先装（见 00 讲）。`robotd` 是较重路径：整仓编译 + `ort`
> 的 onnxruntime 依赖。`--no-policy` 让循环"只持姿势不加载策略"，
> 绕开笔记本上没有 ONNX Runtime / `.onnx` 的问题（design §2.3）。

```bash
cd ~/microduck
# 假鸭 50 Hz 在电脑上跑；--socket 避开 /run/robotd.sock 需要 root
cargo run -p robotd -- --fake --no-policy --socket /tmp/robotd.sock
```

验证点：

1. 日志出现 `--fake: no bus, no robot`（`repo:robotd/src/main.rs:818`），随后
   `control loop running joints=15 hz=50 driving=false`（`repo:robotd/src/main.rs:1173`）——
   driving 为 false 是预期：`--no-policy` 且没人 enable；
2. 进程不崩、不退出，5 分钟级的 `control loop` 摘要行说明循环还活着；
3. Ctrl+C 后打印 `control loop stopped` 干净退出。

想再进一步看循环内部，把 `RUST_LOG` 调到 `robotd=debug`（默认是 `info`）；若要真的发
`robot.enable` / `robotctl monitor`，那需要跑起第二个能说话的角色——先各自 `--help`
看 flag，本讲不替你编命令。**要真鸭子/真 socket 的验证（在板上 `robotctl health`、
看舵机真的跟上 50 Hz）都标【需真机】**，不在本讲范围。

## 8. 收尾检查

- [ ] `period` 由谁定？（答：`params.period()` = 1/`control.hz`，默认 50 → 20 ms；ticker 用 `MissedTickBehavior::Skip`）
- [ ] 为什么是 `Skip` 而不是 `Delay`？（答：`Delay` 把每次唤醒延迟加进周期 → 永久掉速，实测 43.1 Hz）
- [ ] `--fake` 与真总线分别走哪条路？（答：fake 分支直接 `FakeIo::at(DEFAULT_POSITION)`、没有总线；真机 `DynamixelIo::open` + `check_registers`，且开总线是"等待式"而非一次失败就放弃）
- [ ] 读失败的那拍还跑吗？（答：跑。`coast.sample` 用最后好样本续 ≤3 拍，第 1/第 10 次才 warn，第 4 拍起 `sensors=None` → `driving=false` → hold）
- [ ] 为什么 safety.observe 与 odometry.update 只在 `fresh` 上？（答：重放样本不能喂防抖/不能让里程计以为机器人冻住了）
- [ ] torque 什么时候开？（答：bringup 从 `Limp` 经 enable 进 `Homing` 时；`io.rs` 那句 *never because a process began*——update 重启必须让站着的鸭子继续站着）
- [ ] `driving` 的七条闸门能背出几条？（答：enabled、`Bringup::Ready`、controller 在、非 limp-fall、本拍有样本、IMU 收敛、未断电）

**下一步**：第 7 讲进 `controller.step()` 与 params——看一拍里 `duck-control` 的策略链
（roulade > kick > … > stand > walk）到底怎么被调用、`robotd.toml` 每个数字从哪生效。

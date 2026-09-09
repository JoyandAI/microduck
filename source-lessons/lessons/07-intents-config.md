# 07 · 意图与配置：intents → control → robotd.toml 参数落点

> 目的：回答两件事。(1) 从"玩家推摇杆 / 敲 robotctl"到第 4 讲那个 61 维 obs 里的 command 块之间
> 到底发生了什么；(2) `robotd.toml` 里那些数（`gain` / `action_scale` / 低通 / `standing_gain_ratio` /
> `SafetyConfig` 一堆）在 Rust 侧从哪来、改一个数如何生效。前一半在 `intents.rs`，后一半在
> `robotd-params`。第 6 讲已把主循环骨架铺好，本讲只拆它的**两头**——意图来源与参数落点。

## 1. intents.rs：主循环从不问"值是多少"，只问"多新鲜"

`robotd/src/intents.rs:3` 的模块注释把设计一句话讲完：

> *Written by IPC tasks, read once per tick by the control loop. The loop must never wait on a
> client, so each slot is an [`ArcSwap`]: the reader does one atomic load and the writer one
> atomic store, and neither can hold up the other.*
> **Twist and head are separate slots on purpose.** … Every slot is stamped, because the
> loop's real question is never "what is the value" but "how old is it". That is what the
> deadman reads.

关键词三个：**ArcSwap 槽**（读方一次原子 load，写方一次原子 store，互相不阻塞）、**分槽**（twist 与
head 分开，各自单写者）、**盖章**（每个值带到达时间）。最后一条是本讲第一节的主角——它不是"值
是什么"的容器，是"这值有多旧"的容器。

### 1.1 一个带时间戳的槽

每个槽就是一个 `Stamped<T>`（`robotd/src/intents.rs:44`）：

```rust
struct Stamped<T> {
    value: T,
    /// Microseconds since the [`Intents`] epoch.
    at_us: u64,      // 相对进程启动那一刻的微秒数
}
```

`Intents` 里 twist 与 head 各占一个 `ArcSwap<Stamped<...>>`（`intents.rs:122`），pose/mouth/skills/
power 等是别种形态，见下面的槽表。写方 `set_twist`（`intents.rs:242`）/ `set_head`（`intents.rs:249`）
每次写都带 `self.now_us()` 盖一个新戳：

```rust
pub fn set_twist(&self, twist: [f64; 3]) {
    self.twist.store(Arc::new(Stamped { value: twist, at_us: self.now_us() }));
}
```

读方只有 `snapshot()`（`intents.rs:467`）一个入口，主循环每拍调用一次，产出 `Snapshot`
（`intents.rs:184`）。它做三件事：把 twist/head 值组装成 `Command`、把 body 钉回名义零、
算 `twist_age`：

```rust
pub fn snapshot(&self) -> Snapshot {
    let now = self.now_us();
    let twist = self.twist.load();
    let head = self.head.load();
    Snapshot {
        command: Command {
            twist: twist.value,
            head: head.value,
            // 姿态平滑是主循环的逐拍状态，不属于意图槽；
            // 名义零是训练编码，不是占位。
            body: BodyPose::default(),
        },
        twist_age: Duration::from_micros(now.saturating_sub(twist.at_us)),
        enabled: self.enabled.load(Ordering::Relaxed),
        pose,                      // body-pose 意图原样带上，交给主循环平滑
        mouth: f64::from_bits(self.mouth.load(Ordering::Relaxed)),
    }
}
```

注意 body 在 `snapshot()` 里永远是 `BodyPose::default()`（全零）——姿态意图不在这一层塞进
command，而是走 `pose` 字段让主循环用 `cmd_alpha` 平滑（§2）。这是 04 讲"body 全零是标称编码"
在代码里的确切位置。

### 1.2 起点就要"足够旧"：twist 从 at_us = 0 开始

`Intents::new()`（`intents.rs:204`）把初始 twist 盖成 `at_us: 0`，注释说得很白：

> Stamped at zero, so before any client connects the twist already reads as maximally stale
> and the deadman holds the robot still. Starting "fresh" would mean a robot that briefly
> believes it has a live driver.

单测 `the_twist_starts_stale`（`intents.rs:530`）钉的正是这条：`Intents` 一建出来，`twist_age` 就从
epoch 起算，`enabled == false`。所以**没有客户端 = 摇杆没推 = deadman 生效 = 站住**，三件事是同一个
机制，不是三条约定。

### 1.3 槽表

| 意图 | 写方法（IPC/客户端调） | 读/取方法（主循环每拍） | 形态 | 盖不盖时间戳 |
|---|---|---|---|---|
| 速度 twist | `set_twist` | `snapshot().command.twist` + `twist_age` | `ArcSwap<Stamped<[f64;3]>>` | 盖，deadman 只读它 |
| 头 head | `set_head` | `snapshot().command.head` | `ArcSwap<Stamped<[f64;4]>>` | 盖，但 stale 无害 |
| 姿态 pose | `set_pose` | `snapshot().pose` | `ArcSwap<PoseIntent>` | 不盖，`active:false` 即"没人在摆" |
| 嘴 mouth | `set_mouth` | `snapshot().mouth` | `AtomicU64`（f64 的 bits） | 不盖（开着就开着） |
| 技能 skills | `request_skill` | `take_skills()` | `AtomicU32` 位掩码，取走清零 | 事件，不是电平 |
| 上电 init/relax | `request_init` / `request_relax` | `take_power_request()` | `AtomicU8`，取走清零 | 事件（一拍只做一次） |
| 使能 enabled | `set_enabled` | `snapshot().enabled` | `AtomicBool` | 离散电平 |

技能用位掩码而非队列的原因（`intents.rs:75` 附近）：20 ms 一拍内同键第二下没有额外意义，但两个
**不同**键都要被看见——单个 last-writer 槽会丢一个。`roulade` 例外：客户端每拍重发表示"还按着"，
所以它是**电平**，还有自己的 `WHEEE_HOLD_FRESH` 式新鲜度（`intents.rs:101`，roulade 用
`ROULADE_CHAIN_WINDOW`，在 control.rs 见）。

### 1.4 这些槽是谁写的（一句话，细节留给第 8 讲）

IPC 的 JSON-RPC handler 落到 `apply_intent`（`robotd/src/main.rs:2631`）：`robot.move` →
`set_twist([p.vx, p.vy, p.vyaw])`，`robot.head` → `set_head([neck_pitch, head_pitch, head_yaw,
head_roll])`，`robot.pose` → `set_pose`，`robot.do` → `request_skill`。手柄 `padd` 只是这些
RPC 的一个客户端——设计文档 `docs/design/robotd-design.md` §4.3 原话：*the gamepad is a client*。
socket 里怎么传、JSON-RPC 怎么分帧是第 8 讲；本讲只需知道**一切命令都先落进上面这些槽**。

## 2. 到 command 块之前：deadman 把关 + 主循环平滑

主循环拿到 `Snapshot` 后，先过第 5 讲的安全层 deadman（`robotd/src/main.rs:1379`）：

```rust
let snapshot = intents.snapshot();
let (gated, deadman) = safety.gate(snapshot.command, snapshot.twist_age);
let mut limits: Vec<duck_control::safety::Limit> = deadman.into_iter().collect();
```

`gate()`（`duck-control/src/safety.rs:218`）就是**意图新鲜度与 deadman 的关系**的全部：

```rust
pub fn gate(&self, command: Command, intent_age: Duration) -> (Command, Option<Limit>) {
    if intent_age <= self.config.deadman {
        return (command, None);
    }
    let mut stopped = command;
    stopped.twist = [0.0; 3];
    (stopped, Some(Limit::Deadman))
}
```

`deadman` 的时长来自 `SafetyConfig.deadman`，源头是 `[safety] deadman_ms`（§4）。注意它**只清
twist**——`Snapshot.twist_age` 只跟 twist 槽的戳比，head 的 stale 无害（一条注释：*a stale head pose
is harmless; a stale velocity walks the robot into a wall*）。

gate 之后是主循环的**平滑层**（`robotd/src/main.rs:1746` 起），这是"摇杆猛推"变成"策略跟得上的
斜坡"的一步，也是 `[control]` 的两个参数落点：

```rust
for (ema, target) in twist_ema.iter_mut().zip(twist_target) {
    *ema += cmd_alpha * (target - *ema);        // cmd_alpha 平滑 vx/vy/vyaw
}
for (ema, target) in head_ema.iter_mut().zip(gated.head) {
    *ema += head_alpha * (target - *ema);       // head_alpha 平滑头四轴
}
// body：pose.active 时向 snapshot.pose.body 平滑；否则直接回名义零
let command = PolicyCommand {                   // PolicyCommand = obs::Command
    twist: twist_ema,
    head: [...],                                 // 还要叠 chorale 摆头
    body: BodyPose { z: body_ema[0], roll: body_ema[1], pitch: body_ema[2] },
};
```

`cmd_alpha` / `head_alpha` 默认都是 `0.2`（`robotd-params/src/lib.rs:858`），`1.0` 就是直通。
`PolicyCommand` 是 `duck_control::obs::Command` 的别名（`main.rs:39`）——到这里，**摇杆才第一次变成
第 4 讲那个 command 块的结构**：`[vx,vy,vyaw] + [neck_pitch,head_pitch,head_yaw,head_roll] + body`。

> 链路小结（本讲的前半）：`robot.move` → `set_twist`（盖章）→ `snapshot` 拼 `Command` →
> `gate` 按 `twist_age` 决定要不要清速度 → 主循环 EMA 平滑 → 一个**干净的 `Command`**。
> 这个 `Command` 会进 `Observation::build` 的 command 块（第 4 讲 61 维的 48..61）。

## 3. control.rs：`Command` → 选 `Net` → 动作缩放与增益

`robotd/src/control.rs` 是"纯计算"的心脏——模块注释第一句（`control.rs:1`）：

> Turning sensors and a command into joint targets — and scheduling the skills. … It holds no
> IO handle — by construction it cannot command a motor, only propose targets.

主循环在 `driving` 为真时调 `controller.step(...)`（`robotd/src/main.rs:1971`），把 `Command`、姿态
开关、`dt`、电压系数 `scale_mult` 交给它。`step()`（`control.rs:303`）内部严格按模块注释的顺序走：

```text
skill windows ← advance / expire (roulade window, kick timer, ground-pick phase, sit↔stand rise)
command      ← the caller's smoothed command, re-encoded for the active skill
net          ← roulade > kick > ground pick > sit/rise > stand-by-magnitude > walk
action       ← ONNX
targets      ← home pose + action_scale × action
filters      ← optional first-order low-pass on head and legs
```

### 3.1 选 `Net`：技能优先链 + 速度决定走/站

`step()` 里选网的一段（`control.rs:339` 起）就是第 4 讲 `Net` 枚举七个变体的**运行时调度**：

- `roulade` 在跑 → `Net::Roulade`，`effective` 命令**全零**（训练时所有槽归零，"被选上"本身即触发）。
- `kick` 窗口在跑 → `KickLeft/KickRight`，命令同样全零。
- `ground_pick` 在跑 → `Net::GroundPick`，twist 槽编码相位：`twist = [cos θ, sin θ, 0]`，
  θ = 2π × phase（镜像训练端的相位编码）。
- 否则进入坐/站/走：`Sit::Sitting` 把 twist 置 `[1,0,0]`（**posture flag 坐在 vx 槽上**）走
  `SitStand`；`Sit::Rising` 置 `[0;3]` 走 `SitStand`（rise 命令全零所以按站立调优）；`Sit::Up`
  才看速度：

```rust
Sit::Up => {
    if body_active { c.twist = [0.0; 3]; }      // body-pose 模式：清零速度
    let standing = self.policy.will_stand(c.twist_magnitude())
        || (body_active && self.policy.has_standing());
    if standing { (Net::Stand, c, "stand") } else { (Net::Walk, c, "walk") }
}
```

"走 vs 站"不是按键决定的，是**速度大小**决定的：`will_stand`（`duck-control/src/policy.rs:277`）=
装了 standing 网络 ∧ 未被 roller 禁用 ∧ `twist_magnitude ≤ standing_threshold`。阈值
`DEFAULT_STANDING_THRESHOLD = 0.05`（`policy.rs:25`），在 `Policy::load(&paths, DEFAULT_STANDING_THRESHOLD)`
时传进去（`main.rs:1096`）。这就是 04 讲那张表里"低于 0.05 切站"的出处。

### 3.2 技能何时被允许：`busy()` 与 `is_sitting()`

技能请求由主循环从 `take_skills()` 取出后，要求"在驱动中"（`snapshot.enabled ∧ Bringup::Ready`，
`main.rs:1447`），再调用 `Controller` 的方法：

| 方法 | 拒绝条件（`control.rs`） |
|---|---|
| `start_ground_pick`（:215） | 没装 ground_pick 网络 / 已经在跑；**可以抢 kick 尾巴**（原型如此，保留） |
| `start_kick`（:227） | 没装该腿网络 / `kick` 或 `ground_pick` 或 `roulade` 任一在跑 |
| `request_roulade`（:244） | 没装 roulade 网络 / ground_pick 在跑；滚动中重复请求 = 续链（`Ok(false)` 安静） |
| `sit_toggle`（:262） | `Rising` 中途拒绝；Up→Sitting 需要 sitstand 网络 |

`busy()`（`control.rs:205`）= ground_pick ∨ kick ∨ roulade ∨ Rising——一个脚本动作在空中，机器人
不管 twist 是什么都在动；`safeToRestart` / limp-fall 仲裁都读它。`is_sitting()`（`control.rs:199`）单独
区分"坐着"（不算 busy，是停着不是移动）。limp-fall 触发条件里就有一条
`!c.busy() && !c.is_sitting()`（`main.rs:1669`）——翻滚是有意躺下、下蹲会前倾、坐着不是摔，三者
都不能让"摔了就 limp"误伤。

### 3.3 缩放与增益：`Tuning` / `SkillTuning` 在每个分支怎么被消费

这是"`robotd.toml` 的数进到控制律"最直接的一段。`step()` 每拍**按当前活动状态重算** `(scale, gain)`
（`control.rs:402` 起），而不是像原型那样跨转换保存/恢复：

```rust
let standing_tuned = matches!(net, Net::Stand)
    || (matches!(net, Net::KickLeft | Net::KickRight | Net::SitStand)
        && self.policy.will_stand(effective.twist_magnitude()));

let (scale, gain) = match net {
    Net::Roulade => (self.skills.roulade_action_scale,
                     (self.tuning.gain as f64 * self.skills.roulade_gain_ratio).round() as u16),
    Net::GroundPick => (self.skills.ground_pick_action_scale,
                        (self.tuning.gain as f64 * self.skills.ground_pick_gain_ratio).round() as u16),
    Net::SitStand => (1.0,
                      if standing_tuned { (self.tuning.gain as f64
                                           * self.tuning.standing_gain_ratio).round() as u16 }
                      else { self.tuning.gain }),
    _ if standing_tuned => (self.tuning.standing_action_scale,
                            (self.tuning.gain as f64 * self.tuning.standing_gain_ratio).round() as u16),
    _ => (self.tuning.action_scale, self.tuning.gain),
};
let scale = scale * scale_mult;      // scale_mult：电压自适应，默认 1.0
```

所以"同一个 `gain`"在不同网络下是不同数：`tuning.gain`（默认 200）是**行走**的 P 增益；站姿/起身/
踢球窗口是 `gain × standing_gain_ratio`（200 × 0.8 = 160，单测 `standing_softens_the_gain`
在 `control.rs:524` 钉着）；limp-fall 用 `safety.gain_limp`（50，第 5 讲）。`scale` 同理：行走
`action_scale`（0.9），站姿 `standing_action_scale`（1.0，*the standing policy is trained to be applied
whole*），roulade/ground_pick 各自从 `SkillTuning` 取（默认 1.0，翻滚按全行走增益跑）。

`scale_mult` 由主循环算好传入（`main.rs:1930`）：`voltage_adapt` 打开时是 `nominal_voltage /
battery_v.clamp(6.0, 9.5)`，默认关、为 1.0。

再往后是 04 讲已铺的尾巴：`offsets = Observation::scatter_action(&action)`（14→15，`control.rs:432`）
→ `targets[joint] = DEFAULT_POSITION[joint] + scale * offsets[joint]`（:435）→ 低通（:438）→
`Ok(Step { targets, label, gain, busy })`。低通用的是 `tuning.head_lowpass`（头颈组 `5..9`）与
`tuning.legs_lowpass`（除头颈与嘴的其余 10 个关节），`None` = 不过滤：

```rust
if let Some(alpha) = self.tuning.head_lowpass {
    for joint in HEAD_JOINTS { targets[joint] = alpha*targets[joint] + (1.0-alpha)*previous[joint]; }
}
```

### 3.4 主循环拿到 `Step` 后

`controller.step` 返回的 `targets` / `gain` 经 `safety.apply(targets, hold, gain)`（`main.rs:2180`）写总线。
`safety` 只在增益变化时才真的 `io.set_gain`（`safety.rs:272`），所以站/走切换不会每拍烧 15 次总线写。
`Step.label` 变成状态帧里的 `policy` 字段（"walk"/"stand"/"kick_left"/…），`robotctl monitor` 显示的就是它。

## 4. 参数从 `robotd.toml` 到强类型：robotd-params

`robotd/src/params.rs` 只有 7 行——它把一切 re-export 自 `robotd-params` crate：

```rust
//! Startup parameters — re-exported from the `robotd-params` crate.
//! The types moved out so `robotctl configure` can edit the file with the real schema, the
//! real defaults and the real validation instead of a copy that drifts.
pub use robotd_params::*;
```

为什么单独成一个 crate？`robotctl configure`（交互编辑这个文件）要跟 `robotd` 用**同一份 schema**
而不是一份会漂移的拷贝。`robotd-params/src/lib.rs:1` 的 crate 文档把两个关键事实钉死：

- **读一次，不 watch**：*Read once at startup, not watched; live reload is deferred … any change
  to the file requires a `robotd` restart*。
- 文件在 `/etc/robot/robotd.toml`（`DEFAULT_PATH`，`lib.rs:30`），在 `releases/<ver>/` 之外，所以
  更新与回滚都不动它。policy 路径默认落在 `RELEASE_DIR`（`/opt/robot/daemon/current`，`lib.rs:27`），
  于是"普通更新把跟二进制一起训练的 onnx 带上来"。

### 4.1 TOML section → Rust 类型的映射

顶层 `Params`（`lib.rs:34`）含十个 section。跟本讲控制路径相关的：

| TOML section | Rust 类型 | 字段（节选） |
|---|---|---|
| `[bus]` | `Bus`（`lib.rs:790`） | `port: String`（默认 `/dev/ttyS2`，`Bus::default` :845） |
| `[control]` | `Control`（`lib.rs:798`） | `hz`（50）、`cmd_alpha`（0.2）、`head_alpha`（0.2），默认在 `lib.rs:853` |
| `[policy]` | `PolicyParams`（`lib.rs:502`） | `enabled`、`mode`、7 个 onnx 路径、`action_scale: Option<f64>`、`gain: u16` … |
| `[safety]` | `SafetyParams`（`lib.rs:678`） | `fall_gravity_z`、`deadman_ms`、`gain_limp`、`limp_fall_*`、`battery_empty_shutdown` |
| `[update_gate]` | `UpdateGate`（`lib.rs:827`） | `min_achieved_hz`、`stall_periods`、`max_consecutive_errors` |

每个 struct 都标 `#[serde(deny_unknown_fields, default)]`——**多一个键要报错**，但未知键的报错在
`Params::load`（`lib.rs:920`）里会被"剪掉未知键重试"的宽容路径接管（旧 build 的配置里留着新 build
没有的 `[section]`，不应该让 daemon 起不来；被剪掉的键会逐条 `warn`）。

### 4.2 默认值在 `Default` impl；`resolved()` 负责"没写的字段按 mode 补"

TOML 解析出来的是"文件里写了什么"，但不一定是"控制律用什么"——**缺省字段要按 mode 解析**。
`PolicyParams::resolved()`（`lib.rs:605`）产出 `ResolvedPolicy`（`lib.rs:577`），doc 说：

> `[policy]` with every absent field resolved against the mode's defaults. This is what the rest
> of `robotd` consumes — nothing downstream should ever have to ask "walk or roller?" to know
> the action scale.

两处默认值来源要分清：

1. **`impl Default for PolicyParams`（`lib.rs:734`）**：`enabled: true`、`mode: Walk`、`gain: 200`、
   `standing_action_scale: 1.0`、`standing_gain_ratio: 0.8`、`kick_duration: 0.5`、
   `roulade_duration: 1.0`……而 `action_scale`、`head_lowpass`、`legs_lowpass`、
   `ground_pick_period` 等是 `None`——它们**没有**写在文件里时的兜底，而是留给 `resolved()` 按 mode。
2. **`resolved()` 里的 per-mode 分支**：`Mode::Walk` 时 `action_scale` → 0.9、
   `head_lowpass` → Some(0.5)、`legs_lowpass` → Some(0.7)、`ground_pick_period` → 4.0；`Mode::Roller`
   时分别 0.8 / 0.5 / 0.7 / 3.0，walk 路径默认指向 `alpha_*.onnx`，roller 指向 `roller.onnx` +
   `roller_crouch.onnx` 且不装 standing 网络。

   ```rust
   action_scale: self.action_scale.unwrap_or(match self.mode {
       Mode::Walk => 0.9,  Mode::Roller => 0.8,
   }),
   head_lowpass: Some(self.head_lowpass.unwrap_or(0.5)).filter(|a| *a < 1.0),
   ```

   `filter(|a| *a < 1.0)` 是"`1.0` = 直通 = 关"的实现——文件里写 `1.0` 就等于把训练带的低通关掉。

`SafetyParams::default`（`lib.rs:766`）同样是一等默认值源：`deadman_ms: 500`、`gain_limp: 50`、
`limp_fall: true` 等。第 5 讲的 `SafetyConfig` 在 duck-control 里**也有**一个 `Default`
（`safety.rs:68`，值就是原型的数），但它主要是给单测 `Safety::new(..., SafetyConfig::default())` 用；
**robotd 生产路径不依赖它**——`main.rs:1137` 从 `params.safety` 逐字段组装，且 `gain_running` 用的是
`[policy] gain`（`policy_cfg.gain`）而不是 `SafetyConfig` 默认的 200。所以真机上"默认值只存在于
`robotd-params` 一处"这句话成立。

### 4.3 `deploy/robotd.toml` 就是默认值的写照

`deploy/robotd.toml:1` 开头写明：*Every value below is the built-in default.* 它装了 `/etc/robot/robotd.toml`
一次之后**安装器永不覆盖**（:9 注释：the installer never overwrites it），所以反注释掉一个值 = 那块板
**永远**用这个值，哪怕 release 默认值后来改了。文件里的教训原话（:24）：*that is exactly how a fleet
ended up standing at kP 120 while the release default said 160.*

以 `[policy]`（`deploy/robotd.toml:76`）为例，它对应的就是 `PolicyParams` + `resolved()`（下面把
注释散文略去、键与默认值照抄）：

```toml
[policy]
enabled = true
mode = "walk"
# walk        = ".../current/policies/alpha_walking.onnx"
# action_scale = 0.9
# standing_action_scale = 1.0
# standing_gain_ratio = 0.8
# gain = 200
# head_lowpass = 0.5        # 训练的 ACTION_LOW_PASS_HEAD_ALPHA，必须匹配
# legs_lowpass = 0.7        # 训练的 ACTION_LOW_PASS_LEG_ALPHA
# ground_pick_period = 4.0
# ground_pick_action_scale = 1.0
# ground_pick_gain_ratio = 1.0
# kick_duration = 0.5
# roulade_duration = 1.0
# roulade_action_scale = 1.0
# roulade_gain_ratio = 1.0
# voltage_adapt = false
# nominal_voltage = 7.4
```

`[safety]`（`deploy/robotd.toml:164`）对应 `SafetyParams`，第 5 讲的表在这里逐条有注释（deadman 那段
在 :176：*Stop, not limp … this one only ever zeroes the velocity*）。`[bus]`（:15）对应 `Bus`。有一个
单测 `the_shipped_example_matches_the_defaults`（`robotd-params/src/lib.rs:1201`）把 `deploy/robotd.toml`
解析后与 `Params::default()` 逐字段对等——**防止文件描述一台不存在的机器人**。

`registry.rs`（`robotd-params/src/registry.rs:78` 的 `REGISTRY` 表）是给编辑器看的机器可读索引：每个
键的 `Kind`（Bool/Integer/Float/Choice/OptionalPath…）与一行 doc。它的完整性由一个测试钉死
（`registry.rs:458` `the_registry_covers_every_key_exactly` 序列化 `Params::default()` 走整棵键树）——
**在 `Params` 加一个 section，build 仍绿，但这个测试会点名 registry 缺哪些键**，于是 `robotctl configure`
不可能静默不知道某个设置。

## 5. 改一个数从哪生效：把 `gain` 追到底

把整条链连起来。"改 `robotd.toml` 里的 `gain`"这句话，展开是：

1. **编辑** `/etc/robot/robotd.toml` 的 `[policy]`，`gain = 250`（或 `sudo robotctl configure`，它用同一份
   schema 写、写前用 daemon 自己的 loader 验证、原子替换——cheatsheet 保证它不能写出 robotd 起不来的文件）。
2. **重启** `sudo systemctl restart robotd`。因为读一次不 watch（§4），不重启就不生效。
3. `main()` 里 `Params::load`（`main.rs:662`）在启动时把 TOML 解析成 `Params`；`--params`/`--port`/
   `--no-policy` 是 CLI 覆盖（`main.rs:657`，clap 定义在 :204）。默认路径不存在也不算错——未布板的机器
   直接跑在 `Params::default()` 上（`lib.rs:920` 注释）。
4. `control_loop` 前置里 `params.policy.resolved()` → `build_controller`（`main.rs:1060`）把
   `ResolvedPolicy` 拆进 `Tuning { action_scale, standing_action_scale, standing_gain_ratio, gain,
   head_lowpass, legs_lowpass }`（`main.rs:1070`）与 `SkillTuning`（:1078）和 `PolicyPaths`（:1087），
   `Policy::load(&paths, DEFAULT_STANDING_THRESHOLD)` 后 `Controller::new(policy, tuning, skills)`（:1115）。
   同一个 `policy_cfg.gain` 又进 `SafetyConfig.gain_running`（`main.rs:1143`）。
5. 每拍 `Controller::step` 按 §3.3 的 `match net` 消费 `tuning.gain`，产出本拍 `gain`；`safety.apply`
   在增益变化时 `io.set_gain` 真写舵机（`safety.rs:272`）。`robotd init` 子命令也在自己的路径里先
   `io.set_gain(params.policy.gain)`（`main.rs:767`）——它发生在 loop 之前，所以不能留上次的
   `gain_limp` 在那儿站着。

参数 → 消费点速查表：

| 想改的 | TOML | Rust 侧消费 |
|---|---|---|
| 行走/运行 P 增益 | `[policy] gain` | `Tuning.gain` → `step()` 默认分支 → `safety.apply`；`SafetyConfig.gain_running` |
| 动作缩放 | `[policy] action_scale` | `resolved()` → `Tuning.action_scale` → `step()` `_` 分支 |
| 站姿更软 | `[policy] standing_gain_ratio` | `Tuning.standing_gain_ratio` → `step()` standing_tuned 分支 |
| 头/腿低通 | `[policy] head_lowpass` / `legs_lowpass` | `Tuning.*` → `step()` 低通段（`control.rs:438`） |
| 技能时长/缩放 | `[policy] ground_pick_*` / `kick_duration` / `roulade_*` | `SkillTuning` → `step()` 对应 `match net` 分支 |
| 指令平滑 | `[control] cmd_alpha` / `head_alpha` | `main.rs:1213` 的 EMA（§2） |
| 循环频率 | `[control] hz` | `Params::period()`（`lib.rs:995`）→ ticker interval（`main.rs:1180`） |
| 遥控新鲜度 | `[safety] deadman_ms` | `SafetyConfig.deadman` → `gate()`（`safety.rs:218`） |
| 摔/limp | `[safety] limp_fall_*` | `SafetyConfig` / `FallPredictorConfig`（`main.rs:1244`） |

一个例外的"不用重启"通道：`robot.setMode`（D-pad）让 loop 在运行中把 `policy_params.resolved()` 重解
一遍并 `build_controller`（`main.rs:1842`）——但那只换 `[policy]` 里的 mode 预设，不是重读文件。所有
`robotd.toml` 的改动仍然要 restart。

## 6. 自己动手

### 【无鸭可跑】

下面都是只读追踪或轻量单测，不碰文件、不需真机。先决条件仍是先装 Rust 工具链（见 00 导航）：

```bash
cd ~/microduck
# robotd-params 不依赖 ort，编译很轻；这些测试把"文件↔默认值↔resolved 结果"钉在一起：
cargo test -p robotd-params the_shipped_example_matches_the_defaults
cargo test -p robotd-params walk_mode_resolves_to_the_prototype_alpha_config
cargo test -p robotd-params roller_mode_resolves_to_the_prototype_roller_preset
cargo test -p robotd-params command_smoothing_defaults_match_the_prototype
```

读代码把 `gain` 追一遍（不真改）：

```bash
grep -n 'gain' deploy/robotd.toml                 # [policy] 里的注释默认
sed -n '56,84p'  robotd/src/control.rs            # Tuning::default 与 Default
sed -n '1060,1124p' robotd/src/main.rs            # ResolvedPolicy → Tuning/SkillTuning/Controller
sed -n '402,430p'  robotd/src/control.rs          # step() 里 (scale, gain) 的分支
```

想真正看"一个改过的值被读入"而不上真机：`robotd-params` 的测试模式提供了 `write(dir, body)` 助手，
你可以临时在 `robotd-params/src/lib.rs` 的 tests mod 里加一条（**看完删掉，别提交**）：

```rust
#[test]
fn a_higher_gain_survives_load_and_resolve() {
    let dir = tempfile::tempdir().unwrap();
    let path = write(dir.path(), "[policy]\ngain = 250\n");
    let p = Params::load(&path, true).unwrap().policy.resolved();
    assert_eq!(p.gain, 250);
}
```

跑 `cargo test -p robotd-params a_higher_gain_survives_load_and_resolve`。这验证的是 §5 第 3、4 步：
TOML → `Params` → `ResolvedPolicy.gain`。（control 层消费它需要编 `robotd`、依赖 ort，较重——见
第 1 讲无鸭矩阵；真要跑得先整仓编译。）

### 【需真机】

在有鸭板上验证"改一个数真的改了机器"：

```bash
sudo robotctl configure          # 交互改 /etc/robot/robotd.toml（会提示重启哪些 daemon）
# 或直接编辑后：
sudo systemctl restart robotd
journalctl -u robotd -b | grep 'policy loaded'     # build_controller 的日志，含各 onnx 路径
robotctl monitor                                   # 状态帧底部：policy 标签 / 增益 / 达成 Hz
```

把 `gain` 从 200 调到 160 再走几步：同一速度下机器人明显"软"（站姿本来就是 200×0.8=160，走姿这次
也降到 160）。改 `[safety] deadman_ms` 到 200：松开摇杆后约 0.2 s 内机器人就该自己停下——这是
deadman 从"值"变成"停"的最直观观测。任何改动都要 restart 才生效；`robotctl configure` 只写
**被改的键**并原子替换，不会吃掉注释和其他 release 的键。

## 7. 收尾检查

- [ ] 主循环怎么知道摇杆"还活着"？（答：twist 槽带 `at_us` 时间戳，`snapshot().twist_age` 与 `deadman_ms` 比；超了就 `gate` 清速度，且只清 twist）
- [ ] `robot.move` 到 obs 的 command 块之间有几层？（答：`set_twist` 盖章 → `snapshot` 拼 `Command` → `gate` 过 deadman → 主循环 EMA 平滑 → `step()` 按技能重编码 → `Observation::build`）
- [ ] 走 vs 站是谁决定的？（答：不是按键，是 `will_stand(twist_magnitude)`，阈值 0.05 在 `policy.rs:25`）
- [ ] `gain` 有哪些不同的"当前值"？（答：`tuning.gain`=200 行走；站/起身 ×0.8=160；limp-fall 用 `gain_limp`=50；翻滚/下蹲可再乘各自的 `gain_ratio`）
- [ ] 改 `robotd.toml` 为什么要重启？（答：robotd-params 读一次不 watch；不重启 = 老值一直在跑）
- [ ] 默认值到底存在哪一处？（答：`robotd-params` 的 `Default for PolicyParams/SafetyParams/…` + `resolved()` 的 per-mode 补全；`deploy/robotd.toml` 是写照并有单测钉死）

**下一步**：第 8 讲把这一侧的"外皮"接上——`duck-ipc-proto` 与 `robotctl`：socket 上这些 `robot.*`
RPC 怎么分帧、`robotctl monitor` 怎么读回状态，以及对照你的飞特/IMU 改造该在哪下刀。

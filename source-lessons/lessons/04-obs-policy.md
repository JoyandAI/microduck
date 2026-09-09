# 04 · 观测与策略：61 维 obs 拼装到动作缩放

> 目的：看懂"观测怎么拼、策略怎么跑、14 维动作怎么缩回 15 个关节目标"这条链。
> 这是全仓**风险最高的代码**——错一格不报错，只会在真机上摔倒。

## 1. obs.rs：61 个浮点，一格都不能错

`duck-control/src/obs.rs:1` 模块注释第一句就是警告：

> *This is the **highest-risk code in the crate**. It is a flat array of 61 floats whose every
> index must match what the policy was trained against. A wrong offset does not fail loudly —
> it produces a plausible-looking robot that falls over, and the symptom looks like a tuning
> or timing problem rather than an indexing one.*

布局表（`obs.rs:16` 起，doc 里原文照抄）：

```text
index   width  contents
0..3        3  gyro, trunk frame, rad/s
3..6        3  projected gravity, trunk frame, unit vector
6..20      14  joint position minus home pose, mouth excluded
20..34     14  joint velocity, mouth excluded
34..48     14  previous action, mouth excluded
48..61     13  command (below)
```

尾部 command 块（全 obs 里**唯一没有第二真相源**的部分）：

```text
48..51      3  vx, vy, vyaw
51..55      4  neck_pitch, head_pitch, head_yaw, head_roll
55..57      2  body x, y      — always zero, unbound in training
57          1  body z
58          1  body roll
59          1  body pitch
60          1  body yaw       — always zero, unbound in training
```

两处**极容易写错、且对着 `microduck_runtime` 核实过**的坑（`obs.rs` doc 用编号列了）：

1. **body x/y/yaw 是写死 0 的**——它们在训练环境里"未约束"，所以全零 body 命令是**标称编码**，
   不是"以后会补的占位"。
2. **头部位姿在 command 里、且不叠加到策略输出上**。原型有两种模式都做、还叠加，最后用
   `if !new_cmd_obs` + 注释 *"head_offsets are a COMMAND fed via the obs vector instead — don't
   double-add it here"* 把它关掉。两处都做会把头弯两次。

还有一处顺序陷阱：body 块是 `z, roll, pitch`，不是 `z, pitch, roll`。

### 1.1 嘴是怎么被"跳过"的（14 与 15 的桥）

```rust
pub const ACTION_LEN: usize = 14;              // 15 关节 − 嘴
pub const OBS_JOINTS: usize = NUM_JOINTS - 1;  // 观测里同样没有嘴
pub const COMMAND_LEN: usize = 13;
```

跳嘴的映射只写了一次、读写共用（`obs.rs`）：

```rust
const fn joint_of(slot: usize) -> usize {      // 策略槽位 → 关节下标
    if slot < MOUTH_INDEX { slot } else { slot + 1 }
}
```

读方向 `policy_joints()`（positions/velocities/home 三块**同一份**过滤，三块不会对不上）、
写方向 `scatter_action()`（把 14 维动作绕开 mouth 回填 15 个槽）都走 `joint_of`——

```rust
pub fn scatter_action(action: &[f32; ACTION_LEN]) -> [f64; NUM_JOINTS] {
    let mut out = [0.0f64; NUM_JOINTS];
    for (slot, value) in action.iter().enumerate() {
        out[joint_of(slot)] = *value as f64;   // mouth 的槽保持 0，不碰
    }
    out
}
```

> 为什么写这么绕？因为**这里错一位 = 关节 9 之后全部平移一格**，"既灾难又完全安静"。
> 用同一个函数的两端 + 固定宽度数组，把"对不齐"变成编译错误而不是运行时偏移。

`fill()` 的注释补了一个哲学：**观测里出现 0 = 该关节停在 home 位**。所以宁可用定宽数组让编译器
拒绝宽度不匹配，也不能用 `zip`（zip 在短的一侧悄悄停，尾巴留 0 → 策略对着一个"不存在的、
看起来正常的机器人"动作）。

## 2. policy.rs：加载期就把形状钉死

`policy.rs:9` 模块注释是部署安全的关键：

> *Everything is validated **at load**, not at inference. A bundle with the wrong observation
> width, the wrong action count, or a missing ONNX Runtime must fail while the robot is
> standing still — not sixty ticks later, mid stride.*

所以每个网络加载时都 `check_width`：

```rust
check_width(path, "observation width", session.inputs(), OBS_LEN)?; // 61
check_width(path, "action count",      session.outputs(), ACTION_LEN)?; // 14
```

宽度不对 → **加载期就报错**，`robotd` 把它变成"保持姿势 + 报 unhealthy + 说清原因"，
由 updater 回滚——而不是放一台动作无法解释的机器在地上跑。

### 2.1 网络与"谁来选"

`Net` 枚举（`policy.rs:120`）：`Walk / Stand / SitStand / GroundPick / KickLeft / KickRight /
Roulade`。七个技能共享**同一个 61 维布局**——技能只是"换个 session + 换一种 command 编码"，
不是新的契约。走 vs 站由**速度指令大小**自动选：

```rust
pub const DEFAULT_STANDING_THRESHOLD: f64 = 0.05;   // 低于它切站
```

`Policy` 结构里 `stand`/`ground_pick`/`kick_*` 都是 `Option<Session>`——没配对应文件就是没有
这个能力，不是崩溃。哪个网络驱动这一拍是 `robotd` 的调度器说了算（第 6 讲见）。

### 2.2 对 ONNX Runtime 的防御（值得一读的注释）

`ort` 用 `load-dynamic`：**编译期不需要、运行时才 `dlopen`** `libonnxruntime.so`。但 `ort`
对"库缺失/版本不符"是 `expect` panic 而不是返回错误。为此 policy.rs 做了两层防御：

- `ensure_runtime()`：用 `libloading` 在进 `ort` 前先探一下库能不能加载，把"缺库"变成一条
  能告诉操作员的普通错误（`RuntimeMissing`，提示装库或设 `ORT_DYLIB_PATH`）。
- `catching_ort_panics()`：`catch_unwind` 包住所有 `ort` 调用。注释里有个真事——一块板上
  ONNX Runtime 1.20.1，探针通过（文件能加载）但 `ort` 在自己的版本检查里 panic
  (*expected version >= '1.23.x', but got '1.20.1'*)。所以探针只证明"文件能加载"，
  剩下的靠 catch 兜住，连没见过的 panic 也变成可上报错误。

还有一句部署教训写在注释里：**`panic = "abort"` 会废掉这套**——根 `Cargo.toml` 刻意不设
`[profile.release]`，加上就又把控制线程的死法改回去了。

## 3. 从 14 维动作到 15 个目标角度（control.rs）

动作离开 `policy.infer()` 后（`robotd/src/control.rs`），链是：

1. 按当前状态选 `(scale, gain)`：走/站/roulade/ground_pick/sitstand 各有 `action_scale` 与
   `gain_ratio`（`tuning.action_scale` 走默认 0.9、`standing_action_scale` 站姿、技能各自一套）。
2. `offsets = Observation::scatter_action(&action)` —— 14 → 15，绕开嘴。
3. `targets[joint] = DEFAULT_POSITION[joint] + scale * offsets[joint]` —— 在 home 上叠动作。
4. 低通滤波：`head_lowpass`（头颈组）与 `legs_lowpass`（腿组）各自一个 `alpha` 的 EMA——
   `targets = α·targets + (1−α)·previous`，**必须等于训练侧的 `ACTION_LOW_PASS_*`**。
5. 写总线（`RobotIo::write`），拍结束。

> 你训练出的 ONNX 是 `obs[1,61] → actions[1,14]`，但**缩放到关节目标的这些系数不在 ONNX 里**，
> 在 `robotd.toml` / `robotd-params`（第 7 讲）。这就是为什么"只换 ONNX"不够、还要对参数。

## 4. 自己动手【无鸭可跑】

```bash
cd ~/microduck
cargo test -p duck-control the_layout_widths_sum_to_the_declared_input
cargo test -p duck-control every_block_lands_at_its_documented_offset
cargo test -p duck-control unbound_body_axes_are_always_zero
# policy 的纯逻辑测试（不加载 session；要加载 session 的测试需要系统里有 libonnxruntime）：
cargo test -p duck-control the_standing_threshold_matches_the_prototype
```

改一个数看它多快红：把 `obs.rs` 里 layout 注释对应数组的某一格挪一位，跑
`every_block_lands_at_its_documented_offset`——这个测试就是把"注释布局 ↔ 真实拼装"钉在一起。

## 5. 收尾检查

- [ ] 61 = 3+3+14+14+14+13 里，哪两块是"唯一真相源在别处"的？（答：positions/velocities 来自总线；command 没有第二真相源）
- [ ] 为什么 body x/y/yaw 要写死 0？（答：训练里未约束，全零是标称编码）
- [ ] 加载期校验宽度 vs 推理期校验，差在哪？（答：前者站在地上告诉你，后者跑一半摔了才知道）

**下一步**：第 5 讲看这些数怎么喂给摔倒检测与"该松力还是该保持"的裁决。

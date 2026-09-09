# 00 · 使用指南与学习地图

> 本系列是**内部源码精读**：目标是让你**读懂并动手改** `pollen-robotics/microduck` 的
> Rust 运行时——以你手上的飞特（Feetech）换装与 IMU 直连改造为练兵场。它**不是**站内公开教程，
> 不受"三级证据/篇幅/无鸭主线"约束，可以直白引代码、谈改造。

## 这套东西放在哪

```
~/microduck/                     ← 讲授对象：官方运行时源码（19-crate cargo workspace）
  source-lessons/                ← 本系列（此目录不参与编译，也不提交上游）
    lessons/00-*.md … 08-*.md    ← 每讲一份 Markdown（源文件，可 grep/编辑）
    build.mjs                    ← 构建脚本：md → index.html
    index.html                   ← 展示页（双击在浏览器打开，离线自包含）
    assets/marked.min.js         ← 本地化渲染库（离线，不用 CDN）
```

改讲稿 → 在 `source-lessons/` 下跑 `node build.mjs` 重新生成 `index.html`。

## 先读三件事

1. **位置**：每讲都以 `repo:文件:行号` 精确指向真实代码。请开一个终端、`cd ~/microduck`，
   边读边 `sed -n '…p'` 或直接 `code duck-control/src/xxx.rs` 对照。
2. **前置工具**：本机（写本系列时）**还没有 Rust 工具链**（`rustc`/`cargo` 均不在 PATH，
   `~/microduck/target` 也不存在）。命令都列好了，但要真的"跑"得先装：
   ```bash
   curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
   # 装完重开 shell；验证：cargo --version
   cd ~/microduck && cargo test -p duck-control    # 只测控制核心，不需要鸭子/不需要编译全仓
   ```
   只想读不装也完全能跟上——每讲"自己动手"栏都区分了【无鸭可跑】与【需真机】。
3. **三条借力**：这仓库自带一手文档，讲稿引用但不重复：
   - `docs/design/robotd-design.md`（控制路径设计，第 2 节是 bus/IMU/20ms 的出处）
   - `docs/robot/cheatsheet.md` + `docs/robot/cheatsheet-dev.md`（`robotctl` 每一条命令）
   - `~/microduck_rl/docs/`（你的飞特换装文档；第 8 讲把源码理解接回它）

## 一个心智模型：20 ms 的一拍

整个运行时其实只有一张动图。把下面这句话记住，后面每一讲都在填它的细节：

> **每 20 ms（50 Hz）一拍：先在总线上"读一次"拿到 15 个舵机 + 1 个 IMU 的原子快照
> （`Sensors`）→ 纯计算拼出 61 维观测 → 丢给 ONNX 策略推理 → 得到 14 维动作 →
> 缩放成 15 个关节目标角度 → 在总线上"写一次"。**
> 所有能"摔不坏、刷不砖"的安全机制，都是插在这条路径上、或者贴着它转的看门狗。

`duck-control` 是这条路径的"纯逻辑心脏"，`robotd` 是"守门进程"（定时器、IPC、systemd），
其余 crate 是外围。改造飞特/IMU 时你 90% 的时间都在 `duck-control` + `robotd` 里。

## 八讲地图

| 讲 | 标题 | 主要对象 | 你拿走的东西 |
|---|---|---|---|
| 00 | 使用指南与学习地图 | 本页 | 心智模型、阅读法、装工具链 |
| 01 | 先把仓库跑起来 | workspace / duck-control | crate 地图；无鸭 `cargo test` 全绿；fake 哲学 |
| 02 | 常数表与一次事务 | model.rs / io.rs | 15 关节怎么命名编号；`RobotIo` 为何"一次事务" |
| 03 | 总线读与姿态解码 | bus.rs / imu.rs | 一条 `sync_read` 读 16 设备；SFLP 姿态怎么变成 gravity |
| 04 | 观测与策略 | obs.rs / policy.rs | 61 维 obs 的每一格；14→15 的动作缩放链 |
| 05 | 安全网：摔倒与 limp | safety.rs / fall.rs | "读到的数"如何变成"松力/断电"的裁决 |
| 06 | robotd：50 Hz 主循环 | robotd/main.rs | tick 骨架：read→coast→intents→gate→bringup |
| 07 | 意图与配置落点 | intents / control / robotd-params | 摇杆→命令→`robotd.toml` 参数从哪生效 |
| 08 | IPC、robotctl 与你的改造切入点 | duck-ipc-proto / robotctl | 从外面怎么读它；飞特/IMU 下刀清单 |

建议顺序 00→08；02/03/04 是 duck-control 主线，05 可后置但推荐按序。

## 无鸭可跑矩阵

| 我想跑 | 命令 | 要真机吗 | 前提 |
|---|---|---|---|
| 核心单测全跑 | `cd ~/microduck && cargo test -p duck-control` | 否 | 装 Rust；首次编译拉依赖较久 |
| 跑某一个测试 | `cargo test -p duck-control imu_id_does_not_collide` | 否 | 同上（按函数名子串过滤） |
| 假鸭 50Hz 在电脑上跑 | `cargo run -p robotd -- --fake`（需先整仓编译） | 否 | 依赖 ort/onnx 等，较重，见第 6 讲 |
| `robotctl` 看机器人 | — | 是 | `robotctl` 走 unix socket 找 `robotd`，无鸭无 socket |

## 术语速查

| 符号 | 含义 |
|---|---|
| `NUM_JOINTS` = 15 | 关节数（含鸭嘴）；策略动作只输出 14 维（跳过 mouth） |
| `IMU_DXL_ID` = 200 | `imu_to_dxl` 板在总线上的 ID，排在所有舵机之前、同一包读回 |
| `Sensors` | 一次原子快照：15 位置/速度/电流 + `imu` |
| `RobotIo` | 控制循环碰物理世界的唯一边界（trait） |
| `FakeIo` | 无硬件的假实现，单测与 `--fake` 都靠它 |
| `obs[1,61] → actions[1,14]` | 策略 IO 契约（导出 ONNX 也按这个形状） |
| gravity | IMU 给的"上"方向单位向量，安全与观测都吃它 |

## 阅读法（给非 Rust 熟手）

- 先读**模块顶部的 `//!` 文档注释**——这仓库注释极好，几乎每段都是设计决策的坦白。
- 再找**结构体与 trait**（`Sensors`/`RobotIo`/`SflpDecoder`），数据形状决定一切。
- 最后看**函数**。别急着读 `robotd/src/main.rs`（4200+ 行），从 `duck-control` 小文件进。
- 每讲"自己动手"至少做一次；**读不懂就只看那一段代码**，别整文件硬啃。

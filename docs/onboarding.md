# 快速上手 Microduck（Onboarding）

> 这份文档是进入这个仓库的**路径图**：从"这是什么"到"我能跑起来吗"再到"我想改一行代码并看它跑在真的鸭子上"。
> 所有细节背后都有更深的参考文档，文末有索引；这里只给结论和命令。
>
> 仓库里的其他文档都是英文的，为了让你快速入手，本页用中文写。命令本身原样照抄即可。

---

## 0. 一句话背景

Microduck 是一只约 25 cm、800 g 的**双足小鸭机器人**，用**强化学习策略**行走。机身在
Radxa Zero 3W（Rockchip RK3566, aarch64）上跑一组 Rust 守护进程：

- 50 Hz 控制回路驱动 15 个舵机 + IMU（走、站、滚、坐下、踢球、翻身站起来）；
- 神经网络策略从 [`microduck_rl`](https://github.com/pollen-robotics/microduck_rl)（MuJoCo + PPO）训练，
  导出 ONNX，本仓库加载；
- 一套**签名、可回滚**的更新系统（`updaterd`），让新代码能到 robot 上而不会变砖；
- 手机 App 路径（BLE）、游戏手柄、WebRTC 远程、ToF 深度传感器、摄像头。

**这个仓库就是"鸭子的脑子"。** 当前版本 `0.10.0`，942+ 个测试全部通过（CI 强制），
每个 release 都先走 staging 再 promote。

---

## 1. 先选一条路

| 你想做什么 | 去哪 | 需要什么 |
|---|---|---|
| **A. 只是读代码、跑测试** | [§2 构建与测试](#2-构建与测试)，[§7 仓库地图](#7-仓库地图) | 一台电脑 + Rust 1.89+ |
| **B. 没有硬件也想跑起来**（fake 模式） | [§3 无硬件运行](#3-无硬件也能跑--fake-模式) | 同上，再编几个 daemon 即可 |
| **C. 有一台真鸭（已买/已配好）** | [§4.1 使用真机](#41-我只想用它) | 一台真机，会 `robotctl` |
| **D. 从零弄一块开发板，改代码 → 推到板子** | [§4.2 开发板](#42-开发板从零开始)，[§5 开发回路](#5-开发回路改一行代码到板子上) | 板子、ssh key、GitHub token、交叉编译工具链（或 Docker） |
| **E. 参与发布**（tag、staging/promote） | [§6 发布与 CI](#6-发布与-ci) | GitHub 权限 + CI（签名只在 CI 做） |

绝大多数"上手"场景是 **A → B → C/D**：先把测试跑绿，再在 fake 模式下看看各自 daemon 长什么样，
最后拿到板子。下面按这个顺序走。

---

## 2. 构建与测试

### 2.1 前置

- **Rust 1.89+（stable）**，`rustup` 安装即可（项目是 edition 2024）。
- **Linux 主机**需要几个 C 开发包（macOS 不需要）：

  ```bash
  sudo apt-get install -y libudev-dev libgstreamer1.0-dev \
      libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev
  ```

  为什么：`padd` 通过 `gilrs` 绑定 `libudev`；`mediad` 的 GStreamer 管线是
  `cfg(target_os = "linux")`，所以 Linux 上会编译 GStreamer，Mac 上不会。

- **不需要**：任何硬件、网络、Docker。`cargo test --workspace` 全绿 = 你的 checkout 是好的。

### 2.2 日常命令

```bash
cargo test --workspace            # 全量测试（Mac 上 942 个，Linux 上略多）
cargo test -p robotd              # 单个 crate，快很多
cargo fmt --all                   # 格式化
RUSTFLAGS="-D warnings" cargo clippy --workspace --all-targets   # CI 会跑，本地也建议
```

针对板子的 lint（`configd`/`btd` 依赖 NetworkManager/BlueZ，**仅有 Linux 编译**，主机测试通过不代表它们没问题）：

```bash
RUSTFLAGS="-D warnings" cargo clippy -p configd --all-targets --target aarch64-unknown-linux-gnu
```

### 2.3 几个"为什么"（省得踩坑）

- `configd` 的 NetworkManager 客户端和 `btd` 的 BlueZ 客户端是 **Linux-only**：在 Mac 上它们不编译。
- 测试里面最值得看的是 `updater/tests/apply.rs` —— 它用**真引擎 + 注入故障**测试更新路径
  （坏签名、不健康回滚、postinstall 失败、swap 后断电），比任何手工操作更能回答"这到底保证什么"。
- `docs/**` 和所有 `*.md` 被 CI 排除：改文档不会触发交叉编译和覆盖率，**pr 只会跑 `check` 相关 job**。

---

## 3. 无硬件也能跑：`--fake` 模式

所有关键 daemon 都有离线模拟开关，是理解整个系统最快的路径。

```bash
# robotd：对着一只"由空气组成"的鸭子跑 50Hz 回路
cargo run -p robotd -- --fake --no-policy --socket /tmp/robotd.sock

# configd：假的网络后端（仿真 wifi/名字/PIN 全套 API）
cargo run -p configd -- --fake-net --socket /tmp/configd.sock

# tofd：假深度传感器，照常发布 8x8 深度帧
cargo run -p tof -- --fake
```

一些排障用的开关（正好是更新引擎回滚路径的测试工具）：

| 开关 | 作用 |
|---|---|
| `robotd --unhealthy` | 报告不健康 —— 用来测 updater 的自动回滚 |
| `robotd --busy` | 报告"现在不安全重启"（仿佛在走路） |
| `robotd --no-policy` | 不加载策略，只保持开机姿态 —— 测 updater 时用它 |

然后用 `robotctl` 连上去看：

```bash
cargo run -p robotctl -- --robot-socket /tmp/robotd.sock monitor
cargo run -p robotctl -- --robot-socket /tmp/robotd.sock health
```

> 每个 daemon 都接受 `--socket` 覆盖（默认在 `/run/` 下，那是 systemd 的 RuntimeDirectory，
> 本机无 systemd 单元时自己指定即可）。`robotctl` 的全局参数 `--socket`/`--robot-socket`/
> `--config-socket`/`--pad-socket`/`--tof-socket` 就是干这个的。

笔记本侧的客户端 `duckctl`（**永远不会被交叉编译、永远不会装到机器上**，它走 BLE 连真机）：

```bash
cargo run -q -p duckctl -- --name <机器人名> info
cargo run -q -p duckctl -- scan            # 扫一下附近有没有在广播的鸭子
```

---

## 4. 真机

### 4.1 我只想用它

如果你是买了机器只想开箱即用，全部入口在
[`docs/robot/cheatsheet.md`](robot/cheatsheet.md)（`robotctl` 每个命令）和
[`docs/robot/duckctl.md`](robot/duckctl.md)（笔记本用蓝牙控制）。第一反应：

```bash
robotctl version     # 每个 daemon 在跑什么 vs 安装了什么。任何诊断之前先跑这个
robotctl health      # 硬件+软件一份报告；不健康时退出码非 0，可给脚本当闸门
robotctl monitor     # 实时看回路：意图 vs 实际、关节、IMU 姿态、电池、策略名
```

游戏手柄：`sudo robotctl pad pair`（每个手柄一次），按键映射见 cheatsheet。

### 4.2 开发板：从零开始

目标：一块空白板子 → 能装**团队 dev key 签名的分支构建**的"开发板"。
（客户机与开发板的唯一区别就是信任哪把钥匙；开发板会拒绝 release 构建的细节不在本文范围。）

**硬件**：Radxa Zero 3W，flash [Armbian 26.2.1 Minimal](https://www.armbian.com/radxa-zero-3/)
（[Armbian imager](https://www.armbian.com/radxa-zero-3/) 里选 Radxa Zero 3 → **Armbian 26.2.1 Minimal**），
刷写时在 imager 里填好 wifi 和用户名/密码，开机即连网、可直接 ssh。

**依次准备**：

```bash
ssh-copy-id radxa@192.168.1.42                     # 你的 ssh key（provision 会重启板子并自己重连）
export DUCK_TOKEN=github_pat_replace_with_your_token   # 仓库暂时私有，release assets 需要
```

**一条命令完成 provisioning**（从本仓库克隆目录里）：

```bash
./scripts/provision-board.sh --pause-btd-on-pair --name <MY_COOL_ROBOT_NAME> radxa@192.168.1.42
```

它做的事情：发 dev key → 跑 provision（`setup-board.sh` → `migrate-network.sh` → `install.sh`）→
等重启 → 流式看日志 → 最后 `robotctl health`。是**旁观者**：板子自己装完（systemd resume unit），
Ctrl-C 不损失任何东西，日志在 `/var/lib/robot/provision.log`。

**为什么默认带 `--pause-btd-on-pair`**：aic8800 蓝牙模块在 `btd` 广播时手柄无法建立**新**绑定，
暂停一下只为配对窗口。（另有个 `--weird-ble`，只在"暂停也没用"时用，别乱开——见
[`install-dev.md`](robot/install-dev.md#the-three-configurations-and-how-to-tell-which-you-have)。）

**验证真的成了开发板**（这两件事可以查，不用靠记忆）：

```bash
robotctl health
grep -c 'DEV BOARD' /var/lib/robot/provision.log    # 1 = dev key 真的装上了
```

**放一个分支构建上去**：

```bash
sudo robotctl update apply --ref main daemon
```

⚠️ 注意：`apply daemon` **不带 `--ref` = 装最新 stable**，在开发板上通常是**降级**。写命令习惯带上 `--ref`。

常用 flags：`--local`（用本克隆的 provision.sh 测试改动）、`--no-dev-key`（只要 release 的板子）、
`--ref BRANCH`（用分支构建来 provision）。ssh 报 host key 变化（reflash 后）：
`./scripts/provision-board.sh radxa@192.168.1.42 --forget-host-key`。

---

## 5. 开发回路：改一行代码 → 板子上跑

**一次性的准备工作**（三件事，之后不再做）：

1. 板子是开发板（§4.2）。
2. dev 签名私钥放在 `~/.duck-keys/team.dev.key`（CI 签名分支构建用的那把；`DUCK_DEV_SECRET_KEY` 可换路径）。
3. 能交叉编译的工具链，**二选一**：

   ```bash
   # 方案一：zigbuild（默认路径，快）
   cargo install cargo-zigbuild --locked
   brew install zig                      # 或 apt install zig

   # 方案二：什么也不用装，用 Docker 构建（慢一点但零依赖）
   # 之后所有 dev-push 都加 --docker
   ```

**然后就是一条命令**：

```bash
scripts/dev-push.sh --name duck-c51b          # 按名字找机器人（蓝牙询问地址，结果缓存）
scripts/dev-push.sh radxa@192.168.1.42        # 或者直接给地址，跳过蓝牙
export DUCK_ROBOT=duck-c51b && scripts/dev-push.sh   # 也可用环境变量
```

它做的事：交叉编译整个 workspace → 打包成和 release 一样的 artifact → 用 dev 密钥签名 →
拷到板子 `~/duck-sideload` → 通过 `robotctl update apply --from` 走**普通 gated 更新**安装 →
等全部 daemon 报告新版本。

**这就是一次普通更新**：签名、哈希、兼容性、健康门、自动回滚全都在。
所以它坏掉了也会自己回滚；故意回滚也是普通命令：

```bash
sudo robotctl update rollback daemon
```

只编 `imu_probe` / `hls_read` / `hls_control`（不走整包更新）：Windows 上用 WSL +
`aarch64-linux-gnu-gcc`，步骤见 [`docs/robot/bringup-examples.md`](robot/bringup-examples.md)。

版本号是 `0.5.1-dev.local.<时间戳>.<rev>` 形式，**两次构建永不混淆**，工作区脏也没关系。

装 CI 的分支构建（不 push、不本机编译）：

```bash
sudo robotctl update apply --ref <my-branch> daemon
# gh run list --branch main    # 确认 CI 已经构建完，否则 --ref 解析不到
```

---

## 6. 发布与 CI

**机制一句话：CI 是唯一签名的地方；tag 决定 CI 做什么；release 从不本地产生。**

| 你 push 什么 | CI 干的事 |
|---|---|
| pre-release tag `daemon-staging-v0.4.0` | 构建、签名、用**真更新引擎**验证、发布到 **staging** |
| release tag `daemon-v0.4.0` | **promote** staging 0.4.0（同一份字节，重新签名）；没有就直接构建 |

```bash
git tag daemon-staging-v0.4.0 && git push --tags   # 先上 staging，装一只能到真鸭子上
# 验证 OK 后：
git tag daemon-v0.4.0 && git push --tags           # 正式发布 = promote
```

**发布前先 bump workspace 版本**（`Cargo.toml: [workspace.package] version`）——`xtask package`
会拒绝和 tag 不一致的版本，这是防止"鸭子上报一个它没在跑的版本"。

本地的构建工具（脚本化发布用的，**永远不会装到机器人上**）：

```bash
cargo run -p xtask -- package --version 0.4.0 --channel daemon --bin-dir <dir> --out dist/
cargo run -p xtask -- sign    --dir dist/ --key secret.key
cargo run -p xtask -- promote --version 0.4.0 --staging-tag daemon-staging-v0.4.0 --stable-tag daemon-v0.4.0 ...
```

workflow 全家：`ci`（push/PR 检查）、`dev`（分支构建发布）、`release`、`promote`（按 tag 触发）、
`prune-dev-releases`。board 交叉编译用 `cargo board`（= `zigbuild --release --target aarch64-unknown-linux-gnu.2.31`）。
`scripts/board-test.sh` 在 CI 里对真实 userland（Debian 13）执行 60 条断言，其中包括
rollback、篡改拒绝、boot 计数恢复、socket 权限、peer-cred 授权。

---

## 7. 仓库地图

\[workspace] 一个成员 = 一个进程/一个工具/一个库。核心原则：**一个服务一个 crate 一个 systemd unit，
同样一包 release 产物里的二进制**。

### 守护进程（每个都是独立 unit，都是同一个 release 的产物）

| crate | 是什么 | 拥有/负责 | 监听 |
|---|---|---|---|
| `robotd/` | 控制守护进程 | 50Hz 回路、语音、theremin、chorale；**唯一能命令电机的东西** | `/run/robotd.sock` |
| `updater/` | 更新引擎 + `updaterd` | verify、swap、health gate、rollback | `/run/updaterd.sock` |
| `configd/` | 配置守护进程 | wifi（NetworkManager）、机器人名、配对 PIN、重启、手柄配对 | `/run/configd.sock` |
| `btd/` | BLE 前门 | **一无所有**——只是 API 的 BLE 运输层 | BLE GATT |
| `padd/` | 手柄 → intents | 普通 socket 客户端，无特权 | `/run/padd/pad.sock` |
| `mediad/` | 相机、麦克风、WebRTC、远程网关、Web 控制台 | 媒体管线；远程前门 | TCP `:8080`（控制台）/ `:8443`（信令） |
| `tof/` | `tofd`：头部 8×8 深度传感器 | 发布深度帧，谁也读不到 | `/run/tofd/tof.sock` |

### 库（没有 socket、没有 systemd、没有任何进程）

| crate | 是什么 |
|---|---|
| `duck-ipc-proto/` | **线协议**。JSON-RPC 2.0 的所有消息类型。只依赖 serde/serde_json/semver —— 恢复路径不牵扯更新引擎的 http/tar/crypto 依赖树 |
| `duck-control/` | 控制核心：模型、总线、IMU、观测、策略、安全。纯库：无 tokio |
| `kinematics/` | MJCF 模型 + 正运动学；头和手链 |
| `odometry/` | 从足底接触 + IMU 推算"去过哪"，相对运动、会漂 |
| `sounds/` | 语音合成、每台机器人的人设、chorale 乐谱 |
| `pet-detect/` | 一个听头挠痒的小 CNN（板载麦克风） |
| `robotd-params/` | robotd 启动参数：schema、默认值、校验（`robotctl configure` 和 `robotd` 共用） |

### 工具、交付物

| 路径 | 是什么 |
|---|---|
| `robotctl/` | 板上 CLI（含 `monitor`）。**跟机器走** |
| `duckctl/` | 笔记本侧客户端（BLE）。**永不发布、永不交叉编译** |
| `xtask/` | package · sign · promote —— 发布构建工具，不 ship |
| `test-support/` | 测试用的签名 release 夹具，不 ship |
| `deploy/` | 机器人镜像的 OS 级配置：`updater.toml`、信任锚、journald 配置、`robotd.toml` 参考 |
| `policies/` | 一个 release 带的 ONNX 网络 |
| `hooks/` | preinstall / postinstall —— 更新流程里唯一在所有机器上跑的东西 |
| `scripts/` | provision-board.sh、dev-push.sh、setup-*.sh、install.sh、robot-rescue（恢复）、board-test.sh（CI）等 |
| `docs/` | `robot/`（使用）· `design/`（为何如此）· `project/`（记录）· `ideas/`（未设计） |

---

## 8. 架构速成（10 分钟版）

### 8.1 形状

```text
   手柄            手机           你(笔记本)        远程对端        GitHub release
  BLE/USB         BLE             ssh               WebRTC          https
     │              │               │                  │              │
  ┌─padd─┐      ┌──btd──┐      ┌─robotctl─┐       ┌──mediad──┐       │
  └──┬───┘      └──┬────┘      └────┬─────┘       └────┬─────┘       │
     │             │ 同一 API 的子集                     │              │
     ▼             ▼                 ▼                  ▼              ▼
  ┌──────────────────────────────────────────────────────────────────┐
  │        每服务一个 unix socket · JSON-RPC 2.0 · 一行一个对象        │
  └────┬─────────────────────┬────────────────────┬───────────────────┘
       ▼                     ▼                    ▼
  ┌─────────┐          ┌────────────┐       ┌────────────┐
  │ robotd  │          │  configd   │       │  updaterd  │◄──── GitHub releases
  │ robot.* │          │ net.* …    │       │ update.*   │
  │ 50Hz 回路│          │ BlueZ / NM  │       │ verify/swap│
  │ 安全权威 │          │ (D-Bus)    │       │ health gate│
  └────┬────┘          └─────┬──────┘       └─────┬──────┘
       │ Dynamixel            │ D-Bus             │ systemctl restart
       ▼                      ▼                   ▼
 15 舵机 + IMU 一条 UART   BlueZ · NetworkManager  /opt/robot/daemon/current
```

（`tofd` 在旁发布深度帧：`/run/tofd/tof.sock`，`mediad`/`robotd` 订阅，它不读任何人。）

### 8.2 四条最重要的事实

1. **`robotd` 是唯一碰机器的东西。** 15 个舵机 + IMU 共用一条串口总线，50Hz 回路独占。
   所有客户端发的是 **intent**（"走这么快""看那里""站起来"），由 `robotd` 内的安全层决定可执行性。
   没有任何进程能绕过 fall 检测/关节限制/热限制。
2. **三个服务在 `robotd` 死掉时依然可用：`configd`、`updaterd`、`btd`。** 它们是恢复路径——
   控制回路起不来的机器恰恰是别人需要重配 wifi、更新或回滚的机器。所以没有对 `robotd` 的
   systemd 依赖，不加载 ML 运行时，不碰媒体栈。
3. **更新 = 整目录替换，不是打补丁。** 每个 build 落在 `/opt/robot/daemon/releases/<ver>/`，
   `current` 是指向它的符号链接。`updaterd` 验证 minisign 签名 → 换 `current` → 重启 unit →
   问 `robotd` 健康不健康 → 不健康就把旧的放回去（还有 boot counter 兜底 crash-loop）。
4. **状态单写者。** 每份状态只有一个拥有者：wifi 归 NetworkManager（我们从不存凭据）、
   名字/PIN 归 `/var/lib/robot/config/config.json`（`configd` 用 flock + rename 原子写）、
   `robotd.toml` 归 `/etc/robot/`（installer 只写一次）、二进制/策略归 release 目录。
   **`/etc/robot/` 和 `/var/lib/robot/` 之外的内容在更新和回滚后都保留。**

### 8.3 IPC 约定

- JSON-RPC 2.0，一行一个对象（NDJSON），`tokio_util::codec::LinesCodec` 分帧；订阅 = 在连接上推 notification。
- **为什么 unix socket 而不是 localhost HTTP**：0660 + 专属 group 即免费的访问控制；
  `SO_PEERCRED` 拿调用者 uid/gid 做审计和**授权**（`allow_uids`/`allow_gids` 只拦"改机器人"的调用，
  只读调用故意不拦 —— 支持里要能检查一台无权修改的机器人）；而且"把 0.0.0.0 绑错了口子"
  这类 bug 在 unix socket 上**无法表达**，那是暴露固件更新控制的惯用路径。
- **数据面不跨 socket**：`robotd` 不需要摄像头帧，要的是特征（"球在 (x,y)"）。感知跟在传感器旁
  （`mediad`），推送几十字节的特征；控制回路只读本地缓存的最新快照（last-value-wins，非阻塞）。
  帧必须跨界是最后手段（shm 环形缓冲）。
- 一个 API 定义（`duck-ipc-proto`，API v17），多个运输：BLE（子集）、unix socket、WebSocket（LLM 代理用：
  按帧取 JPEG）、WebRTC datachannel（远程遥观/遥操作，`control` 可靠 + `teleop` 不可靠各一条）。

### 8.4 日志与版本

- 所有 daemon **stderr → journald**；第一行日志是身份（`warn` 级）：版本、revision、exe 路径、pid。
- 两种记录、两种持久性：服务日志（journald，`/var/log` 是 zram，**断电即失**）vs 更新历史
  （`/var/lib/robot/updater/update-log.jsonl`，逐条 fsync，**断电也不丢**）。
- `robotctl version` 同时报告**运行中**与**已安装**且标出分歧：更新后几秒内运行版本滞后是正常设计
  （`updaterd` 不能重启自己）；`robotd` 滞后则是异常。

---

## 9. 调试速查

| 症状 | 先做什么 |
|---|---|
| 测试编译不过，缺 `libudev`/GStreamer | `apt-get install` §2.1 那四行 |
| `monitor` 打不开 / 说 "no robot" | socket 权限：是否在 `robot` 组；或用了 `--fake` 的 daemon 没起来 |
| `--ref` 报的错看起来像"烂 release" | `grep -c 'DEV BOARD' /var/lib/robot/provision.log` —— 多半 dev key 没装上 |
| 更新后 daemon 好像还是旧代码 | `robotctl version`，看运行 vs 安装；`journalctl -u <svc> -b` 看启动身份行 |
| ssh 连不上（reflash 后） | `./scripts/provision-board.sh <addr> --forget-host-key` —— host key 变了 |
| 想查某次更新的因果 | `robotctl update log` / `update list`；`robotd --unhealthy` 可以在 bench 上复现回滚 |
| 板上日志找不到了 | `/var/log` 是 zram，断电即失；持久的是 update-log |

---

## 10. 深入去哪

| 主题 | 文档 |
|---|---|
| 全部 `robotctl` 命令 | [`robot/cheatsheet.md`](robot/cheatsheet.md) |
| 笔记本蓝牙控制 | [`robot/duckctl.md`](robot/duckctl.md) |
| 开发板命令、更新陷阱 | [`robot/cheatsheet-dev.md`](robot/cheatsheet-dev.md) |
| dev-push 全参数与失败模式 | [`robot/dev-push.md`](robot/dev-push.md) |
| 总体架构（服务拆分、IPC、状态所有权、安全） | [`design/architecture.md`](design/architecture.md) |
| 控制回路（模型、观测、策略、安全） | [`design/robotd-design.md`](design/robotd-design.md) |
| 更新引擎（验证、原子换、健康门、回滚、release 格式） | [`design/updater-design.md`](design/updater-design.md) |
| 重启顺序（每条路径上谁先谁后） | [`design/restart-order.md`](design/restart-order.md) |
| 手机 App 路径（BLE + configd） | [`design/app-path-design.md`](design/app-path-design.md) |
| WebRTC 远程（信令、控制通道） | [`design/remote-webrtc.md`](design/remote-webrtc.md) |
| 里程碑与现状 | [`project/roadmap.md`](project/roadmap.md) |
| 镜像配置、信任链、日志落盘 | [`../deploy/README.md`](../deploy/README.md) |
| 规范与约定（comment 写 why、每个决策都有测试） | [`../CONTRIBUTING.md`](../CONTRIBUTING.md) |

> 设计文档的编号原则：**一页拥有一个机制，其他页引用它**。两个文档冲突时，不拥有该机制的
> 那页是 bug —— 先读拥有的那页。

# 连接手机 App 的服务端

Status: draft · Date: 2026-10-03

手机如何连上 OpenMicroDuck：没有网时用蓝牙把机器人送上局域网，已经在网上时把它找出来，然后在同一次会话里看画面、发和网页控制台相同的意图、并让机器人自己下载动作模型。

这一页描述的是 **App 的连接顺序和局域网发现**。底下每一种机制已经有其他设计文档描述，这里只写 App 用到的那一层，其余用一句话指向：


| 机制                       | 主人                                         |
| ------------------------ | ------------------------------------------ |
| 服务划分、谁拥有状态               | `[architecture.md](architecture.md)`       |
| 蓝牙 GATT、配网、PIN、广告里的名字    | `[app-path-design.md](app-path-design.md)` |
| WebRTC 信令、会话、控制通道上允许哪些调用 | `[remote-webrtc.md](remote-webrtc.md)`     |
| 浏览器控制台、以及「广告里已经有 IPv4」   | `[webrtc-console.md](webrtc-console.md)`   |
| 发布校验、组件、模型与 daemon 分开更新  | `[updater-design.md](updater-design.md)`   |


不新增守护进程。App 是现有传输的另一个客户端。`btd` 仍然什么都不拥有，`mediad` 仍然是局域网上的前门。

## 0. 现在已经实现的


|      |                                                                                                                                                             |
| ---- | ----------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 蓝牙配网 | `btd` 暴露一个 GATT 服务，把 `net.*` / `system.*` 转到 `configd`。`configd` 经 NetworkManager 扫描、连接，并能区分密码错误。Mac 上走过发现、配对、`system.info`。`net.connect` 还没在手机上跑过，链路默认也不加密 |
| 地址广播 | 广告的厂商数据（company id `0xFFFF`，4 字节 IPv4）就是当前地址。没有地址时播 `0.0.0.0`，字段缺失表示更旧的版本。`btd` 每 5 秒对一下 `net.status`，变了就重播                                                 |
| 看和控制 | `webd`（`:8090`）把 WebSocket 上的 JSON-RPC 原样转给 `robotd`。网页上的起立、行走、技能、关节和姿态都走这里。它没有视频，没有 systemd 单元，平时是人手启动的                                                    |
| 视频   | `mediad` 在 `:8443` 跑信令。摄像头经硬件编码器以 constrained-baseline H.264 送出，旁边有一条可靠有序的 `control` 数据通道，帧格式与 unix socket 上的 JSON-RPC 相同                                   |
| 动作模型 | 设计上每个网络是 `updaterd` 的一个组件，换权重用 reload 而不是重启。`deploy/updater.toml` 里这些组件是故意空着的：来源仓库还不存在。今天的 `.onnx` 打在 daemon 发布包的 `policies/` 里，随整包更换                       |




## 1. 形状

```text
  还没有局域网地址
        │
        ▼
  手机 ──BLE──▸ btd ──▸ configd ──▸ NetworkManager
        │         └──▸ updaterd   （整包更新；这次会话会断）
        │
        │  广告里的 IPv4 变成非 0，或听到下面的信标
        ▼
  手机 ──WebRTC──▸ mediad :8443
                    ├── 视频轨
                    └── control 数据通道 ──▸ robotd / configd / updaterd
                                              （动作模型走这里）
```

两次传输是先后，不是同时开车。蓝牙负责「还上不了网」和「整包更新会把 `mediad` 重启掉」这两件事。人已经能在局域网里打开会话之后，驾驶、画面和动作模型都走这一次 WebRTC，不再开第二条到 `robotd` 的管子。

`webd` 留在工作台上：舵机页和标定页要独占串口，日志页读 journal。App 不连接 `:8090`。网页控制台发的那些 `robot.*` 调用，App 改由 `control` 通道发送，方法名和参数不变。

## 2. 蓝牙：把机器人送上 Wi-Fi

机制在 `[app-path-design.md](app-path-design.md)`。App 需要知道的线协议如下。

扫 **服务 UUID** `6f5d2a10-3b47-4c8e-9a1f-2d7e8c4b6019`。名字在 scan response 里，默认 `duck-` 加序列号哈希的四位十六进制。厂商数据 company id `0xFFFF` 后面恰好 4 字节，网络序 IPv4：


| 读到的               | 含义              | App 接着做               |
| ----------------- | --------------- | --------------------- |
| 4 字节且不是 `0.0.0.0` | 已经在网上           | 进入第 4 节，不必为了地址去连 GATT |
| 4 字节且是 `0.0.0.0`  | 这台机器没有地址        | 连上，配网                 |
| 没有这个字段            | 更旧的固件，广告里还不会报地址 | 连上，问 `net.status`     |


同一特征 `6f5d2a11-3b47-4c8e-9a1f-2d7e8c4b6019`：读一次拿 API 版本，写 NDJSON，订阅通知收回答。一行一个 JSON-RPC 对象，靠换行分帧，没有长度头。重组上限 8 KiB。版本号只用来提示不一致，不用来拒绝后面的调用。

配网顺序：

1. `system.authenticate`（PIN；出厂是 `000000`，每台一样，所以它不是身份证明）。
2. `net.scan`，等这一次扫描结束再画列表。`configd` 会等到 `LastScan` 前进，App 不要自己「再扫一次看看」。
3. `net.connect`，参数是用户选的 SSID 和密码。结果看这次激活，不看网卡现在连着谁。`BadKey` 是密码错，`NotFound` 是无线电看不到这个 SSID，隐藏网络目前会被预检直接拒绝。
4. 成功之后继续听广告。地址变掉会在几秒内重播。App 以新的 IPv4 离开蓝牙，去开 WebRTC。

密码只出现在这一次 `net.connect` 里。App 不落盘、不进日志。这条链路默认仍然是明文，加密是 `[app-path-design.md](app-path-design.md)` §8.1 的未完成项；在它合上之前，配网等于在无线电射程里念出密码。

蓝牙上还可以做整包 `update.apply` 并 `update.subscribe` 看进度，因为更新会重启 `mediad`，进度不能骑在将被拆掉的那条会话上。`update.pin` 和 `update.resetToGolden` 在蓝牙上拒绝。

`net.connect` 在 WebRTC 上拒绝。已经能打开视频会话的机器人说明它有网；给它换网是蓝牙的工作。

## 3. 局域网发现

广告里的地址解决「人站在机器人旁边」。这一节解决另一件事：手机和机器人已经在同一个 Wi-Fi 上，人打开 App，不想再走一遍蓝牙扫描。

### 3.1 三个来源，固定的尝试顺序

1. **上次成功会话记下的地址。** App 保存 `system.info` 里的序列号和当时的 IPv4。下次启动先对 `ip:8443` 做一次短连接。租约没变时，这一步就够了，也不依赖广播。
2. **UDP 信标。** 第 1 步失败再听。一台机器人换了租约、或者手机第一次进这个网络但蓝牙不方便开，靠它。
3. **蓝牙广告。** 前两步都没有，或者信标被 AP 丢掉。广告不经过局域网，客户端隔离对它无效。`0.0.0.0` 则回到第 2 节。

序列号是把三次结果认成同一台机器人的键。名字会改，四位十六进制也会撞。

### 3.2 信标

`configd` 发送。它在 `robotd` 和 `mediad` 都死掉的时候还在，而那正是手机仍需要找到这台机器的时候。它已经掌握名字，也能从 NetworkManager 读到地址。

没有全局 IPv4 时不发。发了 `0.0.0.0` 对局域网没有用，没有地址这件事广告已经在说。

有地址时每秒一个数据报，发到该地址所在子网的广播地址：

```text
MDK1\n
{"v":1,"name":"duck-c51b","serial":"<soc serial>","api":16,"ip4":"192.168.1.42"}
```

`MDK1\n` 是魔数，后面是一行 JSON。整个数据报保持在一个以太网帧里。`v` 只增加、不改旧字段的含义。`api` 是 `API_VERSION`，用来在列表上提示版本不一致。

信标不是 API。上面没有密码、没有 PIN、没有控制方法。端口不放进去：信令固定是 **8443**，和「不把端口塞进 31 字节广告」是同一条约束。非默认端口只存在于开发启动参数里，由调用方另行指定。

不响应查询。手机听两秒到三秒，把听到的按 `serial` 去重。

### 3.3 为什么不是 mDNS

这块板子的镜像上，`.local` 时通时不通，`[install-dev.md](../robot/install-dev.md)` 已经把它排除在「怎么找到 IP」之外。再装一套 Avahi，等于把发现建立在已经不可靠的那条名字解析上。

家用 AP 的客户端隔离会同时丢掉 mDNS 组播和上面的 UDP 广播。所以信标是快路径，不是唯一路径；隔离打开时，顺序里的第 3 步仍然能找到机器人。

## 4. 会话：和网页同一组控制，加上画面

App 作为 **consumer** 连 `ws://<ip>:8443`，协议是 `mediad` 已经在用的 gst-webrtc 信令，表在 `[remote-webrtc.md](remote-webrtc.md)` §7。`list` 回来的 producer `meta` 里有名字和序列号，连上之前就能核对是不是刚才那一台。

一条会话里两样东西：

- **视频轨。** constrained-baseline H.264（`profile-level-id=42e01f`）。手机用系统 WebRTC 栈解码，不另做一路 MJPEG 或私有推流。质量档在机器人的 `[media]` 里，不由 App 逐项改分辨率。
- `control` **数据通道。** 可靠、有序。一行一个 JSON-RPC 对象，和 `robotctl`、`webd` 的 `/ws` 相同。

网页控制台（`webd/web/app.js`）实际在用的调用，这条通道已经全部放行：


| 调用                                   | 作用                                                            |
| ------------------------------------ | ------------------------------------------------------------- |
| `robot.subscribe` `{hz: 10}`         | 关节、姿态、模式。10 Hz 够画界面；不要订到控制环的 50 Hz                            |
| `robot.health`                       | 电量、温度、策略是否加载。订阅流里没有这些                                         |
| `robot.init`                         | 起立                                                            |
| `robot.enable` `{on, toggle: false}` | `on: true` 开始行走，`on: false` 回到站姿                              |
| `robot.move`                         | 速度意图。发出去即可，不要等这一条的应答再发下一条                                     |
| `robot.stop`                         | 速度归零。离开页面、进后台、会话关掉时发一次                                        |
| `robot.do` `{skill}`                 | `ground_pick`、`kick_left`、`kick_right`、`sit_toggle`、`roulade` |


`robot.stop` 只清速度，不卸力矩。卸力是单独的 `robot.relax`，要人明确按。按钮文案不要做成急停：断线之后真正让机器人停下来的是 `robotd` 里按时间计算的 deadman，这条通道上的 stop 只是同样的事情提前做一次。

`robot.move` 走这条可靠通道会有队头阻塞。链路差时表现为整段操作一起停顿。不可靠的 `teleop` 通道还没有作为第一版打开，见 `[remote-webrtc.md](remote-webrtc.md)` §6。

同一时刻只保留这一个写意图的会话。手柄和 App 一起开车会交错写入同一个槽，结果是谁的指令都不完整；这个缺口记在 `[remote-webrtc.md](remote-webrtc.md)` §9，本页不另做一套仲裁。

局域网会话没有额外的门禁。同一网络上能连上信令端口的人就能开车、也能看摄像头。这是 `[remote-webrtc.md](remote-webrtc.md)` §4 的决定。配对 PIN 不能拿来补这个洞：每台机器出厂都是 `000000`。

## 5. 动作模型

「下载更新」是 App 让 `updaterd` 去取它已经信任的包，不是手机把 `.onnx` 传上来。校验、签名和换入仍由 `[updater-design.md](updater-design.md)` 负责。手机上传权重会绕开那套信任，这一版不做。

每个动作网络是一个组件，彼此独立，例如 `model-walk`、`model-stand`、`model-sitstand`、`model-ground-pick`、`model-kick-left`、`model-kick-right`、`model-roulade`。对应文件今天位于发布树的 `policies/`（`alpha_walking.onnx`、`alpha_stand.onnx` 等）。组件的 `on_apply` 是对 `robotd` 的 reload，换权重时不重启 `mediad`，这条视频会话还在。

App 在 `control` 通道上使用已经放行的只读方法，外加下面要放行的写入：


| 调用                                  | 作用                                                                       |
| ----------------------------------- | ------------------------------------------------------------------------ |
| `update.listInstalled`              | 这台机器上每个组件的当前版本                                                           |
| `update.check`                      | 源上有没有更新的包                                                                |
| `update.subscribe`                  | 进度通知。没有 `id`，是一条流，要单独占一条到 `updaterd` 的连接；`mediad` 按方法分车道，App 不用自己拆socket |
| `update.apply`                      | 只对 **reload 型** 的模型组件                                                    |
| `update.rollback` / `update.select` | 回到这台机器上已经跑过的模型版本，同样只限 reload 型                                           |


`mediad` 今天拒绝全部 `update.apply`，理由是安装会重启 `mediad`、进度会话会消失。这个理由对 **daemon** 组件成立，对 reload 型的模型组件不成立。放行时按组件的 `on_apply` 区分：daemon 的 apply、rollback、select 继续拒绝，仍从蓝牙做。模型组件在 `deploy/updater.toml` 里登记之前，`check` 不应为了它们报失败——文件里已经写了为什么空着。

还有两个前提，现在都没有：

- `robotd` 还没有处理 reload 信号。`updater-design.md` 写了 SIGHUP 后重新映射权重；进程里还没有这条路径。在它落地之前，App 可以列出版本，不提供「更新模型」按钮。
- 模型组件的源（清单、签名、地址）要在第一次真正发布那个包的改动里写进 `updater.toml`，而不是先抄 example。

整包 daemon 更新会换掉当时打进去的 `policies/`，也会重启 `mediad`。那是粗路径，进度走蓝牙上的 `update.subscribe`。App 里「更新动作」和「更新系统」是两件事。

模型清单里的 `model_api` 高于本机 daemon 所实现的版本时，引擎拒绝加载。App 把这个错误显示成「先更新系统」，而不是再试一次同一个模型包。

## 6. 实施顺序

1. **App 先走已有的两段。** 蓝牙读地址或配网，然后连 `:8443`，用第 4 节的调用开车和看视频。服务端这一步没有新协议。
2. `configd` **的 UDP 信标。** 第 3.2 节。发现失败时仍能退回广告。
3. **模型组件 +** `robotd` **reload。** 两者齐了，再把 `mediad` 的放行从「所有 apply」改成「daemon 的 apply」。



## 7. 不做

- **不把 App 接到** `webd`**。** `:8090` 的 WebSocket 没有路由表，舵机和标定会在 `robotd` 停掉时自己打开串口。多一条控制通道就是第二个写意图的人。
- **不做第二套发现协议。** 不装 Avahi，不把端口或 SSID 塞进蓝牙广告。
- **不在 App 路径上做云端中继。** 人在局域网外的走法是 `[remote-webrtc.md](remote-webrtc.md)` §7，本地这一段不依赖它。
- **不在这一版给局域网会话加 PIN。** 理由在 `[remote-webrtc.md](remote-webrtc.md)` §4。
- **不由手机推送** `.onnx`**。** 包从机器人已配置的源下载。


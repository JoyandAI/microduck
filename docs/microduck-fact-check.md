# Microduck 硬件与架构事实核查

> 目的：给技术文章提供可追溯的事实边界。证据分为：**官方产品页/Press Kit**、**官方源代码**、**第三方逆向仓库**。除非特别注明，官方规格页的“KEY SPECIFICATIONS”仍标注为 *NOT FINAL YET*，量产版本可能调整。

## 事实矩阵

| claim | value | evidence tier | source link/path | confidence / caveat |
|---|---|---|---|---|
| 机器人尺寸 | 高 25 cm；宽 14 cm | 官方产品 Press Kit | [Microduck Press Kit](https://pollen-robotics.com/microduck/press-kit/) | 高；Press Kit 同时写“规格尚未最终定版”。仓库 README 只写约 25 cm。 |
| 重量 | 小于 800 g（产品页宣传“800 g to pick up”） | 官方产品 Press Kit / 产品页 | [Press Kit](https://pollen-robotics.com/microduck/press-kit/)、[产品页](https://pollen-robotics.com/microduck) | 高；不要把“under 800 g”改写成精确整机重量。 |
| 计算 SoC | Rockchip RK3566，带 AI accelerator | 官方产品 Press Kit | [Press Kit](https://pollen-robotics.com/microduck/press-kit/) | 高；Press Kit 未承诺具体主板、频率或 NPU TOPS。 |
| 内存/存储 | 1 GB RAM、32 GB storage | 官方产品 Press Kit | [Press Kit](https://pollen-robotics.com/microduck/press-kit/) | 高；介质未在该段明确为 eMMC，勿擅自写成某 SKU。 |
| 主控模块型号 | Radxa Zero 3W（RK3566） | 第三方逆向仓库；官方代码仅确认 RK3566 | [microduck-replica README.en.md](https://github.com/fanhao375/microduck-replica/blob/main/README.en.md)、[official README.md](https://github.com/pollen-robotics/microduck/blob/main/README.md) | 中/低；官方产品资料没有公布 Radxa 型号。第三方结论来自设备树/软件推理，不是官方 BOM。 |
| 执行器数量/自由度 | 15 motors / 15 DOF，腿、头颈及喙 | 官方产品 Press Kit；官方源码 NUM_JOINTS=15 | [Press Kit](https://pollen-robotics.com/microduck/press-kit/)、[model.rs](../duck-control/src/model.rs#L11-L19) | 高（数量）；官方未公开电机品牌和具体料号。 |
| 执行器具体型号 | Dynamixel XL330（第三方称 XL330-M288-T） | 第三方逆向仓库；官方源码只写 Dynamixel 总线/servos | [microduck-replica docs/硬件规格速查.md](https://github.com/fanhao375/microduck-replica/blob/main/docs/%E7%A1%AC%E4%BB%B6%E8%A7%84%E6%A0%BC%E9%80%9F%E6%9F%A5.md)、[bus.rs](../duck-control/src/bus.rs#L1-L4) | 中/低；不能在文章中当作官方确认的量产零配件。 |
| RL 动作维度 | 14 outputs；跳过 mouth 关节 | 官方源码 | [obs.rs](../duck-control/src/obs.rs#L8-L19)、[model.rs](../duck-control/src/model.rs#L28-L31) | 高（软件接口）；不与 15 个物理执行器矛盾。14 是策略输出，嘴由上层控制。 |
| 控制频率 | 50 Hz onboard policy/control loop | 官方产品页/Press Kit；官方配置注明沿用原型 | [Press Kit](https://pollen-robotics.com/microduck/press-kit/)、[robotd.toml](../deploy/robotd.toml#L27-L34) | 高（目标值）；代码注释明确称该频率继承 Raspberry Pi 原型、尚未在 Radxa 上重新推导，文章应注明。 |
| IMU 数量 | 2 个（body + head） | 官方产品 Press Kit | [Press Kit](https://pollen-robotics.com/microduck/press-kit/) | 高（官方宣称）；具体芯片型号未公开。 |
| IMU 软件实际使用情况 | 官方控制路径读取一个 IMU；**已迁移**（2026-09-04）：IMU 不再挂在舵机总线上，而是走独立 I²C 总线 `i2c4` → `/dev/i2c-imu`（LSM6DSV16X，0x6A），由单独的 `ImuIo` trait 读取；`Sensors.imu` 字段结构与 `SflpDecoder` 不变。此前源码注释为 “one IMU, one code path” | 官方源代码 | [imu.rs](../duck-control/src/imu.rs#L1-L20)、[io.rs](../duck-control/src/io.rs#L108-L120)、迁移方案 [hls2909-i2c-imu-migration.md](design/hls2909-i2c-imu-migration.md) | 高（软件行为）；与 Press Kit“2 IMUs”存在未解释差异。不能据此断言第二颗不存在，硬件是否焊接需实物证据。 |
| IMU 具体芯片 | LSM6DSV16X（第三方逆向）；BMI088 在 HAT 设备树中被标为 dormant/unused（第三方解读） | 第三方逆向仓库 | [硬件规格速查.md](https://github.com/fanhao375/microduck-replica/blob/main/docs/%E7%A1%AC%E4%BB%B6%E8%A7%84%E6%A0%BC%E9%80%9F%E6%9F%A5.md#L295-L303) | 低/中；官方没有公布芯片料号，且“第二颗物理存在”是推断，不能写成定论。 |
| ToF / LiDAR | 8×8 time-of-flight matrix | 官方产品 Press Kit；官方驱动支持两代 | [Press Kit](https://pollen-robotics.com/microduck/press-kit/)、[tof/lib.rs](../tof/src/lib.rs#L1-L11)、[tof/sensor.rs](../tof/src/sensor.rs#L8-L14) | 高（8×8）；具体是 VL53L5CX 还是 VL53L8CX 由运行时探测，官方未指定单一料号。 |
| 摄像头 | 前置 camera，带 camera-use indicator | 官方产品 Press Kit | [Press Kit](https://pollen-robotics.com/microduck/press-kit/) | 高（功能）；分辨率、视场角官方明确“仍在定稿”。 |
| 摄像头传感器 | IMX219 / Raspberry Pi Camera v2（第三方逆向） | 第三方逆向仓库；官方代码仅实现媒体管线 | [硬件规格速查.md](https://github.com/fanhao375/microduck-replica/blob/main/docs/%E7%A1%AC%E4%BB%B6%E8%A7%84%E6%A0%BC%E9%80%9F%E6%9F%A5.md)、[mediad](../mediad/src/pipeline.rs) | 低/中；不是官方公开零件清单，文章应写“逆向推测”。 |
| 音频 | microphones + speaker；每台生成独特声音 | 官方产品 Press Kit | [Press Kit](https://pollen-robotics.com/microduck/press-kit/) | 高（功能）；codec、功放和麦克风型号未公开。 |
| 音频 codec | TI TLV320AIC3104（第三方逆向） | 第三方逆向仓库；官方源码只描述音频服务 | [硬件方案逆向.md](https://github.com/fanhao375/microduck-replica/blob/main/docs/%E7%A1%AC%E4%BB%B6%E6%96%B9%E6%A1%88%E9%80%86%E5%90%91.md)、[sounds/lib.rs](../sounds/src/lib.rs) | 低；没有官方 BOM/原理图佐证，不能称“官方使用”。 |
| 电池 | 可拆卸 NP-F550 摄像机电池，2600 mAh，约 1 小时（视使用而定） | 官方产品 Press Kit | [Press Kit](https://pollen-robotics.com/microduck/press-kit/) | 高；官方未在该段给出电芯品牌或完整电气参数。 |
| NFC | 头部和喙各 1 个，共 2 天线 | 官方产品 Press Kit | [Press Kit](https://pollen-robotics.com/microduck/press-kit/) | 高；官方代码仓库当前未见 NFC 实现，硬件交互细节待发布。 |
| 软件架构 | robotd（控制/运动/安全）、mediad（相机/音频/WebRTC）、tofd（ToF）、configd、btd、padd、updaterd 等独立守护进程；Unix socket 上 JSON-RPC | 官方源代码/设计文档 | [architecture.md](architecture.md#L31-L90)、[README.md](../README.md#L25-L27) | 高（软件仓库描述）；这是软件架构，不等于硬件设计开源。 |
| 舵机总线 | 15 个舵机共用一条舵机总线（Dynamixel 2.0 / 迁移后 Feetech FT-SCS，端口 `/dev/ttyS2`）；**IMU 不在该总线上**，走 `i2c4` → `/dev/i2c-imu`（2026-09-04） | 官方源代码/配置 | [bus.rs](../duck-control/src/bus.rs#L1-L4)、[robotd.toml](../deploy/robotd.toml#L15-L19)、迁移方案 [hls2909-i2c-imu-migration.md](design/hls2909-i2c-imu-migration.md) | 高（实现）；协议、电平和 UART 映射是实现细节，不应外推为官方硬件 BOM。 |
| 开源范围 | SDK、仿真和 RL 训练软件开源；机械/电子设计文件不属于开源声明 | 官方 Press Kit | [Press Kit](https://pollen-robotics.com/microduck/press-kit/) | 高；引用文章应避免使用“开源硬件”表述。 |

## 写作时的硬边界

1. **“15 电机”与“14 维动作”必须同时解释**：第 15 个物理关节是 mouth；alpha 策略固定输出 14 维，代码通过跳过 `MOUTH_INDEX` 映射到 15 个关节。
2. **双 IMU 是官方产品说法，单 IMU 是当前软件路径事实**。在没有拆机或官方澄清前，不要写“第二颗已确认焊接且被软件弃用”；最多写“软件只读取一个，硬件配置未公开”。
3. **XL330、Radxa Zero 3W、IMX219、TLV320AIC3104、LSM6DSV16X 均不是官方 Press Kit 公布的完整 BOM**。可作为“第三方逆向/源码线索”单列，不能冒充官方零配件参数。
4. 官方 Press Kit 明确提醒相机分辨率/FOV、LiDAR 距离、无线版本和 SDK 语言仍在最终确定，文章不要补写未经证实的数值。

## 信息来源与法律声明

- Pollen Robotics 官方产品页：[pollen-robotics.com/microduck](https://pollen-robotics.com/microduck)
- Pollen Robotics 官方 Press Kit：[pollen-robotics.com/microduck/press-kit](https://pollen-robotics.com/microduck/press-kit/)
- 官方软件仓库：[pollen-robotics/microduck](https://github.com/pollen-robotics/microduck)（Apache-2.0，具体文件以仓库许可证和版权声明为准）
- 第三方复刻/逆向仓库：[fanhao375/microduck-replica](https://github.com/fanhao375/microduck-replica)（仅作线索与交叉核对，不代表 Pollen Robotics 官方立场）

本文档是技术资料整理，不构成 Pollen Robotics 的产品规格、购买建议、质量保证或商业授权。第三方逆向内容可能存在误差；转载时请保留来源、证据等级和不确定性说明，不要将推测性料号用于商业宣传或仿制承诺。

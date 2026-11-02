# 第二讲作业：从业务分析到程序 IO

依据桌面《第二讲：从业务分析到程序 IO.pdf》（22 页），实现范围为第二讲。
第三至第五讲保持教学骨架；因此主程序的正常输出应始终为 `target_state=0`。

## 实现与要求对应

| 讲义要求 | 实现位置与行为 |
| --- | --- |
| Open | `lessons/lesson02_io.cpp`：独立 ROS Context，`/training_io` 节点，两个订阅、一个发布器，单线程 Executor |
| Read | `InputBuffer`：只保留最新图像，RGB/BGR/mono/RGBA/BGRA 转独立拥有内存的 `CV_8UC3`，校验 stride、尺寸、数据长度 |
| 时间和姿态 | ROS 时间与 steady_clock 固定锚点，128 条／1 秒姿态历史，按图像采集时刻 slerp；缺少双侧采样时允许有界最近姿态 |
| Write | 绝对角 rad → deg，yaw 规整到一圈，pitch 按实际协议变号，ROS 当前时间和 `gimbal_link` |
| 安全门控 | 缺帧、输入/反馈过期、非自瞄模式、禁控、NaN/Inf、俯仰越界时发零值停止；禁火时仅允许状态 0/1 |
| Close | 发布停止，最多等待可靠订阅确认 50 ms，取消调度，关闭 Context 并释放资源；重复调用安全，支持重新 Open |
| 长期运行与退出 | 无数据 Read 空返回并短暂休眠 2 ms；主循环保留 Ctrl+C/SIGTERM，图像窗口支持 q/Esc；异常路径析构清理 |
| 状态显示 | 每秒打印协议 yaw/pitch 和 m/s 弹速，每 100 次 Read 打印循环数、有效帧数和输入状态 |

IO 状态成员放在 `io/io_context.hpp` 声明的 `Impl` 内，避免让 ROS 类型进入后续算法接口。
`io/io_state.hpp` 提供可独立验证的数据转换和缓冲状态；实际实现仍在指定的 `lesson02_io.cpp`。

## 坐标、时间及协议说明

算法使用右手 FLU：x 前、y 左、z 上；`imu_q` 将云台 FLU 向量旋转到稳定 FLU。
实际仿真器 `ros2/prelude.rs` 的基变换为 Bevy `(X,Y,Z)` → FLU `(-Z,-X,Y)`。
`ros2/plugin.rs` 反馈从 Bevy YXZ 取得 `yaw`、`pitch=-X角`、`roll=Z角`，因而：

```text
imu_q = Rz(yaw_deg) * Ry(pitch_deg) * Rx(-roll_deg)
```

上式各角在计算前换为 rad。反馈的正 pitch 表示低头；算法 `Command.pitch` 约定为向上的仰角，
故发出的消息 `pitch_deg = -Command.pitch * 180/pi`。yaw 向左为正，发送时不变号。
测试不仅检查单轴，也用独立 Bevy 旋转矩阵和基变换验证复合旋转。
FLU 坐标参见 [ROS REP-103](https://www.ros.org/reps/rep-0103.html)；自定义消息的方向以本仓库发送／接收代码为准。

ROS2 图像与反馈来自仿真器系统时钟，Open 建立一次 ROS 时间到单调时间的锚点。
返回的 `timestamp` 保留采集到当前处理之间的真实延迟，不用“收到消息的当前时间”替代采集时间。
本实现针对讲义仿真器的系统时钟，不支持未经配套修改就切换到 `/clock` 仿真时间。
系统时钟若发生大幅跳变，会拒绝异常数据，需要 Close/Open 重新建锚。

有双侧反馈且采样间隔不超过配置值时采用四元数最短弧插值；否则选择最近采样，
图像与该采样的时间差不得超过 `max_feedback_gap_ms`。没有任何可用反馈时保留一张最新图像等待下一轮，
不阻塞主循环。默认 60 ms 是宽松的通信容差，并不意味着 60 ms 的最近姿态近似具有足够瞄准精度。
后续高速运动阶段可缩小该值，并改成只接收双侧插值成功的数据。

讲义自身有版本差异：

1. 早期示意代码使用 `lesson02_io(IoStage, ...)`，而第 5 节与本地仓库均使用 `IoContext` 四个方法，按后者实现。
2. 第 4 节写 `target_distance=-1` 表示停止，第 5 节又允许未知距离填 -1。
   本地仿真器明确以 `state==0 || distance==-1` 判断停止，因此本实现未知距离填 0，
   用状态 0/1/2 表达停止／控制／发射，避免有效指令被仿真器丢弃。

订阅使用 `SensorDataQoS().keep_last(5)`，兼容仿真器图像的 reliable publisher 和反馈的 best-effort publisher。
指令发布使用 reliable、depth=5，兼容仿真器 best-effort subscription，并支持 CLI 查看和退出时的有界确认。

## 配置

`configs/simulation.yaml` 新增以下可校验字段，原相机标定不变：

```yaml
allow_fire: false
allow_control: true
max_input_age_ms: 200
max_feedback_gap_ms: 60
pitch_limit_rad: 0.785
```

`allow_control=false` 禁止所有控制；`allow_fire=false` 禁止射击。
默认俯仰限位取自仿真器 `config.toml`，若修改仿真机械限位，应同步此配置。
ROS_DOMAIN_ID 默认 89，统一入口也允许由环境显式覆盖。

## 构建和自动测试

仓库根目录执行：

```bash
bash scripts/lesson02.sh build
bash scripts/lesson02.sh test
```

统一入口优先使用系统 `/opt/ros/humble`，找不到时复用本机已存在的
`~/RM_Week1_Tools/deps/ubuntu22_humble` 隔离依赖。只设置当前进程环境，无安装和系统配置变更。
构建仍委托原 `scripts/build_training.sh`。仿真器本次没有改源代码，复用之前构建的二进制。

两项自动测试的覆盖边界：

- `lesson02_contract`：图像像素与独立内存、畸形消息、采集时间、丢帧/重复/乱序/过期/未来数据、
  ±180° 插值、实际仿真坐标、输出单位与方向、机械限位、模式与禁控/禁火等。
- `lesson02_ros_inprocess`：真实 rclcpp 发布订阅、无数据返回、Open/Close 幂等、输入转换、
  线上状态 0/1/2、Close 的停止消息和重新连接。测试使用 Domain 90；所有节点在同一进程，
  不代表独立进程 DDS 通信或真实仿真器图形验收通过。

原始日志位于 `build/lesson02_evidence/`。受限环境中 ROS 的进程内通路可能正常，
同时 UDP 报 `Error creating socket: Operation not permitted`，必须保留这个差别。

## 正常桌面联调和提交录屏

在正常 Ubuntu 桌面终端执行，先关闭旧培训程序与旧仿真器：

```bash
bash scripts/lesson02.sh demo
```

自动运行：录制真实桌面 WebM → 自动测试 → 无数据时 Ctrl+C 退出检查 →
真实仿真器与培训图像窗口 → 30 秒摆放窗口 → 45 秒操纵演示 → 采集节点／话题／QoS／单条指令 →
培训程序 Ctrl+C 和最后停止消息 → 关闭仿真器 → 展示结果 6 秒 → 完成视频封装。

首次录制推荐 `bash scripts/lesson02.sh demo --seconds 90 --prepare-seconds 45`。
左半屏保留终端，右上放仿真器，右下放独立相机。相机窗口现以 640×480 打开并可拖动缩放。
录屏终端保留培训程序真实的 Open、每秒 yaw/pitch/bullet_speed、Close 输出，并展示 CLI 原始消息。
录屏中的图像、日志与 ROS 消息均来自实际进程，没有替代画面。

请让相机窗口与终端并排可见，点击仿真器，用方向键转动云台、W/A/S/D 移动。
演示现在会复制一份临时 `config.toml` 并关闭自瞄接管，原始配置不变，直接用方向键即可。
标题含 `camera (view only; control in simulator)` 的窗口只负责显示，键盘操纵请在有场地／计分板的仿真器窗口进行。
手动单独启动仿真器时，原配置仍可能开启接管，此时按 F5 才能切到手动。
让 yaw 和 pitch 都明显变化，程序会保存真实反馈 CSV。
演示结束前先保留窗口，由脚本发送 Ctrl+C 记录 Close；平时使用时仍可用 q/Esc 退出图像窗口。
程序的 `pitch=25 deg` 不应机械地理解为“水平反馈必须为零”，讲义指出模型存在 25° 装配角。

结果在 `build/lesson02_evidence/desktop_日期时间/`：

- `lesson02-demo.webm`：真实桌面录屏，可直接提交前回看。
- `camera-first.png`、`camera-last.png`：直接从真实 ROS 图像保存的画面。
- `feedback.csv`、`commands.csv`：实时姿态／弹速／控制消息。
- `training.log`、`simulator.log`、`nodes.txt`、`topics.txt`、`image-qos.txt`、`command-once.txt`：运行及 CLI 证据。
- `summary.json`：逐项验收结果。只有所有自动项通过才标 `PASS_AUTOMATED`，还须回看视频确认画面正常。

录屏使用系统现有 GStreamer 的 VP8/WebM 插件，无须安装 FFmpeg。
X11 使用 ximagesrc；Wayland 使用系统 ScreenCast portal 和 PipeWire，运行时请在系统选择框选择整个屏幕。
这是操作系统的屏幕选择流程，不能靠事先的文字授权跳过。
API 依据 [XDG ScreenCast 文档](https://flatpak.github.io/xdg-desktop-portal/docs/doc-org.freedesktop.portal.ScreenCast.html)。
`--seconds 60` 可延长操纵演示，`--prepare-seconds 45` 设置摆窗时间；`--no-record` 只诊断且保持 PARTIAL。
若桌面、ROS 通信或 GPU 不可用，记录真实错误并返回 PARTIAL，不生成替代画面。

运行命令后录屏会自动开始，不需要再按录屏快捷键。终端显示“正在自动录屏”和倒计时，
原始高频日志写入文件；结束会清楚打印视频的绝对路径。
程序还会先等待有空间细节的相机图像，持续接近单色时保留画面并报告灰屏，不能仅凭收到图像就通过。
这个预检不能代替回看视频，也不代表底层图形问题已经修复。

2026-10-31 的 `desktop_20261031_165245` 真实演示 14 项自动检查全通过。
本次复核完整解码 649 帧、每 4 秒抽查画面：场景和视角变化可见，但终端长期被窗口遮住，
不建议直接把该视频作为最终提交。新版本改善窗口尺寸、终端输出和收尾展示，已构建并通过两项测试；
新版的完整桌面录屏仍需按教程实际完成。旧视频只证明旧版实跑，不能证明这些新显示改动已通过桌面验收。

2026-11-01 的灰屏复现有明确的不同线索：`desktop_20261101_001032/simulator.log`
中相机启动 1 秒高度为 -3.395 m，之后持续下坠，最后低于 -63000 m；
这说明相机随观察机器人掉到了场地下方。`setup.rs` 现在给训练观察位建立立即可用的
3.2×0.08×3.2 m 静态支撑碰撞地板，避免异步场景碰撞体尚未就绪时机器人穿过地面。
该修复只作用于训练模式，保留重力、云台操纵和原场景资源。
演示脚本持续读取真实 `CAMERA_STATE`；相机高度低于 -2 m 或无效时立即停止验收、
保留最新实际图像并报告 PARTIAL；最终检查包含 `camera_stayed_above_ground`。
修复前日志、补丁、二进制／库备份，以及多次真实启动和 150 秒场景回归检查记录保留在
`build/lesson02_evidence/grayfix_20261101/`。完整键盘操作录屏仍按上述流程实际完成。

2026-10-29 两次正常桌面运行已产生约 59 秒视频，图像／反馈／指令的跨进程通信和 Close 停止消息成功。
但 yaw/pitch 未变化，第二次保存的相机图像接近单色，因此整体验收仍为 PARTIAL。
后续运行会额外保存实际 Vulkan adapter 与相机位置诊断，供持续灰屏时定位。

手工分终端联调：

```bash
# 终端 1
bash scripts/lesson02.sh simulator
# 终端 2
bash scripts/lesson02.sh training
# 终端 3
bash scripts/lesson02.sh inspect
```

`inspect` 的前几项输出后再等待指令。额外观察频率时，无需手动 source，使用统一入口：

```bash
bash scripts/lesson02.sh ros topic hz /image_raw
bash scripts/lesson02.sh ros topic hz /vision_receive_data
bash scripts/lesson02.sh ros topic echo /vision_receive_data --once --qos-reliability best_effort
```

实际频率以硬件和负载为准，讲义约 35 Hz 是观察参考，不是由脚本固定制造的结果。

## 讲义思考题

1. **最终输出和必要输入**：输出为是否控制、是否发射及绝对 yaw/pitch；
   动态输入为图像、采集时间、同一时刻姿态和 m/s 弹速；初始化读取相机内参、畸变与相机到云台外参。
   检测精度是中间指标，业务目标是提高有效命中与伤害输出效率。
2. **2 rad/s 转动、30 ms 姿态误差**：角误差 `2×0.030=0.060 rad≈3.438°`；
   距离 3 m 的平面横向偏移 `3×tan(0.060)≈0.1802 m=18.02 cm`，小角近似为 18 cm。
   这说明不能把处理结束时姿态冒充图像采集时姿态。
3. **移动目标和延迟**：1.5 m/s 横移、30 ms 延迟，位移为 4.5 cm。
   吞吐量与端到端延迟应分别测量；高 FPS 本身不能保证低延迟。
4. **可靠传输、保留全部历史且处理跟不上**：若发送率为 λ、处理率为 μ 且 λ>μ，
   队列约以 `(λ−μ)t` 增长，内存与等待延迟持续增加，最终可能耗尽资源。
   传感器数据应保留少量最新帧，控制／参数等必须送达的消息另行选择可靠策略。
5. **最近姿态近似误差**：采样相差 Δt、角速度为 ω 时，角误差约为 `ωΔt`，
   目标平面偏差约 `distance×tan(ωΔt)`。有双侧采样的 slerp 对平滑旋转更准确，
   但不能修复错误的采集时间、时钟跳变或大幅非匀速运动。

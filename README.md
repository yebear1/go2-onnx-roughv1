# Go2 局域网手机控制器

横屏手机网页把 Go2 前置相机图传铺满全屏，透明双摇杆悬浮在画面左下和右下。控制指令通过 WebSocket 送到局域网桥接进程。除了宇树官方 Go2 Sport API、`cmd_vel` 和 `sensor_msgs/Joy`，现在也支持树莓派直接订阅 DDS LowState、运行固定 V1 平地策略。策略后端默认只做影子推理，不发布电机指令。

## 数据链路

```text
Go2 前置相机 ── VideoClient/JPEG ──> FastAPI MJPEG ──> 手机全屏图传
手机透明摇杆 ── WebSocket / 20 Hz ──> FastAPI ── ROS 2 / DDS ──> Go2
手机 HUD      <── WebSocket / 5 Hz ─── lf/sportmodestate + lf/lowstate
```

V1 策略后端的数据链路：

```text
Go2 rt/lowstate ──> 树莓派 45维观测 ──> V1 ONNX / 50 Hz ──> 12关节安全适配 ──> rt/lowcmd
手机双摇杆 ───────> WebSocket ────────> vx / vy / yaw ────────┘
```

安全措施：长按 1.2 秒解锁、单控制端占用、350 ms 心跳看门狗、摇杆松手归零、切后台/断开自动停车、服务端限速和“安全停车”按钮。网页端不提供断力硬急停；真正的硬急停仍由实体遥控器或运控系统负责。

网页“安全停车”只执行：速度归零、Sport 模式发布 `StopMove 1003`、锁定网页控制。它绝不发布 `Damp 1001`、电机断力或运控硬急停通道，因此正常情况下会保持机器人运控使能，而不是让机器狗直接失力倒下。

## 1. 机器人侧准备

在 Ubuntu 22.04 + ROS 2 Humble 的控制电脑/伴随计算机安装并编译官方 `unitree_ros2`，确保以下命令能看到话题：

```bash
source /opt/ros/humble/setup.bash
source /path/to/unitree_ros2/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 topic list
```

官方直连示例通常把有线网卡设为 `192.168.123.99/24`，并在 `CYCLONEDDS_URI` 中选中接 Go2 的网卡。实际网卡名以 `ip link` 为准。先验证能收到 `lf/sportmodestate` 或 `sportmodestate`，不要在网页阶段才排查 DDS 网络。

## 2. 安装与启动

为了让虚拟环境继续看见 ROS 的 `rclpy` 和 Unitree 消息包，要加 `--system-site-packages`：

```bash
cd /path/to/go2-web-controller
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install -r requirements.txt

# 在官方 unitree_sdk2_python 仓库克隆完成后安装
pip install -e /path/to/unitree_sdk2_python

export GO2_BACKEND=sport
export GO2_PIN=change-me
export GO2_VIDEO_MODE=sdk
export GO2_NET_IFACE=enp3s0
export UNITREE_ROS2_SETUP=/path/to/unitree_ros2/setup.sh
./run.sh
```

若 `setup.sh` 里已经 source 了 ROS 2 和工作区，`UNITREE_ROS2_SETUP` 就够了。否则先在当前终端手动 source，再运行 `./run.sh`。

手机与控制电脑连同一可信局域网，用手机打开：

```text
http://<控制电脑局域网IP>:8765
```

页面要求手机横屏；右上角按钮可尝试进入全屏。`GO2_NET_IFACE` 要换成实际连接 Go2 的网卡名。

先让 Go2 离地或置于足够空旷的地面，保持原厂遥控器和实体急停随手可用；把速度限制保持在低值完成方向验证。

## 3. 图传模式

使用宇树官方前置相机接口：

```bash
export GO2_VIDEO_MODE=sdk
export GO2_NET_IFACE=enp3s0
```

桥接通过官方 `unitree_sdk2py.go2.video.VideoClient.GetImageSample()` 取得 JPEG，并从 `/video` 输出手机浏览器能显示的 MJPEG。若已有浏览器可播放的 MJPEG 地址，也可以使用：

```bash
export GO2_VIDEO_MODE=url
export GO2_VIDEO_URL=http://<视频服务IP>:<端口>/video
```

无机器人预览使用 `GO2_VIDEO_MODE=mock`。RTSP 地址不能直接放进普通手机浏览器，需先转成 WebRTC、HLS 或 MJPEG。

## 4. 后端模式

### 树莓派 V1 平地策略（默认影子模式）

```bash
export GO2_VIDEO_MODE=off
export GO2_NET_IFACE=eth0
./run_policy_web.sh
```

该启动器强制使用 `GO2_BACKEND=policy`，但不会设置任何低层发送门槛，因此不会创建 `rt/lowcmd` 发布器。连接机器狗后，网页摇杆会驱动 V1 的 45 维观测输入，策略输出仅在内存中经过限位和斜率检查。健康状态可在 `/health` 和网页 HUD 查看。

真实低层发送仅用于后续悬空测试；必须先验证只读话题、高层 SportClient 和固件 MotionSwitcher。所需三重门槛见 `.env.example`，不能把它们写进常驻启动脚本或 systemd 服务。

### 官方 Sport API（推荐）

```bash
export GO2_BACKEND=sport
```

桥接发布 `/api/sport/request` 的 `unitree_api/msg/Request`：移动 `1008`、停止 `1003`、站立 `1004`、趴下 `1005`、恢复站立 `1006`、坐下 `1009`。这是官方 `unitree_ros2` 的高层控制路径。

### 标准 Twist

如果已有节点订阅 `geometry_msgs/msg/Twist`：

```bash
export GO2_BACKEND=twist
export GO2_CMD_VEL_TOPIC=/cmd_vel
```

此模式只提供移动/停止，快捷姿态动作会被禁用。

### Joy / SBUS 驱动适配

如果现有 SBUS ROS 驱动订阅 `sensor_msgs/msg/Joy`：

```bash
export GO2_BACKEND=joy
export GO2_JOY_TOPIC=/joy
export GO2_JOY_FORWARD_AXIS=1
export GO2_JOY_LATERAL_AXIS=0
export GO2_JOY_YAW_AXIS=3
```

轴号必须与该驱动的配置一致；用 `ros2 topic echo /joy` 对照原手柄确认正负方向。Joy 模式不猜测厂商按钮映射，因此姿态动作默认关闭。

如果驱动必须持续保持某个按钮才能维持运控使能，可配置：

```bash
export GO2_JOY_BUTTON_COUNT=12
export GO2_JOY_ENABLE_BUTTON=4   # 示例，必须换成实测索引
export GO2_JOY_ENABLE_VALUE=1
```

安全停车时三个运动轴回零，但这个运控使能按钮会继续保持。不要把它设置成硬急停、Damp 或电机失能按钮。若 SBUS 节点使用自定义消息，把 `ros2 topic info -v <topic>`、消息定义以及手柄中位/使能数据发来，再增加精确适配器。

## 5. 话题名与速度限制

不同固件/命名空间可能使用无 `lf/` 前缀的话题：

```bash
export GO2_SPORT_STATE_TOPIC=sportmodestate
export GO2_LOW_STATE_TOPIC=lowstate
```

首轮测试建议：

```bash
export GO2_MAX_FORWARD=0.35
export GO2_MAX_LATERAL=0.20
export GO2_MAX_YAW=0.55
export GO2_WATCHDOG_MS=350
```

## 6. 无机器人预览

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
GO2_MOCK=1 GO2_VIDEO_MODE=mock GO2_PIN=change-me ./run.sh
```

浏览器访问 `http://127.0.0.1:8765`。健康检查在 `/health`。

## 上线前检查

- 手机与桥接电脑使用独立、可信的机器人 Wi‑Fi/VLAN；PIN 只是局域网误操作防护，不是加密。
- 防火墙只允许机器人控制网段访问 8765；不要把端口映射到公网。
- 原厂遥控器/实体硬急停已配对并在手边；网页只做安全停车，不替代断力硬急停。
- 先悬空或低速验证前后、左右、旋转的方向，再落地。
- 切后台、关屏、断 Wi‑Fi 后，确认 350 ms 内停止并锁定。
- 确认同一时刻只有一个高层运动控制源，避免原厂 App、遥控器和网页互相争抢。

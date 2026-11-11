# 树莓派平地模型运行时

这一目录只负责 ONNX 推理与数据布局，不包含 DDS/ROS、网络连接或电机指令。默认运行任何脚本都不会连接 Go2。

当前固定模型是 `spacemit-robotics/unitree-go2` 历史提交 `ec9a4fe1059b` 记录的 V1 平地模型：

- 文件：`models/go2_blind_rough_v1.onnx`
- 大小：`758549` 字节
- SHA-256：`dac692d44da6ecef5aa10ab1007217ed60c150a0b27f28b6d392ad1be20c4e30`
- 输入：45 维 `[ang_vel, gravity, command, dof_pos, dof_vel, last_action]`
- 输出：12 维策略动作
- 控制周期：20 ms（50 Hz）

GitHub 主分支后来改成 V2，要求哈希 `f2fd66...`，但公开 Release 资产目前不存在；外部模型归档仍返回上述 V1。因此这里明确固定为 V1，不把旧权重伪装成 V2。

离线自检：

```bash
cd ~/go2-web-controller
.venv-policy/bin/python -m policy_runtime.selftest
.venv-policy/bin/python -m policy_runtime.model_inventory
```

`model_inventory` 还会逐个校验并测试综合地形、招手、坐下和前跳模型。它只用零输入检查 ONNX 文件能否在树莓派加载和推理，不代表这些动作已经具备真实机器人运行条件。

## 分阶段接入原厂主控

只读话题探测（不会创建发布者）：

```bash
./run_sdk.sh -m go2_dds.read_only_probe --interface eth0 --seconds 15
```

平地模型影子推理（订阅状态、记录输出，但不发布控制）：

```bash
./run_sdk.sh -m go2_dds.shadow_walk --interface eth0 --topic rt/lowstate --seconds 30
```

`go2_dds.sport_safe_test` 是后续高层移动验证入口，默认锁定。只有机器狗在空旷场地、实体遥控器在手且用户明确确认后，才能同时提供发送开关和确认短语。

`go2_dds.policy_controller` 已实现 V1 的 LowState→ONNX→LowCmd 适配，包括 20 电机槽初始化、12 关节顺序、CRC、关节限位、每帧目标斜率限制、状态超时、命令看门狗和连续有效状态预热。默认不会构造发布器。网页影子运行：

```bash
GO2_VIDEO_MODE=off ./run_policy_web.sh
```

此时手机摇杆的 `vx/vy/yaw` 已送进 V1 模型，但状态页必须显示“V1 影子推理”，`publisher_created=false`、`lowcmd_writes=0`。低层发送需要三个独立启动门槛，具体值保存在 `.env.example`；这些门槛只能在机器狗可靠悬空、实体遥控器和物理急停就绪后临时提供。网页解锁本身无权开启低层发送。

低层发布器的存在不等于原厂运动服务已经退出。接狗后仍必须先完成只读话题探测和高层 SportClient 测试，再单独确认固件的 MotionSwitcher 行为；程序不会自动抢占或释放原厂运控模式。

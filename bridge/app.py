from __future__ import annotations

import asyncio
import json
import math
import os
import threading
import time
from contextlib import asynccontextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, RedirectResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles


ROOT = Path(__file__).resolve().parents[1]
WEB_DIR = ROOT / "web"


def env_float(name: str, default: float) -> float:
    try:
        return float(os.getenv(name, str(default)))
    except ValueError:
        return default


def clamp(value: Any, low: float, high: float) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return 0.0
    if not math.isfinite(number):
        return 0.0
    return max(low, min(high, number))


@dataclass(frozen=True)
class Config:
    backend: str = os.getenv("GO2_BACKEND", "sport").lower()
    mock: bool = os.getenv("GO2_MOCK", "0") == "1"
    pin: str = os.getenv("GO2_PIN", "")
    watchdog_ms: int = int(os.getenv("GO2_WATCHDOG_MS", "350"))
    max_forward: float = env_float("GO2_MAX_FORWARD", 0.8)
    max_lateral: float = env_float("GO2_MAX_LATERAL", 0.45)
    max_yaw: float = env_float("GO2_MAX_YAW", 1.2)
    cmd_vel_topic: str = os.getenv("GO2_CMD_VEL_TOPIC", "/cmd_vel")
    joy_topic: str = os.getenv("GO2_JOY_TOPIC", "/joy")
    sport_state_topic: str = os.getenv("GO2_SPORT_STATE_TOPIC", "lf/sportmodestate")
    low_state_topic: str = os.getenv("GO2_LOW_STATE_TOPIC", "lf/lowstate")
    joy_forward_axis: int = int(os.getenv("GO2_JOY_FORWARD_AXIS", "1"))
    joy_lateral_axis: int = int(os.getenv("GO2_JOY_LATERAL_AXIS", "0"))
    joy_yaw_axis: int = int(os.getenv("GO2_JOY_YAW_AXIS", "3"))
    joy_axis_count: int = int(os.getenv("GO2_JOY_AXIS_COUNT", "8"))
    joy_button_count: int = int(os.getenv("GO2_JOY_BUTTON_COUNT", "12"))
    joy_enable_button: int = int(os.getenv("GO2_JOY_ENABLE_BUTTON", "-1"))
    joy_enable_value: int = int(os.getenv("GO2_JOY_ENABLE_VALUE", "1"))
    video_mode: str = os.getenv("GO2_VIDEO_MODE", "mock" if os.getenv("GO2_MOCK", "0") == "1" else "sdk").lower()
    video_url: str = os.getenv("GO2_VIDEO_URL", "")
    net_iface: str = os.getenv("GO2_NET_IFACE", "")
    dds_domain: int = int(os.getenv("GO2_DDS_DOMAIN", "0"))
    dds_low_state_topic: str = os.getenv("GO2_DDS_LOW_STATE_TOPIC", "rt/lowstate")
    dds_low_cmd_topic: str = os.getenv("GO2_DDS_LOW_CMD_TOPIC", "rt/lowcmd")
    policy_model: str = os.getenv(
        "GO2_POLICY_MODEL",
        str(ROOT / "policy_runtime" / "models" / "go2_blind_rough_v1.onnx"),
    )
    lowlevel_enable: bool = os.getenv("GO2_LOWLEVEL_ENABLE", "0") == "1"
    lowlevel_test_mode: str = os.getenv("GO2_LOWLEVEL_TEST_MODE", "")
    lowlevel_confirmation: str = os.getenv("GO2_LOWLEVEL_CONFIRM", "")


CONFIG = Config()


class RosAdapter:
    API_IDS = {
        "balance_stand": 1002,
        "stop": 1003,
        "stand_up": 1004,
        "stand_down": 1005,
        "recovery_stand": 1006,
        "sit": 1009,
        "rise_sit": 1010,
    }

    def __init__(self, config: Config):
        self.config = config
        self.mock = config.mock
        self.node = None
        self.publisher = None
        self.spin_thread: threading.Thread | None = None
        self._shutdown = threading.Event()
        self._publish_lock = threading.Lock()
        self.telemetry: dict[str, Any] = {
            "mode": "待机",
            "mode_id": 0,
            "gait": "idle",
            "battery": None,
            "voltage": None,
            "velocity": [0.0, 0.0, 0.0],
            "rpy": [0.0, 0.0, 0.0],
            "body_height": 0.0,
            "robot_seen": False,
        }

    def start(self) -> None:
        if self.mock:
            return
        try:
            import rclpy
            from geometry_msgs.msg import Twist
            from sensor_msgs.msg import Joy
            from unitree_api.msg import Request
            from unitree_go.msg import LowState, SportModeState
        except ImportError as exc:
            raise RuntimeError(
                "ROS 2/Unitree Python 消息包不可用。请先 source ROS 2 与 unitree_ros2，"
                "或设置 GO2_MOCK=1 预览。"
            ) from exc

        self._rclpy = rclpy
        self._Request = Request
        self._Twist = Twist
        self._Joy = Joy
        rclpy.init(args=None)
        self.node = rclpy.create_node("go2_web_controller")

        if self.config.backend == "sport":
            self.publisher = self.node.create_publisher(Request, "/api/sport/request", 10)
        elif self.config.backend == "twist":
            self.publisher = self.node.create_publisher(Twist, self.config.cmd_vel_topic, 10)
        elif self.config.backend == "joy":
            self.publisher = self.node.create_publisher(Joy, self.config.joy_topic, 10)
        else:
            raise RuntimeError("GO2_BACKEND 仅支持 sport、twist 或 joy")

        self.node.create_subscription(
            SportModeState, self.config.sport_state_topic, self._on_sport_state, 5
        )
        self.node.create_subscription(
            LowState, self.config.low_state_topic, self._on_low_state, 5
        )
        self.spin_thread = threading.Thread(target=self._spin, name="go2-ros-spin", daemon=True)
        self.spin_thread.start()

    def _spin(self) -> None:
        while not self._shutdown.is_set():
            self._rclpy.spin_once(self.node, timeout_sec=0.1)

    def _on_sport_state(self, msg: Any) -> None:
        mode_names = {
            0: "待机", 1: "平衡站立", 2: "姿态", 3: "行走", 5: "趴下",
            6: "关节锁定", 7: "阻尼", 8: "恢复站立", 10: "坐下",
        }
        gait_names = {0: "idle", 1: "trot", 2: "run", 3: "stairs", 4: "downstairs", 9: "adjust"}
        self.telemetry.update({
            "mode": mode_names.get(int(msg.mode), f"模式 {msg.mode}"),
            "mode_id": int(msg.mode),
            "gait": gait_names.get(int(msg.gait_type), str(msg.gait_type)),
            "velocity": [round(float(v), 3) for v in msg.velocity],
            "rpy": [round(float(v), 3) for v in msg.imu_state.rpy],
            "body_height": round(float(msg.body_height), 3),
            "robot_seen": True,
        })

    def _on_low_state(self, msg: Any) -> None:
        self.telemetry.update({
            "battery": int(msg.bms_state.soc),
            "voltage": round(float(msg.power_v), 1),
            "robot_seen": True,
        })

    def move(self, forward: float, lateral: float, yaw: float) -> None:
        if self.mock:
            self.telemetry.update({
                "velocity": [round(forward, 3), round(lateral, 3), round(yaw, 3)],
                "mode": "模拟行走" if abs(forward) + abs(lateral) + abs(yaw) > 0.01 else "模拟待机",
                "mode_id": 3 if abs(forward) + abs(lateral) + abs(yaw) > 0.01 else 0,
                "robot_seen": True,
            })
            return
        with self._publish_lock:
            if self.config.backend == "sport":
                msg = self._Request()
                msg.header.identity.api_id = 1008
                msg.parameter = json.dumps({"x": forward, "y": lateral, "z": yaw}, separators=(",", ":"))
            elif self.config.backend == "twist":
                msg = self._Twist()
                msg.linear.x = forward
                msg.linear.y = lateral
                msg.angular.z = yaw
            else:
                msg = self._Joy()
                msg.axes = [0.0] * max(4, self.config.joy_axis_count)
                msg.axes[self.config.joy_forward_axis] = forward / self.config.max_forward
                msg.axes[self.config.joy_lateral_axis] = lateral / self.config.max_lateral
                msg.axes[self.config.joy_yaw_axis] = yaw / self.config.max_yaw
                msg.buttons = [0] * max(0, self.config.joy_button_count)
                if 0 <= self.config.joy_enable_button < len(msg.buttons):
                    msg.buttons[self.config.joy_enable_button] = self.config.joy_enable_value
            self.publisher.publish(msg)

    def action(self, action: str) -> bool:
        if action not in self.API_IDS:
            return False
        if self.mock:
            labels = {
                "balance_stand": "平衡站立", "stop": "已安全停车",
                "stand_up": "站立", "stand_down": "趴下", "recovery_stand": "恢复站立",
                "sit": "坐下", "rise_sit": "起身",
            }
            self.telemetry["mode"] = labels[action]
            self.telemetry["robot_seen"] = True
            return True
        if self.config.backend != "sport":
            return False
        with self._publish_lock:
            msg = self._Request()
            msg.header.identity.api_id = self.API_IDS[action]
            self.publisher.publish(msg)
        return True

    def controlled_stop(self) -> None:
        """Stop locomotion while keeping the motion controller powered.

        This deliberately never publishes Damp (1001), motor-off, or a hard
        emergency-stop channel. Hardware emergency stop remains independent.
        """
        self.move(0.0, 0.0, 0.0)
        if self.config.backend == "sport":
            self.action("stop")

    def close(self) -> None:
        try:
            self.controlled_stop()
        except Exception:
            pass
        self._shutdown.set()
        if self.spin_thread:
            self.spin_thread.join(timeout=1.0)
        if self.node is not None:
            self.node.destroy_node()
            self._rclpy.shutdown()


class DdsPolicyAdapter:
    """Feed web velocity commands into the V1 policy on Unitree DDS.

    This adapter remains shadow-only unless the independent low-level startup
    gates are supplied. Merely unlocking the browser never creates a publisher.
    """

    def __init__(self, config: Config):
        self.config = config
        self.mock = False
        self.controller = None
        self.telemetry: dict[str, Any] = {
            "mode": "V1 影子推理",
            "mode_id": 0,
            "gait": "policy_v1",
            "battery": None,
            "voltage": None,
            "velocity": [0.0, 0.0, 0.0],
            "rpy": [0.0, 0.0, 0.0],
            "body_height": 0.0,
            "robot_seen": False,
            "publisher_created": False,
            "policy_frames": 0,
            "lowcmd_writes": 0,
            "fault": "",
        }

    def start(self) -> None:
        from go2_dds.policy_controller import PolicyController

        interface = self.config.net_iface or "eth0"
        self.controller = PolicyController(
            interface=interface,
            domain=self.config.dds_domain,
            state_topic=self.config.dds_low_state_topic,
            command_topic=self.config.dds_low_cmd_topic,
            model=self.config.policy_model,
            enable_send=self.config.lowlevel_enable,
            test_mode=self.config.lowlevel_test_mode,
            confirmation=self.config.lowlevel_confirmation,
            status_callback=self._on_status,
        )
        self.controller.start()

    def _on_status(self, status: dict[str, Any]) -> None:
        velocity = status.get("velocity_command", [0.0, 0.0, 0.0])
        publisher_created = bool(status.get("publisher_created"))
        fault = str(status.get("fault", ""))
        if fault:
            mode = "策略保护锁定"
        elif publisher_created:
            mode = "V1 悬空测试"
        else:
            mode = "V1 影子推理"
        self.telemetry.update({
            "mode": mode,
            "mode_id": 3 if publisher_created else 0,
            "battery": status.get("battery"),
            "voltage": status.get("voltage"),
            "velocity": [round(float(value), 3) for value in velocity],
            "robot_seen": bool(status.get("robot_seen")),
            "publisher_created": publisher_created,
            "policy_frames": int(status.get("frames", 0)),
            "lowcmd_writes": int(status.get("writes", 0)),
            "fault": fault,
        })

    def move(self, forward: float, lateral: float, yaw: float) -> None:
        if self.controller is None:
            return
        command = (
            clamp(forward, -1.0, 1.0),
            clamp(lateral, -0.4, 0.4),
            clamp(yaw, -1.0, 1.0),
        )
        self.controller.set_command(*command)
        self.telemetry["velocity"] = [round(value, 3) for value in command]

    def action(self, action: str) -> bool:
        return False

    def controlled_stop(self) -> None:
        if self.controller is not None:
            self.controller.stop_command()
        self.telemetry["velocity"] = [0.0, 0.0, 0.0]

    def close(self) -> None:
        if self.controller is not None:
            self.controller.close()
            self.controller = None


class VideoProvider:
    """Turn official Go2 front-camera JPEG samples into a browser MJPEG feed."""

    def __init__(self, config: Config):
        self.config = config
        self.mode = config.video_mode
        self.client = None
        self.ready = self.mode in ("mock", "url")
        self.error = ""
        self.frame_count = 0
        self.last_frame_at = 0.0
        self._lock = threading.Lock()
        self._shutdown = threading.Event()

    def start(self) -> None:
        if self.mode in ("mock", "url", "off"):
            return
        if self.mode != "sdk":
            self.error = "GO2_VIDEO_MODE 仅支持 sdk、url、mock 或 off"
            return
        try:
            from unitree_sdk2py.core.channel import ChannelFactoryInitialize
            from unitree_sdk2py.go2.video.video_client import VideoClient

            if self.config.net_iface:
                ChannelFactoryInitialize(0, self.config.net_iface)
            else:
                ChannelFactoryInitialize(0)
            self.client = VideoClient()
            self.client.SetTimeout(3.0)
            self.client.Init()
            self.ready = True
        except Exception as exc:
            self.error = f"相机 SDK 初始化失败: {exc}"
            self.ready = False

    def status(self) -> dict[str, Any]:
        age_ms = None
        if self.last_frame_at:
            age_ms = int((time.monotonic() - self.last_frame_at) * 1000)
        return {
            "mode": self.mode,
            "ready": self.ready,
            "error": self.error,
            "frames": self.frame_count,
            "frame_age_ms": age_ms,
        }

    def mjpeg_frames(self):
        while not self._shutdown.is_set():
            if not self.client:
                time.sleep(0.2)
                continue
            try:
                with self._lock:
                    code, data = self.client.GetImageSample()
                if code != 0 or not data:
                    self.error = f"GetImageSample 返回错误码 {code}"
                    time.sleep(0.08)
                    continue
                frame = bytes(data)
                self.frame_count += 1
                self.last_frame_at = time.monotonic()
                self.error = ""
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    b"Cache-Control: no-store\r\n\r\n" + frame + b"\r\n"
                )
            except GeneratorExit:
                break
            except Exception as exc:
                self.error = f"读取相机失败: {exc}"
                time.sleep(0.15)

    def close(self) -> None:
        self._shutdown.set()


class ControlHub:
    def __init__(self, adapter: RosAdapter, video: VideoProvider, config: Config):
        self.adapter = adapter
        self.video = video
        self.config = config
        self.clients: set[WebSocket] = set()
        self.owner: WebSocket | None = None
        self.armed = False
        self.last_command = 0.0
        self._watchdog_task: asyncio.Task | None = None
        self._telemetry_task: asyncio.Task | None = None

    def capabilities(self) -> dict[str, Any]:
        return {
            "type": "hello",
            "backend": self.config.backend,
            "mock": self.config.mock,
            "pin_required": bool(self.config.pin),
            "actions": self.config.backend == "sport" or self.config.mock,
            "limits": {
                "forward": self.config.max_forward,
                "lateral": self.config.max_lateral,
                "yaw": self.config.max_yaw,
            },
            "watchdog_ms": self.config.watchdog_ms,
            "video": self.video.status(),
        }

    async def start(self) -> None:
        self._watchdog_task = asyncio.create_task(self._watchdog())
        self._telemetry_task = asyncio.create_task(self._send_telemetry())

    async def close(self) -> None:
        for task in (self._watchdog_task, self._telemetry_task):
            if task:
                task.cancel()
        self.adapter.close()

    async def broadcast(self, payload: dict[str, Any]) -> None:
        stale: list[WebSocket] = []
        message = json.dumps(payload, ensure_ascii=False)
        for client in tuple(self.clients):
            try:
                await client.send_text(message)
            except Exception:
                stale.append(client)
        for client in stale:
            self.clients.discard(client)

    async def safe_stop(self, reason: str) -> None:
        was_active = self.armed or self.owner is not None
        self.armed = False
        self.owner = None
        self.adapter.controlled_stop()
        if was_active:
            await self.broadcast({"type": "armed", "armed": False, "reason": reason})

    async def _watchdog(self) -> None:
        while True:
            await asyncio.sleep(0.05)
            if self.armed and (time.monotonic() - self.last_command) * 1000 > self.config.watchdog_ms:
                await self.safe_stop("控制心跳超时，已安全停车并保持运控使能")

    async def _send_telemetry(self) -> None:
        while True:
            await asyncio.sleep(0.2)
            await self.broadcast({
                "type": "telemetry",
                "armed": self.armed,
                "controller_busy": self.owner is not None,
                "data": self.adapter.telemetry,
                "video": self.video.status(),
                "server_time": int(time.time() * 1000),
            })

    async def handle(self, ws: WebSocket) -> None:
        await ws.accept()
        self.clients.add(ws)
        await ws.send_json(self.capabilities())
        authenticated = not bool(self.config.pin)
        try:
            while True:
                raw = await ws.receive_text()
                try:
                    message = json.loads(raw)
                except json.JSONDecodeError:
                    continue
                kind = message.get("type")

                if kind == "auth":
                    authenticated = not self.config.pin or str(message.get("pin", "")) == self.config.pin
                    await ws.send_json({"type": "auth", "ok": authenticated})
                    continue
                if not authenticated:
                    await ws.send_json({"type": "error", "message": "请先输入控制 PIN"})
                    continue

                if kind == "arm":
                    if self.owner not in (None, ws):
                        await ws.send_json({"type": "error", "message": "另一台设备正在控制"})
                        continue
                    self.owner = ws
                    self.armed = True
                    self.last_command = time.monotonic()
                    await self.broadcast({"type": "armed", "armed": True})
                elif kind == "disarm":
                    if self.owner in (None, ws):
                        await self.safe_stop("已安全停车并锁定网页控制")
                elif kind in ("drive", "heartbeat"):
                    if self.owner is ws and self.armed:
                        self.last_command = time.monotonic()
                        if kind == "drive":
                            forward = clamp(message.get("forward"), -1, 1) * self.config.max_forward
                            lateral = clamp(message.get("lateral"), -1, 1) * self.config.max_lateral
                            yaw = clamp(message.get("yaw"), -1, 1) * self.config.max_yaw
                            self.adapter.move(forward, lateral, yaw)
                elif kind == "action":
                    if self.owner is not ws or not self.armed:
                        await ws.send_json({"type": "error", "message": "请先长按解锁"})
                        continue
                    action = str(message.get("action", ""))
                    if action == "stop":
                        self.adapter.controlled_stop()
                    elif not self.adapter.action(action):
                        await ws.send_json({"type": "error", "message": "当前后端不支持此动作"})
                elif kind == "safe_stop":
                    await self.safe_stop("已安全停车：速度归零、保持运控使能")
        except WebSocketDisconnect:
            pass
        finally:
            self.clients.discard(ws)
            if self.owner is ws:
                await self.safe_stop("控制端断开，已安全停车并保持运控使能")


adapter = DdsPolicyAdapter(CONFIG) if CONFIG.backend == "policy" else RosAdapter(CONFIG)
video = VideoProvider(CONFIG)
hub = ControlHub(adapter, video, CONFIG)


@asynccontextmanager
async def lifespan(_: FastAPI):
    video.start()
    adapter.start()
    await hub.start()
    yield
    await hub.close()
    video.close()


app = FastAPI(title="Go2 Web Controller", lifespan=lifespan)
app.mount("/assets", StaticFiles(directory=WEB_DIR / "assets"), name="assets")


@app.get("/")
def index() -> FileResponse:
    return FileResponse(WEB_DIR / "index.html")


@app.get("/health")
def health() -> dict[str, Any]:
    result = {
        "ok": True,
        "backend": CONFIG.backend,
        "mock": CONFIG.mock,
        "robot_seen": adapter.telemetry["robot_seen"],
        "armed": hub.armed,
        "video": video.status(),
    }
    if CONFIG.backend == "policy":
        result["policy"] = {
            "model": "go2_blind_rough_v1.onnx",
            "publisher_created": bool(adapter.telemetry.get("publisher_created", False)),
            "frames": int(adapter.telemetry.get("policy_frames", 0)),
            "lowcmd_writes": int(adapter.telemetry.get("lowcmd_writes", 0)),
            "command": list(adapter.telemetry.get("velocity", [0.0, 0.0, 0.0])),
            "fault": str(adapter.telemetry.get("fault", "")),
        }
    return result


@app.get("/video")
def video_feed():
    if CONFIG.video_mode == "url" and CONFIG.video_url:
        return RedirectResponse(CONFIG.video_url)
    if CONFIG.video_mode == "sdk" and video.ready:
        return StreamingResponse(
            video.mjpeg_frames(),
            media_type="multipart/x-mixed-replace; boundary=frame",
            headers={"Cache-Control": "no-store, no-cache, must-revalidate"},
        )
    return FileResponse(WEB_DIR / "assets" / "mock-camera.svg", media_type="image/svg+xml")


@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket) -> None:
    await hub.handle(ws)

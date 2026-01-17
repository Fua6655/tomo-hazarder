#!/usr/bin/env python3
import asyncio
import socket
import threading

from fastapi import FastAPI, WebSocket

from importlib.resources import files
from fastapi.staticfiles import StaticFiles
from fastapi.responses import RedirectResponse

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, String, Bool

# =====================================================
# ================= CONFIG ============================
# =====================================================

UDP_PORT = 9999          # ESP -> WEB
LOG_LIMIT = 200

# =====================================================
# ================= ROS BRIDGE ========================
# =====================================================

class WebRosBridge(Node):
    def __init__(self, loop: asyncio.AbstractEventLoop):
        super().__init__("web_ros_bridge")

        self.loop = loop
        self.active_source = "web"

        # ---- SUBSCRIBE active source ----
        self.create_subscription(
            String,
            "factory/active_source",
            self.source_cb,
            10
        )

        # ---- PUBLISHERS (commands) ----
        self.pub_states = self.create_publisher(UInt8MultiArray, "web/states", 10)
        self.pub_events = self.create_publisher(UInt8MultiArray, "web/events", 10)
        self.pub_lights = self.create_publisher(UInt8MultiArray, "web/lights", 10)

        # ---- FORCE WEB CONTROL ----
        self.pub_force = self.create_publisher(Bool, "web/force_control", 10)

    # -------------------------------------------------
    def publish(self, target: str, data: list[int]):
        msg = UInt8MultiArray()
        msg.data = data

        if target == "states":
            self.pub_states.publish(msg)
        elif target == "events":
            self.pub_events.publish(msg)
        elif target == "lights":
            self.pub_lights.publish(msg)

    # -------------------------------------------------
    def force_web(self, enable: bool):
        msg = Bool()
        msg.data = enable
        self.pub_force.publish(msg)

    # -------------------------------------------------
    def source_cb(self, msg: String):
        self.active_source = msg.data

        asyncio.run_coroutine_threadsafe(
            broadcast({
                "type": "source",
                "value": msg.data
            }),
            self.loop
        )


ros_node: WebRosBridge | None = None

# =====================================================
# ================= FASTAPI APP =======================
# =====================================================

app = FastAPI()

# =====================================================
# ================= STATIC FILES ======================
# =====================================================
html_dir = files("control_tomo.web") / "html"

print(f"[WEB] HTML dir: {html_dir}")

app.mount(
    "/html",
    StaticFiles(directory=str(html_dir)),
    name="html"
)

@app.get("/")
def root():
    return RedirectResponse("/html/index.html")

# =====================================================
# ================= STATE =============================
# =====================================================

WS_CLIENTS: set[WebSocket] = set()
states: dict[str, str] = {}
log_buffer: list[str] = []

# =====================================================
# ================= WEBSOCKET =========================
# =====================================================

@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()
    WS_CLIENTS.add(ws)

    # init stanje
    await ws.send_json({
        "type": "init",
        "states": states,
        "logs": log_buffer
    })

    try:
        while True:
            data = await ws.receive_json()

            # ---------- COMMAND ----------
            if data.get("type") == "cmd":
                target = data.get("target")
                values = data.get("data")

                if ros_node and isinstance(values, list):
                    ros_node.publish(target, values)

            # ---------- FORCE WEB ----------
            elif data.get("type") == "force":
                if ros_node:
                    ros_node.force_web(bool(data.get("value", False)))

    except:
        pass
    finally:
        WS_CLIENTS.discard(ws)

# ----------------------------------------------------

async def broadcast(payload: dict):
    dead = []
    for ws in WS_CLIENTS:
        try:
            await ws.send_json(payload)
        except:
            dead.append(ws)

    for d in dead:
        WS_CLIENTS.discard(d)

# =====================================================
# ================= UDP LISTENER ======================
# =====================================================

def udp_listener(loop: asyncio.AbstractEventLoop):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", UDP_PORT))

    while True:
        data, _ = sock.recvfrom(512)
        msg = data.decode(errors="ignore").strip()

        if msg.startswith("ACK"):
            continue

        if msg.startswith("STATE,"):
            _, name, value = msg.split(",", 2)
            states[name] = value

            asyncio.run_coroutine_threadsafe(
                broadcast({
                    "type": "state",
                    "name": name,
                    "value": value
                }),
                loop
            )

        elif msg.startswith("LOG,"):
            _, text = msg.split(",", 1)
            log_buffer.append(text)
            log_buffer[:] = log_buffer[-LOG_LIMIT:]

            asyncio.run_coroutine_threadsafe(
                broadcast({
                    "type": "log",
                    "text": text
                }),
                loop
            )

        else:
            log_buffer.append(msg)
            log_buffer[:] = log_buffer[-LOG_LIMIT:]

            asyncio.run_coroutine_threadsafe(
                broadcast({
                    "type": "log",
                    "text": msg
                }),
                loop
            )

# =====================================================
# ================= STARTUP ===========================
# =====================================================

@app.on_event("startup")
async def startup():
    global ros_node

    loop = asyncio.get_running_loop()

    rclpy.init(args=None)
    ros_node = WebRosBridge(loop)

    threading.Thread(
        target=rclpy.spin,
        args=(ros_node,),
        daemon=True
    ).start()

    threading.Thread(
        target=udp_listener,
        args=(loop,),
        daemon=True
    ).start()

# =====================================================

def main():
    import uvicorn
    uvicorn.run(
        "control_tomo.web.web_server:app",
        host="0.0.0.0",
        port=8000,
        log_level="info"
    )

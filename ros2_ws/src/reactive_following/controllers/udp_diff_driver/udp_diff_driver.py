"""One Webots-native controller instance per robot: motors + camera recognition.

All control and sensing are local to this robot. The robot name determines its
UDP port, but never provides any other robot's world pose or identity.
"""

import json
import socket

from controller import Robot
from perception import select_nearest_robot

WHEEL_RADIUS_M = 0.055
HALF_WHEEL_SPACING_M = 0.125
MAX_WHEEL_RAD_S = 12.0
COMMAND_TIMEOUT_S = 0.7     # Webots simulation seconds
SENSOR_PERIOD_S = 0.10      # Webots simulation seconds
BASE_UDP_PORT = 15000
VALID_ROBOT_NAMES = {f"R{i}" for i in range(1, 6)}


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def main():
    robot = Robot()
    name = robot.getName()
    if name not in VALID_ROBOT_NAMES:
        raise RuntimeError(f"Unexpected robot name: {name!r}; expected R1..R5")
    port = BASE_UDP_PORT + int(name[1:])

    step_ms = int(robot.getBasicTimeStep())
    left = robot.getDevice("left_wheel_motor")
    right = robot.getDevice("right_wheel_motor")
    camera = robot.getDevice("front_camera")
    if left is None or right is None or camera is None:
        raise RuntimeError(f"{name}: missing wheel motors or front_camera")

    left.setPosition(float("inf"))
    right.setPosition(float("inf"))
    left.setVelocity(0.0)
    right.setVelocity(0.0)
    camera.enable(step_ms)              # For Webots camera overlay
    camera.recognitionEnable(step_ms)   # Current visible robot measurements

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", port))
    sock.setblocking(False)
    print(f"[Webots {name}] Camera + wheel controller at UDP 127.0.0.1:{port}", flush=True)

    v = w = 0.0
    last_rx_sim_time = -100.0
    last_sensor_sim_time = -100.0
    bridge_address = None
    last_logged_detection = None

    try:
        while robot.step(step_ms) != -1:
            now = robot.getTime()
            while True:
                try:
                    packet, sender = sock.recvfrom(4096)
                except BlockingIOError:
                    break
                except OSError as exc:
                    print(f"[Webots {name}] UDP receive error: {exc}", flush=True)
                    break
                try:
                    msg = json.loads(packet.decode("utf-8"))
                    v = float(msg["linear"])
                    w = float(msg["angular"])
                except (ValueError, KeyError, TypeError, UnicodeError) as exc:
                    print(f"[Webots {name}] Invalid command: {exc}", flush=True)
                    continue
                bridge_address = sender
                last_rx_sim_time = now
                try:
                    sock.sendto(f"{name}:ok".encode("ascii"), sender)
                except OSError:
                    pass

            if now - last_rx_sim_time > COMMAND_TIMEOUT_S:
                v = w = 0.0
            left_rad_s = (v - HALF_WHEEL_SPACING_M * w) / WHEEL_RADIUS_M
            right_rad_s = (v + HALF_WHEEL_SPACING_M * w) / WHEEL_RADIUS_M
            left.setVelocity(clamp(left_rad_s, -MAX_WHEEL_RAD_S, MAX_WHEEL_RAD_S))
            right.setVelocity(clamp(right_rad_s, -MAX_WHEEL_RAD_S, MAX_WHEEL_RAD_S))

            if bridge_address is None or now - last_sensor_sim_time < SENSOR_PERIOD_S:
                continue
            last_sensor_sim_time = now

            detection = select_nearest_robot(camera.getRecognitionObjects())
            if detection is None:
                telemetry = {"type": "target", "visible": False, "count": 0}
            else:
                distance, bearing, count = detection
                telemetry = {
                    "type": "target", "visible": True,
                    "distance_m": round(distance, 4),
                    "bearing_rad": round(bearing, 5),
                    "count": count,
                }
            try:
                sock.sendto(json.dumps(telemetry).encode("utf-8"), bridge_address)
            except OSError as exc:
                print(f"[Webots {name}] UDP sensor send error: {exc}", flush=True)
            if telemetry["visible"] != last_logged_detection:
                last_logged_detection = telemetry["visible"]
                print(f"[Webots {name}] {'target visible' if last_logged_detection else 'no target'}", flush=True)
    finally:
        left.setVelocity(0.0)
        right.setVelocity(0.0)
        sock.close()


if __name__ == "__main__":
    main()

"""One Webots controller per robot: wheel UDP + local camera and 360° LiDAR.

Only this robot's Webots devices are read. No Supervisor, GPS, global pose,
robot-to-robot messages or target tracking history.
"""
import json
import math
import socket
from controller import Robot
from perception import scan_markers
from lidar_scan import combine_layers

WHEEL_RADIUS_M = 0.055
HALF_WHEEL_SPACING_M = 0.125
MAX_WHEEL_RAD_S = 12.0
COMMAND_TIMEOUT_S = 0.7
SENSOR_PERIOD_S = 0.064


def clamp(x, a, b):
    return min(b, max(a, x))


def main():
    robot = Robot()
    name = robot.getName()
    if name not in {f'R{i}' for i in range(1, 6)}:
        raise RuntimeError('Expected Webots robot name R1..R5')
    port = 15000 + int(name[1:])
    dt = int(robot.getBasicTimeStep())
    left = robot.getDevice('left_wheel_motor')
    right = robot.getDevice('right_wheel_motor')
    camera = robot.getDevice('front_camera')
    lidar = robot.getDevice('safety_lidar')
    bumper = robot.getDevice('collision_bumper')
    if any(item is None for item in (left, right, camera, lidar, bumper)):
        raise RuntimeError(f'{name}: motor/camera/LiDAR/bumper missing')
    for motor in (left, right):
        motor.setPosition(float('inf'))
        motor.setVelocity(0.0)
    camera.enable(dt)
    camera.recognitionEnable(dt)
    lidar.enable(dt * 2)
    bumper.enable(dt)
    if lidar.getHorizontalResolution() != 360 or lidar.getNumberOfLayers() != 3:
        raise RuntimeError(f'{name}: expected 360 beams x 3 LiDAR layers')

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', port))
    sock.setblocking(False)
    v = w = 0.0
    bridge = None
    last_rx = last_tx = -100.0
    contact_was_active = False
    contact_episodes = 0
    last_contact_at = -100.0
    last_log = None
    print(f'[Webots {name}] camera + 360deg lidar UDP :{port}', flush=True)
    try:
        while robot.step(dt) != -1:
            now = robot.getTime()
            while True:
                try:
                    packet, sender = sock.recvfrom(4096)
                except BlockingIOError:
                    break
                except ConnectionResetError:
                    # Windows 10054 when a previous UDP peer stopped.
                    bridge = None
                    v = w = 0.0
                    break
                try:
                    cmd = json.loads(packet.decode('utf-8'))
                    new_v, new_w = float(cmd['linear']), float(cmd['angular'])
                    if not (math.isfinite(new_v) and math.isfinite(new_w)):
                        continue
                except (ValueError, TypeError, KeyError, UnicodeError):
                    continue
                v, w = new_v, new_w
                bridge, last_rx = sender, now
                try:
                    sock.sendto(f'{name}:ok'.encode('ascii'), sender)
                except OSError:
                    bridge = None
                    v = w = 0.0
                    break
            if now - last_rx > COMMAND_TIMEOUT_S:
                v = w = 0.0
            left.setVelocity(clamp((v - HALF_WHEEL_SPACING_M * w) / WHEEL_RADIUS_M,
                                   -MAX_WHEEL_RAD_S, MAX_WHEEL_RAD_S))
            right.setVelocity(clamp((v + HALF_WHEEL_SPACING_M * w) / WHEEL_RADIUS_M,
                                    -MAX_WHEEL_RAD_S, MAX_WHEEL_RAD_S))
            contact = bumper.getValue() > 0.5
            if contact and not contact_was_active and now - last_contact_at >= 0.3:
                contact_episodes += 1
                last_contact_at = now
            contact_was_active = contact

            if bridge is None or now - last_tx < SENSOR_PERIOD_S:
                continue
            last_tx = now
            observed = scan_markers(camera.getRecognitionObjects())
            rear, front = observed['rear'], observed['front']
            lidar_ranges = combine_layers(lidar.getRangeImage())
            telemetry = {
                'type': 'target',
                'visible': rear is not None,
                'rear_visible': rear is not None,
                'front_visible': front is not None,
                'distance_m': round(rear[0], 4) if rear else None,
                'bearing_rad': round(rear[1], 5) if rear else None,
                'front_distance_m': round(front[0], 4) if front else None,
                'front_bearing_rad': round(front[1], 5) if front else None,
                'count': observed['count'],
                'raw_recognition_count': observed['raw_count'],
                'recognition_models': observed['model_types'],
                'camera_classification': ('front=%d rear=%d ambiguous=%d unmatched=%d bodies=%d' % (
                    observed['front_count'], observed['rear_count'],
                    observed['uncertain_count'], observed['unmatched_count'], observed['body_count'])),
                'lidar_ranges_m': lidar_ranges,
                'contact': contact,
                'contact_episodes': contact_episodes,
                'sim_time_s': round(now, 4)
            }
            try:
                sock.sendto(json.dumps(telemetry, separators=(',', ':')).encode('utf-8'), bridge)
            except OSError:
                bridge = None
            mode = (rear is not None, front is not None, lidar_ranges is not None)
            if mode != last_log:
                last_log = mode
                print(f'[Webots {name}] green={mode[0]} red={mode[1]} lidar_valid={mode[2]}', flush=True)
    finally:
        left.setVelocity(0.0)
        right.setVelocity(0.0)
        sock.close()

if __name__ == '__main__':
    main()

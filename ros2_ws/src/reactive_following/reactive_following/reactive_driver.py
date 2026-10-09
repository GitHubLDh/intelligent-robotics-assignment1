import math

import rclpy
from geometry_msgs.msg import TwistStamped

# E-puck v2 physical constants (from the Webots E-puck PROTO).
WHEEL_RADIUS = 0.02
TRACK_WIDTH = 0.052
MAX_WHEEL_VELOCITY = 6.0  # rad/s, below the e-puck2 hardware max (~7.536 rad/s)


class ReactiveFollowingDriver:
    """Skeleton driver: sensing and motor control are wired up and working.
    The actual reactive behaviour (what to do with the sensor reading) is
    NOT implemented yet - that's the part to design. See step().
    """

    def init(self, webots_node, properties):
        self.robot = webots_node.robot
        self.timestep = int(self.robot.getBasicTimeStep())

        self.left_motor = self.robot.getDevice('left wheel motor')
        self.right_motor = self.robot.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

        self.lidar = self.robot.getDevice('lidar')
        self.lidar.enable(self.timestep)

        rclpy.init(args=None)
        self.node = rclpy.create_node('reactive_following_driver')

        # Per-robot "base curve" when nothing is detected - set from the
        # launch file so each of the 5 robots can have a different one.
        self.node.declare_parameter('base_linear_speed', 0.05)
        self.node.declare_parameter('base_angular_speed', 0.3)
        self.base_v = self.node.get_parameter('base_linear_speed').value
        self.base_w = self.node.get_parameter('base_angular_speed').value

        self.cmd_pub = self.node.create_publisher(TwistStamped, 'cmd_vel', 1)

    def _nearest_target_bearing(self):
        """Returns the bearing (radians, +left/-right) to the nearest
        detected robot within the lidar's range/FOV, or None if nothing
        is detected. Already handles the Webots Lidar API - no need to
        touch this part."""
        ranges = self.lidar.getRangeImage()
        if not ranges:
            return None

        fov = self.lidar.getFov()
        n = len(ranges)
        angle_step = fov / (n - 1)

        nearest_range = float('inf')
        nearest_bearing = None
        for i, r in enumerate(ranges):
            if math.isinf(r) or math.isnan(r):
                continue
            if r < nearest_range:
                nearest_range = r
                # index 0 = +fov/2 (left edge), decreasing to -fov/2 (right edge)
                nearest_bearing = fov / 2 - i * angle_step

        return nearest_bearing

    def _set_wheel_velocities(self, v, w):
        """Converts desired linear speed v (m/s) and angular speed w (rad/s)
        into left/right wheel motor velocities and applies them. Already
        working - no need to touch this part."""
        left_speed = (v - w * TRACK_WIDTH / 2.0) / WHEEL_RADIUS
        right_speed = (v + w * TRACK_WIDTH / 2.0) / WHEEL_RADIUS
        left_speed = max(-MAX_WHEEL_VELOCITY, min(MAX_WHEEL_VELOCITY, left_speed))
        right_speed = max(-MAX_WHEEL_VELOCITY, min(MAX_WHEEL_VELOCITY, right_speed))
        self.left_motor.setVelocity(left_speed)
        self.right_motor.setVelocity(right_speed)

    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0)

        bearing = self._nearest_target_bearing()

        # TODO (this is the actual assignment): decide v and w here.
        #
        # Per the spec:
        #  - bearing is None            -> no robot detected, follow the base curve
        #  - bearing is not None        -> a robot is detected, turn toward it
        #                                  (turn strength should depend on how
        #                                  misaligned we are - think about what
        #                                  function of `bearing` makes sense)
        #  - linear speed should stay constant in all cases (per the spec)
        #
        # For now this just always drives the base curve, ignoring detections,
        # so the robots move but don't actually follow each other yet.
        v = self.base_v
        w = self.base_w

        self._set_wheel_velocities(v, w)

        msg = TwistStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.twist.linear.x = v
        msg.twist.angular.z = w
        self.cmd_pub.publish(msg)

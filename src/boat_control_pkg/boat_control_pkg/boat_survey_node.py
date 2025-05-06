#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from px4_msgs.msg import OffboardControlMode, VehicleCommand, TrajectorySetpoint, VehicleLocalPosition
from std_msgs.msg import String

class SimpleBoatController(Node):
    def __init__(self):
        super().__init__('simple_boat_controller')

        # QoS setup for matching PX4 reliability
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT  

        # Publishers
        self.offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.status_pub = self.create_publisher(String, '/anushka/current_boat_location', 10)

        # Subscriptions
        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.position_callback,
            qos_profile
        )

        self.current_x = None
        self.current_y = None

        # Dummy warmup
        self.startup_counter = 0
        self.startup_warmup_loops = 20  # ~2 seconds at 10Hz

        # Waypoints to test
        self.waypoints = [[0.0, 0.0], [5.0, 0.0]]
        self.current_waypoint_index = 0
        self.reached_waypoints = set()

        # Flags
        self.mode_set = False
        self.armed = False

        self.timer = self.create_timer(0.1, self.control_loop)

    def position_callback(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        z = msg.z
        self.status_pub.publish(String(data=f"x: {self.current_x:.2f}, y: {self.current_y:.2f}, z: {z:.2f}"))
        self.get_logger().info(f"📍 Received position: x={self.current_x:.2f}, y={self.current_y:.2f}, z={z:.2f}")

    def control_loop(self):
        now = self.get_clock().now().nanoseconds // 1000

        # 1. Always publish offboard control mode
        offboard = OffboardControlMode()
        offboard.timestamp = now
        offboard.position = True
        self.offboard_pub.publish(offboard)
        self.get_logger().debug("Published OffboardControlMode")

        # 2. Publish dummy setpoint every loop
        dummy = TrajectorySetpoint()
        dummy.timestamp = now
        dummy.position = [0.0, 0.0, -1.0]
        dummy.yaw = 0.0
        self.setpoint_pub.publish(dummy)
        self.get_logger().debug("Published dummy TrajectorySetpoint")

        # 3. Wait a few loops before arming and switching
        if self.startup_counter < self.startup_warmup_loops:
            self.startup_counter += 1
            self.get_logger().debug(f"Warmup loop {self.startup_counter}/{self.startup_warmup_loops}")
            return

        # 4. Set OFFBOARD mode
        if not self.mode_set:
            self.set_offboard_mode()
            self.mode_set = True

        # 5. Arm vehicle
        if not self.armed:
            self.arm()
            self.armed = True

        # 6. Navigate through waypoints
        if self.current_waypoint_index < len(self.waypoints):
            target_x, target_y = self.waypoints[self.current_waypoint_index]

            if self.current_x is not None and self.current_y is not None:
                dx = self.current_x - target_x
                dy = self.current_y - target_y
                dist = (dx**2 + dy**2)**0.5

                self.get_logger().info(f"🚩 Distance to waypoint {self.current_waypoint_index}: {dist:.2f} m")

                if dist < 0.5:
                    if self.current_waypoint_index not in self.reached_waypoints:
                        self.get_logger().info(f"✅ Reached waypoint: ({target_x:.1f}, {target_y:.1f})")
                        self.reached_waypoints.add(self.current_waypoint_index)
                    self.current_waypoint_index += 1
                    return

                # Send trajectory setpoint
                traj = TrajectorySetpoint()
                traj.timestamp = now
                traj.position = [target_x, target_y, -1.0]
                traj.yaw = 0.0
                self.setpoint_pub.publish(traj)
                self.get_logger().info(f"📤 Sent setpoint to ({target_x:.1f}, {target_y:.1f})")

    def set_offboard_mode(self):
        msg = VehicleCommand()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.param1 = 1.0
        msg.param2 = 6.0  # OFFBOARD mode
        msg.command = 176
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)
        self.get_logger().info("📡 Sent: Offboard Mode")

    def arm(self):
        msg = VehicleCommand()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.param1 = 1.0
        msg.command = 400  # ARM
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)
        self.get_logger().info("🔫 Sent: Arm Command")

def main(args=None):
    rclpy.init(args=args)
    node = SimpleBoatController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


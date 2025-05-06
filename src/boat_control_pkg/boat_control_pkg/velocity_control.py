#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.parameter import Parameter
from px4_msgs.msg import (
    OffboardControlMode,
    VehicleCommand,
    TrajectorySetpoint,
    VehicleLocalPosition,
    VehicleStatus
)
import math
import time
import numpy as np # Used for NaN

class PositionWaypointCommander(Node):
    """
    Node for controlling a PX4 vehicle to a single waypoint using Offboard mode,
    sending POSITION setpoints (compatible with standard MPC).
    Corrected for IndentationError.
    """
    def __init__(self):
        super().__init__('position_waypoint_commander')

        # --- Parameters ---
        self.declare_parameter('waypoint_x', 100.0) # Example target
        self.declare_parameter('waypoint_y', 0.0)  # Example target
        self.declare_parameter('waypoint_z', 0.0)
        self.declare_parameter('acceptance_radius', 0.5) # Meters
        self.declare_parameter('control_frequency', 10.0) # Hz
        self.declare_parameter('offboard_heartbeat_hz', 5.0) # Required minimum > 2Hz
        self.declare_parameter('arm_timeout_sec', 5.0)
        self.declare_parameter('offboard_timeout_sec', 5.0)

        self.waypoint = [
            self.get_parameter('waypoint_x').value,
            self.get_parameter('waypoint_y').value,
            self.get_parameter('waypoint_z').value
        ]
        self.acceptance_radius = self.get_parameter('acceptance_radius').value
        self.control_interval = 1.0 / self.get_parameter('control_frequency').value
        self.offboard_interval = 1.0 / self.get_parameter('offboard_heartbeat_hz').value

        self.get_logger().info(f"Target Waypoint: {self.waypoint}, Acceptance Radius: {self.acceptance_radius}m")
        self.get_logger().info(f"Control Frequency: {1.0/self.control_interval} Hz, Offboard Heartbeat: {1.0/self.offboard_interval} Hz")

        # --- QoS Profiles ---
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # --- Publishers ---
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_reliable)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_reliable)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_reliable)

        # --- Subscribers ---
        position_topic_name = '/fmu/out/vehicle_local_position' # !!! VERIFY THIS NAME !!!
        status_topic_name = '/fmu/out/vehicle_status_v1'
        self.get_logger().info(f"Subscribing to position topic: {position_topic_name}")
        self.position_sub = self.create_subscription(
            VehicleLocalPosition, position_topic_name, self.position_callback, qos_best_effort)
        self.get_logger().info(f"Subscribing to status topic: {status_topic_name}")
        self.status_sub = self.create_subscription(
            VehicleStatus, status_topic_name, self.status_callback, qos_best_effort)

        # --- State Variables ---
        self.current_pos = None
        self.current_yaw = None
        self.vehicle_status = None
        self.state = "INIT"
        self.last_offboard_sent_time = 0
        self.last_setpoint_sent_time = 0
        self.command_start_time = 0
        self.last_init_log_time = 0

        # --- Timers ---
        self.control_timer = self.create_timer(self.control_interval, self.control_loop)
        self.offboard_timer = self.create_timer(self.offboard_interval, self.publish_offboard_mode)

    # --- Callbacks ---
    def position_callback(self, msg):
        self.current_pos = [msg.x, msg.y, msg.z]
        self.current_yaw = msg.heading

    def status_callback(self, msg):
        self.vehicle_status = msg

    # --- State Checks ---
    def is_armed(self):
        return self.vehicle_status and self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def is_in_offboard_mode(self):
        return self.vehicle_status and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

    # --- Publishing Functions ---
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param7 = float(param7)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.command_pub.publish(msg)

    def publish_offboard_mode(self):
        """ Publishes the OffboardControlMode message heartbeat signalling POSITION control intent. """
        now_ns = self.get_clock().now().nanoseconds
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = now_ns // 1000
        self.offboard_pub.publish(msg)
        self.last_offboard_sent_time = now_ns / 1e9

    def publish_position_setpoint(self):
        """ Calculates yaw and publishes the TrajectorySetpoint using POSITION control. """
        if self.current_pos is None:
            if self.state != "INIT":
                 self.get_logger().warn("Cannot publish setpoint, current position unknown.", throttle_duration_sec=5)
            return

        dx = self.waypoint[0] - self.current_pos[0]
        dy = self.waypoint[1] - self.current_pos[1]
        dist_xy = math.hypot(dx, dy)

        target_yaw = math.atan2(dy, dx)

        msg = TrajectorySetpoint()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.position = [self.waypoint[0], self.waypoint[1], self.waypoint[2]] # Command target position
        msg.velocity = [np.nan, np.nan, np.nan] # Set velocity fields to NaN
        msg.acceleration = [np.nan, np.nan, np.nan] # Set acceleration fields to NaN
        msg.yaw = target_yaw
        msg.yawspeed = np.nan

        self.setpoint_pub.publish(msg)
        self.last_setpoint_sent_time = self.get_clock().now().nanoseconds / 1e9

        self.get_logger().debug(f"Sent setpoint: Pos=[{msg.position[0]:.1f}, {msg.position[1]:.1f}], Yaw={math.degrees(target_yaw):.1f}° (Dist: {dist_xy:.2f}m)")

    # --- Command Functions ---
    def arm(self):
        self.get_logger().info("Sending ARM command...")
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.command_start_time = time.time()

    def disarm(self):
        self.get_logger().info("Sending DISARM command...")
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)

    def set_offboard_mode(self):
        self.get_logger().info("Sending SET_MODE=OFFBOARD command...")
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.command_start_time = time.time()

    # --- Main Control Logic ---
    def check_waypoint_reached(self):
        if self.current_pos is None: return False
        dx = self.waypoint[0] - self.current_pos[0]
        dy = self.waypoint[1] - self.current_pos[1]
        dist_xy = math.hypot(dx, dy)
        return dist_xy < self.acceptance_radius

    def control_loop(self):
        """ Main state machine and control logic, executed periodically."""
        now = time.time()
        state_changed = False

        # --- State Machine ---
        if self.state == "INIT":
            # Wait for first position AND status messages
            if self.current_pos is not None and self.vehicle_status is not None:
                # Ensure we are sending offboard heartbeat BEFORE trying to switch mode
                if (now - self.last_offboard_sent_time) < 2 * self.offboard_interval:
                    self.get_logger().info("Position and Status received.")
                    self.state = "ARMING"
                    state_changed = True
                else:
                    # Log less frequently while waiting for heartbeat
                    if (now - self.last_init_log_time) > 5.0:
                        self.get_logger().info("Waiting for first Offboard heartbeat to be sent...")
                        self.last_init_log_time = now
            else:
                 # Log less frequently while waiting for messages
                 # *** CORRECTED INDENTATION FOR THIS BLOCK ***
                 if (now - self.last_init_log_time) > 5.0:
                     missing = []
                     if self.current_pos is None:
                         missing.append("Position")
                     if self.vehicle_status is None:
                         missing.append("Status")
                     self.get_logger().info(f"Waiting for first {' and '.join(missing)} message(s)...")
                     self.last_init_log_time = now
                 # *** END CORRECTION ***

        elif self.state == "ARMING":
            # Need to send setpoints before/during arming for PX4 checks
            self.publish_position_setpoint() # Send initial target position

            if not self.is_armed():
                if self.command_start_time > 0 and (now - self.command_start_time) > self.get_parameter('arm_timeout_sec').value:
                     self.get_logger().error("Arming timed out!")
                     self.state = "FAILED"
                     state_changed = True
                elif self.command_start_time == 0 or (now - self.command_start_time) > 1.0:
                    self.arm()
            else:
                self.get_logger().info("Vehicle ARMED")
                self.state = "OFFBOARD_REQUEST"
                self.command_start_time = 0
                state_changed = True

        elif self.state == "OFFBOARD_REQUEST":
             # Keep publishing position setpoint
             self.publish_position_setpoint()

             if not self.is_in_offboard_mode():
                 if self.command_start_time > 0 and (now - self.command_start_time) > self.get_parameter('offboard_timeout_sec').value:
                     self.get_logger().error("Switching to Offboard mode timed out!")
                     self.state = "FAILED"
                     state_changed = True
                 elif self.command_start_time == 0 or (now - self.command_start_time) > 1.0:
                     self.set_offboard_mode()
             else:
                 self.get_logger().info("Vehicle in OFFBOARD mode")
                 self.state = "ACTIVE"
                 self.command_start_time = 0
                 state_changed = True

        elif self.state == "ACTIVE":
            # Publish position setpoint
            self.publish_position_setpoint()

            # Check if waypoint is reached
            if self.check_waypoint_reached():
                self.get_logger().info("-------------------------")
                self.get_logger().info(">>> Waypoint Reached! <<<")
                self.get_logger().info(f"Final Position: [{self.current_pos[0]:.2f}, {self.current_pos[1]:.2f}]")
                self.get_logger().info("-------------------------")
                self.state = "WAYPOINT_REACHED"
                state_changed = True

        elif self.state == "WAYPOINT_REACHED":
            self.get_logger().info("Waypoint reached state. Idling.", throttle_duration_sec=10)
            # Note: Continues sending final position setpoint unless explicitly stopped/changed

        elif self.state == "FAILED":
            if self.is_armed():
                 if self.command_start_time > 0 and (now - self.command_start_time) > 1.0:
                     self.disarm()
                     self.command_start_time = now
            self.get_logger().error("Control script failed. Check logs.", throttle_duration_sec=10)


        if state_changed:
            self.get_logger().info(f"State changed to: {self.state}")


def main(args=None):
    rclpy.init(args=args)
    print("Starting Position Waypoint Commander node...")
    node = PositionWaypointCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboard interrupt detected. Shutting down...")
        if node.state not in ["INIT", "FAILED"] and node.is_armed():
             print("Attempting to disarm...")
             node.disarm()
             time.sleep(1)
    finally:
        if rclpy.ok():
             node.destroy_node()
             rclpy.shutdown()
        print("Node shut down.")


if __name__ == '__main__':
    main()

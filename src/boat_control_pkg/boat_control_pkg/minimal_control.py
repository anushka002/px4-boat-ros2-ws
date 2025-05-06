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
    VehicleStatus  # Make sure you have the correct message type if v1 implies a different structure (usually not)
)
import math
import time

class OneWaypointCommander(Node):
    """
    Node for controlling a PX4 vehicle (boat simulation) to a single waypoint
    using Offboard mode. Corrected for BEST_EFFORT QoS on subscribers.
    """
    def __init__(self):
        super().__init__('one_waypoint_commander')

        # --- Parameters ---
        self.declare_parameter('waypoint_x', 15.0)
        self.declare_parameter('waypoint_y', 15.0)
        self.declare_parameter('waypoint_z', 0.0) # Use 0.0 for surface vehicles assuming local frame origin is at water level
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
        # RELIABLE for commands and setpoints sent TO PX4
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        # BEST_EFFORT for status/position data received FROM PX4
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE, # PX4 publishers are volatile
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # --- Publishers (Use Reliable) ---
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_reliable)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_reliable)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_reliable)

        # --- Subscribers (Use Best Effort to match PX4 publishers) ---

        # !!! IMPORTANT !!!
        # VERIFY this position topic name is correct for your setup using:
        # ros2 topic list | grep position
        position_topic_name = '/fmu/out/vehicle_local_position'
        self.get_logger().info(f"Subscribing to position topic: {position_topic_name}")
        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            position_topic_name,
            self.position_callback,
            qos_best_effort  # Use BEST_EFFORT
        )

        # Use the corrected status topic name
        status_topic_name = '/fmu/out/vehicle_status_v1'
        self.get_logger().info(f"Subscribing to status topic: {status_topic_name}")
        self.status_sub = self.create_subscription(
            VehicleStatus,
            status_topic_name,
            self.status_callback,
            qos_best_effort # Use BEST_EFFORT
        )

        # --- State Variables ---
        self.current_pos = None  # Stores the latest [x, y, z]
        self.current_yaw = None  # Stores the latest yaw
        self.vehicle_status = None # Stores latest VehicleStatus msg

        self.state = "INIT" # INIT, ARMING, OFFBOARD_REQUEST, ACTIVE, WAYPOINT_REACHED, FAILED
        self.last_offboard_sent_time = 0
        self.last_setpoint_sent_time = 0
        self.command_start_time = 0
        self.last_init_log_time = 0 # For throttling INIT log message

        # --- Timers ---
        self.control_timer = self.create_timer(self.control_interval, self.control_loop)
        self.offboard_timer = self.create_timer(self.offboard_interval, self.publish_offboard_mode)

    # --- Callbacks ---
    def position_callback(self, msg):
        """Handles incoming vehicle position data."""
        # self.get_logger().debug("<<< Position Callback Triggered >>>") # Uncomment for verbose debugging
        self.current_pos = [msg.x, msg.y, msg.z]
        self.current_yaw = msg.heading # Assuming heading is the yaw in radians

    def status_callback(self, msg):
        """Handles incoming vehicle status data."""
        # self.get_logger().debug("<<< Status Callback Triggered >>>") # Uncomment for verbose debugging
        self.vehicle_status = msg

    # --- State Checks ---
    def is_armed(self):
        """Checks if the vehicle is armed based on status."""
        return self.vehicle_status and self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def is_in_offboard_mode(self):
        """Checks if the vehicle is in offboard mode based on status."""
        return self.vehicle_status and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

    def get_mode_name(self, nav_state):
        """Returns human-readable mode name."""
        # Add more states as needed from VehicleStatus message definition
        if nav_state == VehicleStatus.NAVIGATION_STATE_MANUAL: return "MANUAL"
        if nav_state == VehicleStatus.NAVIGATION_STATE_ALTCTL: return "ALTITUDE CONTROL"
        if nav_state == VehicleStatus.NAVIGATION_STATE_POSCTL: return "POSITION CONTROL"
        if nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_MISSION: return "AUTO MISSION"
        if nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER: return "AUTO LOITER"
        if nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_RTL: return "AUTO RTL"
        if nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD: return "OFFBOARD"
        return f"UNKNOWN ({nav_state})"

    # --- Publishing Functions ---
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        """Publishes a VehicleCommand message."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param7 = float(param7) # Often used for altitude in commands like NAV_WAYPOINT
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.command_pub.publish(msg)

    def publish_offboard_mode(self):
        """Publishes the OffboardControlMode message heartbeat."""
        now_ns = self.get_clock().now().nanoseconds
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = now_ns // 1000
        self.offboard_pub.publish(msg)
        self.last_offboard_sent_time = now_ns / 1e9 # Store time in seconds

    def publish_target_setpoint(self):
        """Calculates and publishes the TrajectorySetpoint towards the waypoint."""
        if self.current_pos is None:
            # Don't warn if we are still in INIT state
            if self.state != "INIT":
                self.get_logger().warn("Cannot publish setpoint, current position unknown.", throttle_duration_sec=5)
            return

        dx = self.waypoint[0] - self.current_pos[0]
        dy = self.waypoint[1] - self.current_pos[1]

        # Calculate yaw to face the target waypoint's XY coordinates
        target_yaw = math.atan2(dy, dx)

        msg = TrajectorySetpoint()
        # Use target waypoint coordinates directly
        msg.position = [self.waypoint[0], self.waypoint[1], self.waypoint[2]]
        msg.yaw = target_yaw
        # msg.velocity = [float('nan'), float('nan'), float('nan')] # Set NaN for PX4 to ignore
        # msg.acceleration = [float('nan'), float('nan'), float('nan')] # Set NaN for PX4 to ignore
        msg.timestamp = self.get_clock().now().nanoseconds // 1000

        self.setpoint_pub.publish(msg)
        self.last_setpoint_sent_time = self.get_clock().now().nanoseconds / 1e9

        self.get_logger().debug(f"Sent setpoint: Pos=[{msg.position[0]:.1f}, {msg.position[1]:.1f}, {msg.position[2]:.1f}], Yaw={math.degrees(msg.yaw):.1f}°")


    # --- Command Functions ---
    def arm(self):
        """Sends the ARM command."""
        self.get_logger().info("Sending ARM command...")
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.command_start_time = time.time() # Start timer for timeout check

    def disarm(self):
        """Sends the DISARM command."""
        self.get_logger().info("Sending DISARM command...")
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)

    def set_offboard_mode(self):
        """Sends the command to switch to OFFBOARD mode."""
        self.get_logger().info("Sending SET_MODE=OFFBOARD command...")
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0) # 1.0 = Custom Mode, 6.0 = Offboard
        self.command_start_time = time.time() # Start timer for timeout check

    # --- Main Control Logic ---
    def check_waypoint_reached(self):
        """Checks if the vehicle is within the acceptance radius of the waypoint."""
        if self.current_pos is None:
            return False

        dx = self.waypoint[0] - self.current_pos[0]
        dy = self.waypoint[1] - self.current_pos[1]
        dist_xy = math.hypot(dx, dy)

        return dist_xy < self.acceptance_radius

    def control_loop(self):
        """Main state machine and control logic, executed periodically."""
        now = time.time()
        state_changed = False

        # --- State Machine ---
        if self.state == "INIT":
            # Wait for first position AND status messages
            if self.current_pos is not None and self.vehicle_status is not None:
                # Ensure we are sending offboard heartbeat BEFORE trying to switch mode
                # The separate offboard_timer handles this, just check if one was sent recently
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
                 if (now - self.last_init_log_time) > 5.0:
                     missing = []
                     if self.current_pos is None: missing.append("Position")
                     if self.vehicle_status is None: missing.append("Status")
                     self.get_logger().info(f"Waiting for first {' and '.join(missing)} message(s)...")
                     self.last_init_log_time = now

        elif self.state == "ARMING":
            # Need to send setpoints before/during arming for PX4 checks
            self.publish_target_setpoint() # Send initial target

            if not self.is_armed():
                # Check for timeout only after attempting to command
                if self.command_start_time > 0 and (now - self.command_start_time) > self.get_parameter('arm_timeout_sec').value:
                     self.get_logger().error("Arming timed out!")
                     self.state = "FAILED"
                     state_changed = True
                # Send arm command periodically until armed
                elif self.command_start_time == 0 or (now - self.command_start_time) > 1.0: # Retry every second
                    self.arm()
            else:
                self.get_logger().info("Vehicle ARMED")
                self.state = "OFFBOARD_REQUEST"
                self.command_start_time = 0 # Reset timer for next command
                state_changed = True

        elif self.state == "OFFBOARD_REQUEST":
             # Keep sending setpoints while trying to switch mode
             self.publish_target_setpoint()

             if not self.is_in_offboard_mode():
                 # Check for timeout only after attempting to command
                 if self.command_start_time > 0 and (now - self.command_start_time) > self.get_parameter('offboard_timeout_sec').value:
                     self.get_logger().error("Switching to Offboard mode timed out!")
                     self.state = "FAILED"
                     state_changed = True
                 # Send offboard command periodically until mode switch confirmed
                 elif self.command_start_time == 0 or (now - self.command_start_time) > 1.0: # Retry every second
                     self.set_offboard_mode()
             else:
                 self.get_logger().info("Vehicle in OFFBOARD mode")
                 self.state = "ACTIVE"
                 self.command_start_time = 0 # Reset timer
                 state_changed = True

        elif self.state == "ACTIVE":
            # Continuously publish the target setpoint
            self.publish_target_setpoint()

            # Check if waypoint is reached
            if self.check_waypoint_reached():
                self.get_logger().info("-------------------------")
                self.get_logger().info(">>> Waypoint Reached! <<<")
                self.get_logger().info(f"Final Position: [{self.current_pos[0]:.2f}, {self.current_pos[1]:.2f}]")
                self.get_logger().info("-------------------------")
                self.state = "WAYPOINT_REACHED"
                state_changed = True
                # Optional: Disarm or switch to Loiter mode here
                # self.disarm()

        elif self.state == "WAYPOINT_REACHED":
            # Keep publishing offboard heartbeat (handled by separate timer)
            # Optionally publish setpoint to hold position, or just idle
            self.get_logger().info("Waypoint reached state. Idling.", throttle_duration_sec=10)
            pass

        elif self.state == "FAILED":
            # Stop sending commands, maybe disarm
            if self.is_armed():
                 # Send disarm periodically only if arming/offboard failed
                 if self.command_start_time > 0 and (now - self.command_start_time) > 1.0:
                     self.disarm()
                     self.command_start_time = now # Reset disarm timer
            self.get_logger().error("Control script failed. Check logs.", throttle_duration_sec=10)
            pass

        if state_changed:
            self.get_logger().info(f"State changed to: {self.state}")


def main(args=None):
    rclpy.init(args=args)
    print("Starting One Waypoint Commander node...")
    node = OneWaypointCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboard interrupt detected. Shutting down...")
        # Attempt to disarm if node is destroyed cleanly
        if node.state not in ["INIT", "FAILED"] and node.is_armed():
             print("Attempting to disarm...")
             node.disarm()
             time.sleep(1) # Allow time for command to be sent
    finally:
        if rclpy.ok():
             node.destroy_node()
             rclpy.shutdown()
        print("Node shut down.")

if __name__ == '__main__':
    main()

#!/usr/bin/env python3

# Standard Python Libraries
import sys
import math
import time
import numpy as np

# ROS 2 Libraries
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, Point # For publishing coordinates
from cv_bridge import CvBridge, CvBridgeError # Use CvBridge from cv_bridge package

# Removed TF2 imports

# OpenCV Libraries
import cv2

class ColorObjectDetector(Node):
    """
    Detects objects based on specified HSV color ranges, estimates their 3D
    coordinates relative to the camera, calculates the intersection of the
    camera ray with the water plane, transforms this intersection point
    to the base_link frame using a HARDCODED transform, publishes the
    coordinates (with Z explicitly set to water level), and logs a per-frame ID. (ROS 2 Version)
    """
    def __init__(self):
        super().__init__('color_object_detector') # Initialize the Node with a name
        self.get_logger().info("ROS 2 Color Object Detector Node Started (Ray-Plane Intersection - Explicit Z)")

        # --- Parameters ---
        self.declare_parameter('camera_height', 1.3) # Z of camera origin relative to base_link origin
        self.declare_parameter('camera_pitch', 0.3) # Pitch of camera relative to base_link
        # CRITICAL: Vertical distance of base_link origin *above* the water surface plane (Z=0 world)
        # Positive if base_link origin is ABOVE water, Negative if BELOW, Zero if AT water level.
        self.declare_parameter('base_link_z_offset_from_water', 0.0)
        self.declare_parameter('min_contour_area', 200)
        self.declare_parameter('camera_frame_id', 'camera_link')
        self.declare_parameter('base_frame_id', 'base_link')

        # Get parameter values
        self.camera_height_m = self.get_parameter('camera_height').get_parameter_value().double_value
        self.camera_pitch_rad = self.get_parameter('camera_pitch').get_parameter_value().double_value
        self.base_link_z_offset = self.get_parameter('base_link_z_offset_from_water').get_parameter_value().double_value
        self.min_contour_area = self.get_parameter('min_contour_area').get_parameter_value().integer_value
        self.camera_frame_id = self.get_parameter('camera_frame_id').get_parameter_value().string_value
        self.base_frame_id = self.get_parameter('base_frame_id').get_parameter_value().string_value

        # Calculate effective camera height above the water plane
        self.effective_camera_height_m = self.camera_height_m + self.base_link_z_offset
        # Water plane Z coordinate in the base_link frame
        self.water_z_in_base_frame = -self.base_link_z_offset


        # --- HSV Color Ranges ---
        self.color_ranges = {
            'red': {
                'lower1': np.array([0, 120, 70]), 'upper1': np.array([10, 255, 255]),
                'lower2': np.array([170, 120, 70]), 'upper2': np.array([180, 255, 255])
            },
            'green': {
                'lower1': np.array([35, 100, 50]), 'upper1': np.array([85, 255, 255])
            },
            'white': {
                'lower1': np.array([0, 0, 200]), 'upper1': np.array([180, 50, 255])
            },
            'plastic': {
                'lower1': np.array([0, 0, 60]), 'upper1': np.array([180, 50, 150])
            }
        }
        self.get_logger().info(f"Using Camera Height (rel to base): {self.camera_height_m} m")
        self.get_logger().info(f"Using Camera Pitch (rel to base): {self.camera_pitch_rad} rad")
        self.get_logger().info(f"CRITICAL PARAM: Base Link Z Offset from Water: {self.base_link_z_offset} m")
        self.get_logger().info(f"--> Effective Camera Height above Water: {self.effective_camera_height_m} m")
        self.get_logger().info(f"--> Water Plane Z in base_link frame: {self.water_z_in_base_frame} m")
        self.get_logger().info(f"Using Min Contour Area: {self.min_contour_area}")
        self.get_logger().info(f"Expecting Camera Frame ID: {self.camera_frame_id}")
        self.get_logger().info(f"Target Base Frame ID: {self.base_frame_id}")
        self.get_logger().info(f"Defined color ranges: {list(self.color_ranges.keys())}")

        # --- Hardcoded Transform (camera_link -> base_link) ---
        self.T_base_camera = np.array([1.1, 0.0, 1.3]) # Translation vector (Camera Origin in Base Frame)

        pitch = self.camera_pitch_rad # Use parameter for consistency
        cos_p = math.cos(pitch)
        sin_p = math.sin(pitch)
        # Rotation matrix for camera frame relative to base frame
        self.R_base_camera = np.array([
            [cos_p,  0,  sin_p],
            [0,      1,  0     ],
            [-sin_p, 0,  cos_p ]
        ])

        self.get_logger().info(f"Using Hardcoded Translation T_base_camera: {self.T_base_camera}")
        self.get_logger().info(f"Using Hardcoded Rotation R_base_camera:\n{self.R_base_camera}")
        # --- End Hardcoded Transform ---


        # --- ROS Communication ---
        self.bridge = CvBridge()
        self.camera_info = None
        self.camera_matrix = None # Intrinsic matrix K
        self.distortion_coeffs = None
        self.image_height = None
        self.image_width = None
        self.vfov = None # Vertical Field of View
        self.cam_info_received = False

        # --- QoS Profiles ---
        sensor_qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        caminfo_qos_profile = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1, durability=DurabilityPolicy.VOLATILE)
        publisher_qos_profile = QoSProfile(depth=10)

        # --- Subscribers ---
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, sensor_qos_profile)
        self.cam_info_sub = self.create_subscription(CameraInfo, '/camera/camera_info', self.cam_info_callback, caminfo_qos_profile)

        # --- Publishers ---
        # Only publishing base_link coordinates now, as they represent the water surface intersection
        self.coord_pubs_base = {}
        for color in self.color_ranges.keys():
            base_topic_name = f'/detected_objects/base_link/{color}_position'
            self.coord_pubs_base[color] = self.create_publisher(PointStamped, base_topic_name, publisher_qos_profile)
            self.get_logger().info(f"Publishing {color} base_link water intersection coordinates on {base_topic_name}")

        # Publishing camera frame coords can be useful for debugging projection
        self.coord_pubs_camera_debug = {}
        for color in self.color_ranges.keys():
             cam_topic_name = f'/detected_objects/camera_debug/{color}_position'
             self.coord_pubs_camera_debug[color] = self.create_publisher(PointStamped, cam_topic_name, publisher_qos_profile)


        self.annotated_image_pub = self.create_publisher(Image, '/detected_objects/annotated_image', publisher_qos_profile)

        self.get_logger().info("Waiting for camera info...")


    def cam_info_callback(self, msg):
        """ Stores camera intrinsic parameters and calculates VFOV. """
        if not self.cam_info_received:
             self.get_logger().info("Received CameraInfo message.")
        self.camera_info = msg
        self.image_height = msg.height
        self.image_width = msg.width
        self.camera_matrix = np.array(msg.k).reshape((3, 3))
        self.distortion_coeffs = np.array(msg.d)

        if self.camera_matrix[1, 1] > 0:
             fy = self.camera_matrix[1, 1]
             self.vfov = 2 * math.atan(self.image_height / (2 * fy))
             if not self.cam_info_received:
                 self.get_logger().info(f"Camera Info Parsed: Size {self.image_width}x{self.image_height}, VFOV: {math.degrees(self.vfov):.2f} degrees")
             self.cam_info_received = True
        else:
             self.get_logger().warn("Invalid fy in CameraInfo, cannot calculate VFOV.")


    def image_callback(self, msg):
        """ Processes image, finds contours, calculates ray-plane intersection, publishes result in base_link frame. """
        if not self.cam_info_received:
            self.get_logger().warn("Waiting for complete camera info, cannot process image yet.")
            return

        if msg.header.frame_id != self.camera_frame_id:
             self.get_logger().warn(f"Image frame_id '{msg.header.frame_id}' != expected '{self.camera_frame_id}'. Check config.")

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return

        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        detected_objects_this_frame = []
        detection_counters = {color: 0 for color in self.color_ranges.keys()}

        # Iterate through defined colors
        for color_name, ranges in self.color_ranges.items():
            mask1 = cv2.inRange(hsv_image, ranges['lower1'], ranges['upper1'])
            if 'lower2' in ranges:
                mask2 = cv2.inRange(hsv_image, ranges['lower2'], ranges['upper2'])
                mask = cv2.bitwise_or(mask1, mask2)
            else:
                mask = mask1

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                area = cv2.contourArea(contour)
                if area > self.min_contour_area:
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        center_u = int(M["m10"] / M["m00"])
                        center_v = int(M["m01"] / M["m00"])

                        # --- Calculate Ray Direction in Camera Frame ---
                        fx = self.camera_matrix[0, 0]
                        fy = self.camera_matrix[1, 1]
                        cx = self.camera_matrix[0, 2]
                        cy = self.camera_matrix[1, 2]

                        # Normalized direction vector components in camera frame
                        x_norm = (center_u - cx) / fx
                        y_norm = (center_v - cy) / fy
                        z_norm = 1.0 # Points forward along camera Z axis

                        V_cam_dir = np.array([x_norm, y_norm, z_norm])
                        # Normalize the direction vector
                        norm = np.linalg.norm(V_cam_dir)
                        if norm < 1e-6: continue # Avoid division by zero
                        V_cam_dir_normalized = V_cam_dir / norm

                        # --- Optional: Publish debug camera point (at fixed distance) ---
                        debug_dist = 5.0 # Arbitrary distance for visualization
                        X_cam_dbg = V_cam_dir_normalized[0] * debug_dist
                        Y_cam_dbg = V_cam_dir_normalized[1] * debug_dist
                        Z_cam_dbg = V_cam_dir_normalized[2] * debug_dist

                        point_msg_camera_dbg = PointStamped()
                        point_msg_camera_dbg.header.stamp = msg.header.stamp
                        point_msg_camera_dbg.header.frame_id = self.camera_frame_id
                        point_msg_camera_dbg.point.x = X_cam_dbg
                        point_msg_camera_dbg.point.y = Y_cam_dbg
                        point_msg_camera_dbg.point.z = Z_cam_dbg
                        if color_name in self.coord_pubs_camera_debug:
                            self.coord_pubs_camera_debug[color_name].publish(point_msg_camera_dbg)
                        # --- End Debug Publish ---


                        # --- Calculate Ray in Base Frame ---
                        # Ray origin in base frame is the camera's translation
                        O_base = self.T_base_camera
                        # Ray direction in base frame (rotate camera direction vector)
                        V_base_dir = self.R_base_camera @ V_cam_dir_normalized

                        # --- Calculate Ray Intersection with Water Plane (Z = water_z_in_base_frame) ---
                        # Ray: P(t) = O_base + t * V_base_dir
                        # Plane: Z = self.water_z_in_base_frame

                        # Check if ray is parallel to the plane or points away from it vertically
                        if abs(V_base_dir[2]) < 1e-6:
                            # self.get_logger().debug(f"Ray direction Z component near zero ({V_base_dir[2]:.3f}), skipping intersection.")
                            continue
                        # Check if ray points upwards relative to base_link Z if camera is above water
                        # or downwards if camera is below water. It should point towards the plane.
                        # If V_base_dir[2] has the same sign as (O_base[2] - self.water_z_in_base_frame),
                        # it means the ray is pointing away from the plane.
                        if (O_base[2] - self.water_z_in_base_frame) * V_base_dir[2] > 0:
                            # self.get_logger().debug(f"Ray pointing away from water plane, skipping.")
                            continue


                        # Solve for t (distance along the ray)
                        t = (self.water_z_in_base_frame - O_base[2]) / V_base_dir[2]

                        # Check if intersection is behind the camera origin (t < 0)
                        # This check might be redundant now given the check above, but keep for safety.
                        if t < 0:
                            # self.get_logger().debug(f"Intersection point is behind camera (t={t:.2f}), skipping.")
                            continue

                        # Calculate the intersection point in the base frame
                        P_intersect_base = O_base + t * V_base_dir

                        X_base = P_intersect_base[0]
                        Y_base = P_intersect_base[1]
                        # Explicitly set Z to the defined water plane level
                        Z_base = self.water_z_in_base_frame

                        # Assign ID for this frame
                        detection_counters[color_name] += 1
                        current_id = detection_counters[color_name]

                        # --- Publish Coordinates (Base Frame - Intersection Point) ---
                        point_msg_base = PointStamped()
                        point_msg_base.header.stamp = msg.header.stamp # Use same timestamp
                        point_msg_base.header.frame_id = self.base_frame_id # Set target frame
                        point_msg_base.point.x = X_base
                        point_msg_base.point.y = Y_base
                        point_msg_base.point.z = Z_base # Publish the EXPLICIT water plane Z

                        if color_name in self.coord_pubs_base:
                            self.coord_pubs_base[color_name].publish(point_msg_base)
                            self.get_logger().debug(f"Detected {color_name} ID {current_id} at base frame water intersection: ({X_base:.2f}, {Y_base:.2f}, {Z_base:.2f})")

                        # Add to list for annotation
                        detected_objects_this_frame.append({
                            'center': (center_u, center_v), 'color': color_name, 'id': current_id,
                            'coords_base': (X_base, Y_base, Z_base),
                            'contour': contour
                        })


        # --- Annotate Image (Optional) ---
        if self.annotated_image_pub.get_subscription_count() > 0:
            annotated_image = cv_image.copy()
            for obj in detected_objects_this_frame:
                 cv2.drawContours(annotated_image, [obj['contour']], -1, (0, 255, 0), 2)
                 cv2.circle(annotated_image, obj['center'], 5, (0, 0, 255), -1)
                 # Display ID and coordinates (base frame intersection)
                 text = f"{obj['color']}-{obj['id']} B:({obj['coords_base'][0]:.1f},{obj['coords_base'][1]:.1f},{obj['coords_base'][2]:.1f})"
                 cv2.putText(annotated_image, text, (obj['center'][0] + 10, obj['center'][1] + 10),
                             cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)

            try:
                 annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding="bgr8")
                 annotated_msg.header = msg.header
                 self.annotated_image_pub.publish(annotated_msg)
            except CvBridgeError as e:
                 self.get_logger().error(f"CV Bridge Error for annotated image: {e}")


def main(args=None):
    rclpy.init(args=args)
    try:
        color_detector_node = ColorObjectDetector()
        rclpy.spin(color_detector_node)
    except KeyboardInterrupt:
        print("Color Object Detector node shut down via KeyboardInterrupt.")
    except Exception as e:
        print(f"An error occurred: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'color_detector_node' in locals() and hasattr(color_detector_node, 'context') and color_detector_node.context.ok():
             color_detector_node.destroy_node()
        if rclpy.ok():
             rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()


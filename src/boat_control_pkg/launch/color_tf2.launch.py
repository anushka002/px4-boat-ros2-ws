import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
# import xacro # Removed this line as it's not currently used and causing an error

def generate_launch_description():

    # === Configure Paths ===
    pkg_name = 'boat_control_pkg' # CHANGE THIS TO YOUR PACKAGE NAME
    # urdf_file_subpath = 'urdf/boat.urdf' # Path to URDF relative to package share
    # Use xacro if your URDF is an xacro file
    urdf_file_subpath = 'urdf/boat.urdf' # CHANGE THIS if your file is different

    # === Load URDF ===
    # Construct the full path to the URDF file
    urdf_path = os.path.join(get_package_share_directory(pkg_name), urdf_file_subpath)

    # Read the URDF file content
    # Use xacro processing if needed:
    # robot_description_content = xacro.process_file(urdf_path).toxml()
    # Otherwise, just read the file:
    try:
        with open(urdf_path, 'r') as file:
            robot_description_content = file.read()
    except FileNotFoundError:
        print(f"Error: URDF file not found at {urdf_path}")
        # Handle error appropriately, maybe raise an exception or exit
        robot_description_content = "" # Set to empty string to avoid crashing robot_state_publisher immediately

    robot_description = {'robot_description': robot_description_content}

    # === Nodes ===
    # Robot State Publisher Node
    # Takes the URDF loaded via the 'robot_description' parameter and publishes
    # the static transforms (/tf_static) between links defined by fixed joints.
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description] # Pass the URDF content
    )

    # (Optional) Start your detector node in the same launch file
    color_detector_node = Node(
         package=pkg_name,
         executable='color_trash_detector', # Name from your setup.py entry point
         name='color_object_detector',      # Node name used in the code
         output='screen',
         # You can pass parameters here if needed, e.g.:
         # parameters=[
         #     {'camera_frame_id': 'camera_link_actual'},
         #     {'base_frame_id': 'base_link_actual'}
         # ]
    )


    # === Launch Description ===
    return LaunchDescription([
        robot_state_publisher_node,
        color_detector_node # Include this line if you want to launch the detector too
    ])


import os # Import os module
from glob import glob # Import glob module
from setuptools import find_packages, setup

package_name = 'boat_control_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add these lines to install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Add these lines to install URDF files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        # Add these lines to install mesh files
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.dae')), # Adjust extension if needed (e.g., *.stl)
        # Add other mesh types if needed:
        # (os.path.join('share', package_name, 'meshes'), glob('meshes/*.stl')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anushka-satav',
    maintainer_email='anushka-satav@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'boat_survey_node = boat_control_pkg.boat_survey_node:main',
            'yolo_trash_detector = boat_control_pkg.yolo_trash_detector:main',
            'color_trash_detector = boat_control_pkg.color_trash_detector:main', # This entry point is correct
            'minimal_control = boat_control_pkg.minimal_control:main',
            'velocity_control = boat_control_pkg.velocity_control:main',

        ],
    },
)

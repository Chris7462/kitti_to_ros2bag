from os.path import join

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = join(
        get_package_share_directory('kitti_to_ros2bag'), 'params', 'kitti_to_ros2bag.yaml'
    )
    kitti_to_ros2bag_node = Node(
        package='kitti_to_ros2bag',
        executable='kitti_to_ros2bag_node',
        name='kitti_to_ros2bag_node',
        output='screen',
        parameters=[params]
    )


    return LaunchDescription([
        kitti_to_ros2bag_node
    ])

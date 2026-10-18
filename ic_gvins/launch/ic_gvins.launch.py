import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('ic_gvins')
    default_config = os.path.join(package_share, 'config', 'kaist-urban39.yaml')
    configfile = LaunchConfiguration('configfile')
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'configfile',
            default_value=default_config,
            description='IC-GVINS YAML configuration file.',
        ),
        DeclareLaunchArgument('use_rviz', default_value='false',
                              description='Start rviz2 (requires the rviz2 package).'),

        Node(
            package='ic_gvins',
            executable='ic_gvins_ros',
            name='ic_gvins_node',
            output='screen',
            parameters=[{
                'imu_topic': '/imu0',
                'gnss_topic': '/gnss0',
                'image_topic': '/cam0',

                # TODO: i2Nav-Robot dataset
                # 'imu_topic': '/adi/adis16465/imu',
                # 'gnss_topic': '/novatel/oem7/fix',
                # 'image_topic': '/avt_camera/left/image',

                'configfile': configfile,
            }],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_map_broadcaster',
            arguments=['0', '0', '0', '-1', '0', '0', '0', 'map', 'world'],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='visualisation',
            output='log',
            condition=IfCondition(use_rviz),
            arguments=['-d', os.path.join(package_share, 'config', 'visualization.rviz')],
        ),
    ])

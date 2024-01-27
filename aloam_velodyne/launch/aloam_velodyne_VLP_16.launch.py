# Import necessary Python modules
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Define parameters
    params = {
        'scan_line': 16,
        'minimum_range': 0.3,
        'mapping_skip_frame': 1,
        'mapping_line_resolution': 0.2,
        'mapping_plane_resolution': 0.4,
    }

    # Create a LaunchDescription
    return LaunchDescription([
        # Set parameters
        DeclareLaunchArgument('scan_line', default_value=str(params['scan_line'])),
        DeclareLaunchArgument('minimum_range', default_value=str(params['minimum_range'])),
        DeclareLaunchArgument('mapping_skip_frame', default_value=str(params['mapping_skip_frame'])),
        DeclareLaunchArgument('mapping_line_resolution', default_value=str(params['mapping_line_resolution'])),
        DeclareLaunchArgument('mapping_plane_resolution', default_value=str(params['mapping_plane_resolution'])),

        # Launch nodes
        Node(
            package='aloam_velodyne',
            executable='ascanRegistration',
            name='ascanRegistration',
            output='screen',
            parameters=[{'scan_line': params['scan_line'],
				'minimum_range': params['minimum_range']}]
        ),
        Node(
            package='aloam_velodyne',
            executable='alaserOdometry',
            name='alaserOdometry',
            output='screen',
            parameters=[{'mapping_skip_frame': params['mapping_skip_frame']}]
        ),
        Node(
            package='aloam_velodyne',
            executable='alaserMapping',
            name='alaserMapping',
            output='screen',
            parameters=[{'mapping_line_resolution': params['mapping_line_resolution'],
				'mapping_plane_resolution': params['mapping_plane_resolution']}]
        ),
    ])

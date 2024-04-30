# Import necessary Python modules
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Define parameters
    params = {
        'publish_clouds': 'false',
        'scan_line': 16,
        'minimum_range': 0.3,
        'mapping_skip_frame': 5,
        'mapping_line_resolution': 0.2, # ORIG
        'mapping_plane_resolution': 0.4, # ORIG
    }

    # Create a LaunchDescription
    return LaunchDescription([
        # Set parameters
        DeclareLaunchArgument('publish_clouds', default_value=str(params['publish_clouds'])),
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
            parameters=[{'scan_line': LaunchConfiguration('scan_line'),
				'minimum_range': LaunchConfiguration('minimum_range')}],
            remappings=[('/velodyne_points','/point_cloud'),]
        ),
        Node(
            package='aloam_velodyne',
            executable='alaserOdometry',
            name='alaserOdometry',
            output='screen',
            parameters=[{'mapping_skip_frame': LaunchConfiguration('mapping_skip_frame')}]
        ),
        Node(
            package='aloam_velodyne',
            executable='alaserMapping',
            name='alaserMapping',
            output='screen',
            parameters=[{'publish_clouds': LaunchConfiguration('publish_clouds'),
                'mapping_line_resolution': LaunchConfiguration('mapping_line_resolution'),
		'mapping_plane_resolution': LaunchConfiguration('mapping_plane_resolution')}]
        ),
        Node(
            package='aloam_velodyne',
            executable='mappedPoseBroadcaster',
            name='mappedPoseBroadcaster',
            output='screen'
        ),
    ])

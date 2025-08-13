from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments (optional overrides)
        DeclareLaunchArgument(
            'csv_folder_path',
            default_value='/tmp',
            description='Path to save CSV logs'
        ),
        DeclareLaunchArgument(
            'pointcloud_topic',
            default_value='/velodyne/velodyne_points',
            description='Input point cloud topic'
        ),
        DeclareLaunchArgument(
            'resolution',
            default_value="0.01",
            description='Resolution for '
        ),


        # Compressor Node
        Node(
            package='cloudini_ros2',
            executable='compress',
            name='compress',
            parameters=[{
                'csv_folder_path': LaunchConfiguration('csv_folder_path'),
                'PC_topic': LaunchConfiguration('pointcloud_topic'),
                'resolution': LaunchConfiguration('resolution'),
            }],
            output='screen'
        ),

        # Decompressor Node
        Node(
            package='cloudini_ros2',
            executable='decompress',
            name='decompress',
            parameters=[{
                'csv_folder_path': LaunchConfiguration('csv_folder_path'),
            }],
            output='screen'
        ),

        # Stats Node
        Node(
            package='point_cloud_metrics',
            executable='pointcloud_metrics_node',
            name='pointcloud_metrics_node',
            parameters=[{
                'metrics_csv_path': LaunchConfiguration('csv_folder_path'),
                'cloud1_topic': LaunchConfiguration('pointcloud_topic'),              # Original pointcloud
                'cloud2_topic': '/decompressed_pointcloud_draco',         # Decompressed pointcloud
            }],
            output='screen'
        )
    ])

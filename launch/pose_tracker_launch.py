from launch.substitutions import PathJoinSubstitution
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        #static map transform
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_map_to_pelvis',
            arguments=[
                '0', '0', '0',            # translation: x y z
                '-0.5', '-0.5', '-0.5', '0.5',  # rotation: quaternion x y z w
                'map', 'pelvis'           # parent frame, child frame
            ]
        ),
        
        # PointCloud Transformer Node
        Node(
            package='camera_tf_transformer',
            executable='pointcloud_transformer',
            name='pointcloud_transformer',
            output='screen',
            parameters=[{
                'input_topic': '/pose_tracker/points',
                'output_topic': '/points_in_pelvis',
                'target_frame': 'pelvis'
            }]
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'robot_description': Command([
                    'cat ',
                    PathJoinSubstitution([
                        FindPackageShare('pose_tracker'),
                        'urdf',
                        'g1_29dof_lock_waist.urdf'
                    ])
                ])}
            ],
        ),

        # Joint State Publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),

        # Your Pose Tracker Node
        # Node(
        #     package='pose_tracker',
        #     executable='pose_tracker_node',
        #     name='pose_tracker',
        #     output='screen',
        # ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=[
                '-d',
                PathJoinSubstitution([
                    FindPackageShare('pose_tracker'),
                    'rviz',
                    'pose_tracker.rviz'
                ])
            ],
        ),
    ])

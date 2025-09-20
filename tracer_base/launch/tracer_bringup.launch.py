import os
import launch
import launch_ros

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 기존 tracer_base 관련 인자
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false',
                                             description='Use simulation clock if true')
    port_name_arg = DeclareLaunchArgument('port_name', default_value='can0',
                                         description='CAN bus name, e.g. can0')
    odom_frame_arg = DeclareLaunchArgument('odom_frame', default_value='odom',
                                           description='Odometry frame id')
    base_link_frame_arg = DeclareLaunchArgument('base_frame', default_value='base_link',
                                                description='Base link frame id')
    odom_topic_arg = DeclareLaunchArgument('odom_topic_name', default_value='odom',
                                           description='Odometry topic name')
    is_tracer_mini_arg = DeclareLaunchArgument('is_tracer_mini', default_value='false',
                                          description='Scout mini model')
    simulated_robot_arg = DeclareLaunchArgument('simulated_robot', default_value='false',
                                                   description='Whether running with simulator')
    sim_control_rate_arg = DeclareLaunchArgument('control_rate', default_value='50',
                                                 description='Simulation control loop update rate')

    # URDF 관련
    model_arg = LaunchConfiguration('model', default='')

    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([FindPackageShare('tracer_description'), 'urdf', 'tracer_v1.xacro'])
    ])

    # 노드들
    tracer_base_node = Node(
        package='tracer_base',
        executable='tracer_base_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'port_name': LaunchConfiguration('port_name'),                
                'odom_frame': LaunchConfiguration('odom_frame'),
                'base_frame': LaunchConfiguration('base_frame'),
                'odom_topic_name': LaunchConfiguration('odom_topic_name'),
                'is_tracer_mini': LaunchConfiguration('is_tracer_mini'),
                'simulated_robot': LaunchConfiguration('simulated_robot'),
                'control_rate': LaunchConfiguration('control_rate'),
        }]
    )

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_pub_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('tracer_description'),
            'rviz',
            'tracer_base.rviz'
        ])],
        output='screen'
    )

    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        port_name_arg,        
        odom_frame_arg,
        base_link_frame_arg,
        odom_topic_arg,
        is_tracer_mini_arg,
        simulated_robot_arg,
        sim_control_rate_arg,

        # Nodes
        tracer_base_node,
        robot_state_pub_node,
        joint_state_pub_gui_node,
        rviz_node
    ])


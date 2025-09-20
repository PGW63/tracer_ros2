from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # launch 인자
    model_arg = LaunchConfiguration('model', default='')

    # xacro를 이용해 URDF 생성
    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([FindPackageShare('tracer_description'), 'urdf', 'tracer_v1.xacro'])
    ])

    return LaunchDescription([
        # robot_description 파라미터 설정과 robot_state_publisher 노드 실행
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        # joint_state_publisher_gui 노드
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # RViz 실행
        #Node(
        #    package='rviz2',
        #    executable='rviz2',
        #    name='rviz2',
        #    arguments=['-d', PathJoinSubstitution([FindPackageShare('tracer_description'), 'rviz', 'model_display.rviz'])],
        #    output='screen'
        #)
    ])


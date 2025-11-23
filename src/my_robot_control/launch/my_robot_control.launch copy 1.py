from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Crear LaunchDescription
    ld = LaunchDescription()

    # Configuració inicial del robot
    x0 = LaunchConfiguration('x0', default='0.0')       # posició X inicial
    y0 = LaunchConfiguration('y0', default='0.0')       # posició Y inicial
    yaw0 = LaunchConfiguration('yaw0', default='0.0')   # orientació inicial (radians)

    # Node per fer spawn del robot al món de Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'rubot',
            '-file', '/home/user/ROS2_rUBot_mecanum_ws/src/my_robot_description/urdf/rubot_mecanum_custom2_lidar.urdf',
            '-x', x0,
            '-y', y0,
            '-Y', yaw0
        ],
        output='screen'
    )
    ld.add_action(spawn_robot)

    # Node del control del robot
    my_robot_control_node = Node(
        package="my_robot_control",
        executable="my_robot_control_exec",
        name="robot_control",
        parameters=[
            {"vx": 0.3},
            {"vy": 0.0},
            {"w": 0.0},
            {"td": 2.0}
        ]
    )
    ld.add_action(my_robot_control_node)

    return ld

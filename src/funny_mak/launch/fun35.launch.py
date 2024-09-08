from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument,ExecuteProcess,TimerAction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    launch_description = LaunchDescription()

    motorsim1_package_name = 'motorsim'

    motorsim1_node = Node(
        package=motorsim1_package_name,
        name='motorsim',
        namespace='',
        executable='motorsim_node.py',

    )
    launch_description.add_action(motorsim1_node)

    motor1_name = '/motor6504'

    crate_motor = TimerAction(
        period=0.1,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'service', 'call',  # The ros2 service call command
                    '/spawn_motor',              # The service name
                    'motorsim_interfaces/srv/MotorSpawn',  # The service type
                    f'{{"name": "{motor1_name}"}}'  # The arguments in JSON format
                ],
                output='screen'
            )
        ]
    )
    launch_description.add_action(crate_motor)

    funny_mak_package_name = 'funny_mak'

    encoder_node = Node(
        package=funny_mak_package_name,
        name='encoder',
        namespace='',
        executable='encoder_node.py',
    )
    launch_description.add_action(encoder_node)

    controller_node = Node(
        package=funny_mak_package_name,
        name='controller',
        namespace='',
        executable='controller_node.py',
    )
    launch_description.add_action(controller_node)

    return launch_description
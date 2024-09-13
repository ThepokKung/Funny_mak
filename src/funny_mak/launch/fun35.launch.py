from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument,ExecuteProcess,TimerAction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    launch_description = LaunchDescription()

    funny_mak_package_name = 'funny_mak'

    motorsim1_package_name = 'motorsim'

    # random_target_generator_node = Node(
    #     package=funny_mak_package_name,
    #     name='randomtarget',
    #     namespace='',
    #     executable='randomTargetGenerator_node.py',
    # )
    random_target_generator_node = TimerAction(
        period=0.3,
        actions=[
            Node(
                package=funny_mak_package_name,
                name='randomtarget',
                namespace='',
                executable='randomTargetGenerator_node.py',
            )
        ]
    )
    launch_description.add_action(random_target_generator_node)

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

    scheduler_node = Node(
        package=funny_mak_package_name,
        name='scheduler',
        namespace='',
        executable='scheduler_node.py',
    )
    launch_description.add_action(scheduler_node)

    return launch_description
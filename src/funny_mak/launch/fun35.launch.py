from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument,ExecuteProcess,TimerAction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    launch_description = LaunchDescription()
    #Define packgage Name
    funny_mak_package_name = 'funny_mak'
    motorsim1_package_name = 'motorsim'

    #Define Parameter
    #Random Tartget gen
    num_targets = 20
    target_min = 0
    target_max = 360
    motor1_ns = 'kraiwich'
    yaml_file_path = '~/fun3.5_ws/src/funny_mak/config/via_point.yaml'

    #Yaml param

    random_target_generator_node = Node(
        package=funny_mak_package_name,
        name='randomtarget',
        namespace='',
        executable='randomTargetGenerator_node.py',
        parameters=[
            {'num_targets': num_targets},
            {'target_min' : target_min},
            {'target_max': target_max},
            {'file_yaml_path': yaml_file_path}
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

    crate_motor = TimerAction(
        period=0.1,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'service', 'call',  # The ros2 service call command
                    '/spawn_motor',              # The service name
                    'motorsim_interfaces/srv/MotorSpawn',  # The service type
                    f'{{"name": "/{motor1_ns}"}}'  # The arguments in JSON format
                ],
                output='screen'
            )
        ]
    )
    launch_description.add_action(crate_motor)

    encoder_node = Node(
        package=funny_mak_package_name,
        name='encoder',
        namespace=motor1_ns,
        executable='encoder_node.py',
    )
    launch_description.add_action(encoder_node)

    controller_node = Node(
        package=funny_mak_package_name,
        name='controller',
        namespace=motor1_ns,
        executable='controller_node.py',
    )
    launch_description.add_action(controller_node)

    scheduler_node = Node(
        package=funny_mak_package_name,
        name='scheduler',
        namespace='',
        executable='scheduler_node.py',
        parameters=[
            {'file_yaml_path': yaml_file_path}
        ]
    )
    launch_description.add_action(scheduler_node)

    return launch_description
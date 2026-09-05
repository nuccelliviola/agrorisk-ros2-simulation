"""
Launch file per l'avvio del mondo Webots e dei controller ROS2
associati al drone e al rover tramite webots_ros2_driver.
"""
import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('agro_webots')
    world = os.path.join(package_dir, 'worlds', 'agro_field.wbt')

    webots = WebotsLauncher(world=world)

    robot_description_path = os.path.join(
        package_dir, 'resource', 'drone_webots.urdf')
    drone_driver = WebotsController(
        robot_name='drone_1',
        parameters=[{'robot_description': robot_description_path}],
    )

    rover_robot_description_path = os.path.join(
        package_dir, 'resource', 'rover_webots.urdf')
    rover_driver = WebotsController(
        robot_name='rover_1',
        parameters=[{'robot_description': rover_robot_description_path}],
    )

    return LaunchDescription([
        webots,
        drone_driver,
        rover_driver,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=webots,
                on_exit=[EmitEvent(event=Shutdown())],
            )
        ),
    ])

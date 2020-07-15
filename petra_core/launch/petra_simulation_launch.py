import launch.substitutions
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo

def generate_launch_description():

    emit_shutdown_action = launch.actions.Shutdown(reason='launch is shutting down')

    return LaunchDescription([
        Node(
            package='petra_drivers',
            node_executable='Screen',
            node_name='Screen',
            output='screen'
        ),
        #Node(
        #    package='petra_drivers',
        #    node_executable='Keyboard',
        #    node_name='Keyboard',
        #    output='screen'
        #),
        #Node(
        #    package='petra_drivers',
        #    node_executable='RobotDummy',
        #    node_name='RobotDummy'
        #),
        Node(
            package='petra_services',
            node_executable='Communication',
            node_name='Communication'
        ),
        Node(
            package='petra_services',
            node_executable='Navigation2Dummy',
            node_name='Navigation2Dummy'
        ),
        Node(
            package='petra_services',
            node_executable='PatientMonitoringDummy',
            node_name='PatientMonitoringDummy'
        ),
        Node(
            package='petra_drivers',
            node_executable='PairingModuleDummy',
            node_name='PairingModuleDummy'
        ),
        Node(
            package='petra_drivers',
            node_executable='ManipulatorDummy',
            node_name='ManipulatorDummy'
        ),
        Node(
            package='petra_drivers',
            node_executable='BatteryDummy',
            node_name='BatteryDummy'
        ),
        Node(
            package='petra_central_control',
            node_executable='CCU',
            node_name='CCU',
            output='screen',
            on_exit=[LogInfo(msg=["CCU has stopped. Stopping everything..."]), emit_shutdown_action],
        )
    ])

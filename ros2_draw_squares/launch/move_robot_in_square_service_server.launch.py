from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='ros2_draw_squares', executable='move_robot_service_server', output='screen'),
    ])

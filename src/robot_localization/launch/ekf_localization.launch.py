import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command


def generate_launch_description():
    # Arguments
    frequency_arg = DeclareLaunchArgument(
        "frequency",
        default_value="30",
        description="Frequency of the EKF localization node",
    )
    sensor_timeout_arg = DeclareLaunchArgument(
        "sensor_timeout",
        default_value="0.1",
        description="Timeout duration for sensor data",
    )
    two_d_mode_arg = DeclareLaunchArgument(
        "two_d_mode",
        default_value="true",
        description="Enable 2D mode for EKF localization",
    )

    # EKF Localization Node
    ekf_localization_node = ExecuteProcess(
        cmd=[Command("ros2", "run", "robot_localization", "ekf_localization_node")],
        output="screen",
        additional_env={"ROS_NAMESPACE": LaunchConfiguration("namespace")},
    )

    # Load EKF Parameters
    load_params_action = ExecuteProcess(
        cmd=[Command("ros2", "param", "load", LaunchConfiguration("params_file"))],
        output="screen",
    )

    return LaunchDescription(
        [
            frequency_arg,
            sensor_timeout_arg,
            two_d_mode_arg,
            ekf_localization_node,
            load_params_action,
        ]
    )


if __name__ == "__main__":
    ld = generate_launch_description()
    launch.logging.get_logger("launch").addHandler(launch.logging.NullHandler())
    launch.logging.get_logger("launch.frontend").addHandler(
        launch.logging.NullHandler()
    )
    launch.LaunchService().include_launch_description(ld).run()

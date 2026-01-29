from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, RegisterEventHandler, Shutdown
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

launch_args = [
    DeclareLaunchArgument(name="namespace", default_value="ov_msckf", description="namespace"),
    DeclareLaunchArgument(name="ov_enable", default_value="true", description="enable OpenVINS node"),
    DeclareLaunchArgument(name="rviz_enable", default_value="true", description="enable rviz node"),
    DeclareLaunchArgument(
        name="verbosity",
        default_value="INFO",
        description="ALL, DEBUG, INFO, WARNING, ERROR, SILENT",
    ),
    DeclareLaunchArgument(
        name="config",
        default_value="tum_vi",
        description="euroc_mav, tum_vi, rpng_aruco, kaist",
    ),
    DeclareLaunchArgument(
        name="config_path",
        default_value="",
        description="path to estimator_config.yaml. If not given, determined based on provided 'config' above",
    ),
    DeclareLaunchArgument(
        name="max_cameras",
        default_value="2",
        description="how many cameras we have 1 = mono, 2 = stereo, >2 = binocular (all mono tracking)",
    ),
    DeclareLaunchArgument(
        name="use_stereo",
        default_value="true",
        description="if we have more than 1 camera, if we should try to track stereo constraints between pairs",
    ),
    DeclareLaunchArgument(
        name="bag_start",
        default_value="0.0",
        description="time offset (sec) into bag to start",
    ),
    DeclareLaunchArgument(
        name="bag_durr",
        default_value="-1.0",
        description="duration (sec) to run, <0 means to the end of the bag",
    ),
    DeclareLaunchArgument(
        name="dataset",
        default_value="dataset-room1_512_16",
        description="dataset name",
    ),
    DeclareLaunchArgument(
        name="bag",
        default_value=[
            TextSubstitution(text="/home/patrick/datasets/"),
            LaunchConfiguration("config"),
            TextSubstitution(text="/"),
            LaunchConfiguration("dataset"),
        ],
        description="path to rosbag2 directory",
    ),
    DeclareLaunchArgument(
        name="path_gt",
        default_value="",
        description="path to gt csv file (optional)",
    ),
    DeclareLaunchArgument(
        name="record_timing_information",
        default_value="false",
        description="record timing info",
    ),
    DeclareLaunchArgument(
        name="record_timing_filepath",
        default_value="/tmp/traj_timing.txt",
        description="timing output file",
    ),
]


def launch_setup(context):
    config_path = LaunchConfiguration("config_path").perform(context)
    if not config_path:
        configs_dir = os.path.join(get_package_share_directory("ov_msckf"), "config")
        available_configs = os.listdir(configs_dir)
        config = LaunchConfiguration("config").perform(context)
        if config in available_configs:
            config_path = os.path.join(
                get_package_share_directory("ov_msckf"),
                "config",
                config,
                "estimator_config.yaml",
            )
        else:
            return [
                LogInfo(
                    msg="ERROR: unknown config: '{}' - Available configs are: {} - not starting OpenVINS".format(
                        config, ", ".join(available_configs)
                    )
                )
            ]
    else:
        if not os.path.isfile(config_path):
            return [
                LogInfo(
                    msg="ERROR: config_path file: '{}' - does not exist. - not starting OpenVINS".format(
                        config_path
                    )
                )
            ]

    node1 = Node(
        package="ov_msckf",
        executable="ros2_serial_msckf",
        condition=IfCondition(LaunchConfiguration("ov_enable")),
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[
            {"verbosity": LaunchConfiguration("verbosity")},
            {"config_path": config_path},
            {"use_stereo": LaunchConfiguration("use_stereo")},
            {"max_cameras": LaunchConfiguration("max_cameras")},
            {"path_bag": LaunchConfiguration("bag")},
            {"bag_start": LaunchConfiguration("bag_start")},
            {"bag_durr": LaunchConfiguration("bag_durr")},
            {"path_gt": LaunchConfiguration("path_gt")},
            {"record_timing_information": LaunchConfiguration("record_timing_information")},
            {"record_timing_filepath": LaunchConfiguration("record_timing_filepath")},
        ],
    )

    shutdown_on_exit = RegisterEventHandler(
        OnProcessExit(target_action=node1, on_exit=[Shutdown()])
    )

    node2 = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(LaunchConfiguration("rviz_enable")),
        arguments=[
            "-d"
            + os.path.join(
                get_package_share_directory("ov_msckf"), "launch", "display_ros2.rviz"
            ),
            "--ros-args",
            "--log-level",
            "warn",
        ],
    )

    return [node1, node2, shutdown_on_exit]


def generate_launch_description():
    opfunc = OpaqueFunction(function=launch_setup)
    ld = LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld

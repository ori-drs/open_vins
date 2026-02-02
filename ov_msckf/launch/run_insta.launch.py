from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import os
import sys

launch_args = [
    DeclareLaunchArgument(
        name="bag",
        default_value="",
        description="path to ros2 bag to play (if empty, do not play)",
    ),
    DeclareLaunchArgument(
        name="bag_rate",
        default_value="1.0",
        description="ros2 bag play rate (1.0 = realtime speed)",
    ),
    DeclareLaunchArgument(name="namespace", default_value="ov_msckf", description="namespace"),
    DeclareLaunchArgument(
        name="ov_enable", default_value="true", description="enable OpenVINS node"
    ),
    DeclareLaunchArgument(
        name="rviz_enable", default_value="true", description="enable rviz node"
    ),
    DeclareLaunchArgument(
        name="config",
        default_value="insta_oxford",
        description="euroc_mav, tum_vi, rpng_aruco...",
    ),
    DeclareLaunchArgument(
        name="config_path",
        default_value="",
        description="path to estimator_config.yaml. If not given, determined based on provided 'config' above",
    ),
    DeclareLaunchArgument(
        name="verbosity",
        default_value="INFO",
        description="ALL, DEBUG, INFO, WARNING, ERROR, SILENT",
    ),
    DeclareLaunchArgument(
        name="use_stereo",
        default_value="false",
        description="if we have more than 1 camera, if we should try to track stereo constraints between pairs",
    ),
    DeclareLaunchArgument(
        name="max_cameras",
        default_value="2",
        description="how many cameras we have 1 = mono, 2 = stereo, >2 = binocular (all mono tracking)",
    ),
    DeclareLaunchArgument(
        name="save_total_state",
        default_value="false",
        description="record the total state with calibration and features to a txt file",
    ),
    DeclareLaunchArgument(
        name="filepath_est",
        default_value="/tmp/ov_estimate.txt",
        description="output path for full state estimate",
    ),
    DeclareLaunchArgument(
        name="filepath_std",
        default_value="/tmp/ov_estimate_std.txt",
        description="output path for state standard deviation",
    ),
    DeclareLaunchArgument(
        name="filepath_gt",
        default_value="/tmp/ov_groundtruth.txt",
        description="output path for groundtruth state",
    ),
    DeclareLaunchArgument(
        name="filepath_odom",
        default_value="/tmp/ov_odometry.txt",
        description="output path for odom in TUM format",
    ),
    DeclareLaunchArgument(
        name="image_conversion",
        default_value="true",
        description="Enable compressed-to-image conversion node"
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
                            "config",config,"estimator_config.yaml"
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
                        config_path)
                    )
            ]
    node1 = Node(
        package="ov_msckf",
        executable="run_subscribe_msckf",
        condition=IfCondition(LaunchConfiguration("ov_enable")),
        namespace=LaunchConfiguration("namespace"),
        output='screen',
        parameters=[
            {"verbosity": LaunchConfiguration("verbosity")},
            {"use_stereo": LaunchConfiguration("use_stereo")},
            {"max_cameras": LaunchConfiguration("max_cameras")},
            {"config_path": config_path},
            {"save_total_state": LaunchConfiguration("save_total_state")},
            {"filepath_est": LaunchConfiguration("filepath_est")},
            {"filepath_std": LaunchConfiguration("filepath_std")},
            {"filepath_gt": LaunchConfiguration("filepath_gt")},
            {"filepath_odom": LaunchConfiguration("filepath_odom")},
        ],
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

    node3 = Node(
        package="challenge_tools_ros",
        executable="image_conversion_node.py",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        condition=IfCondition(LaunchConfiguration("image_conversion")),
        arguments=[
            "/cam0/image_raw/compressed",
            "/cam1/image_raw/compressed",
            "/cam0/image_raw",
            "/cam1/image_raw",
        ],
    )

    bag_play = ExecuteProcess(
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration("bag"), "' != ''"])),
        cmd=[
            "ros2",
            "bag",
            "play",
            LaunchConfiguration("bag"),
            "--clock",
            "--rate",
            LaunchConfiguration("bag_rate"),
        ],
        output="screen",
    )

    return [node1, node2, node3, bag_play]


def generate_launch_description():
    opfunc = OpaqueFunction(function=launch_setup)
    ld = LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld

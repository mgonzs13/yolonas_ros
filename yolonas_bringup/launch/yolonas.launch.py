# Copyright (C) 2023  Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    #
    # ARGS
    #
    model = LaunchConfiguration("model")
    model_cmd = DeclareLaunchArgument(
        "model",
        default_value="yolo_nas_s",
        description="Model name or path")

    pretrained_weights = LaunchConfiguration("pretrained_weights")
    pretrained_weights_cmd = DeclareLaunchArgument(
        "pretrained_weights",
        default_value="coco",
        description="Pretrained weights (COCO, imagenet)")

    num_classes = LaunchConfiguration("num_classes")
    num_classes_cmd = DeclareLaunchArgument(
        "num_classes",
        default_value="-1",
        description="")

    checkpoint_path = LaunchConfiguration("checkpoint_path")
    checkpoint_path_cmd = DeclareLaunchArgument(
        "checkpoint_path",
        default_value="",
        description="Path to checkpoint file")

    device = LaunchConfiguration("device")
    device_cmd = DeclareLaunchArgument(
        "device",
        default_value="cuda:0",
        description="Device to use (GPU/CPU)")

    enable = LaunchConfiguration("enable")
    enable_cmd = DeclareLaunchArgument(
        "enable",
        default_value="True",
        description="Wheter to start darknet enabled")

    threshold = LaunchConfiguration("threshold")
    threshold_cmd = DeclareLaunchArgument(
        "threshold",
        default_value="0.5",
        description="Minimum probability of a detection to be published")

    input_image_topic = LaunchConfiguration("input_image_topic")
    input_image_topic_cmd = DeclareLaunchArgument(
        "input_image_topic",
        default_value="/camera/rgb/image_raw",
        description="Name of the input image topic")

    namespace = LaunchConfiguration("namespace")
    namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="yolo",
        description="Namespace for the nodes")

    #
    # NODES
    #
    detector_node_cmd = Node(
        package="yolonas_ros",
        executable="yolonas_node",
        name="yolonas_node",
        namespace=namespace,
        parameters=[{
            "model": model,
            "pretrained_weights": pretrained_weights,
            "num_classes": num_classes,
            "checkpoint_path": checkpoint_path,
            "device": device,
            "enable": enable,
            "threshold": threshold
        }],
        remappings=[("image_raw", input_image_topic)]
    )

    ld = LaunchDescription()

    ld.add_action(model_cmd)
    ld.add_action(pretrained_weights_cmd)
    ld.add_action(num_classes_cmd)
    ld.add_action(checkpoint_path_cmd)
    ld.add_action(device_cmd)
    ld.add_action(enable_cmd)
    ld.add_action(threshold_cmd)
    ld.add_action(input_image_topic_cmd)
    ld.add_action(namespace_cmd)

    ld.add_action(detector_node_cmd)

    return ld

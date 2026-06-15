from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Launch YOLO classifier node as composable lifecycle node."""

    model_path_arg = DeclareLaunchArgument(
        "model_path",
        default_value="/home/dingxiaoyi/workspace/cangyirobot/src/perception/yolo_detector/models/yolo26n.onnx",
        description="Path to ONNX model file",
    )
    labels_path_arg = DeclareLaunchArgument(
        "labels_path",
        default_value="/home/dingxiaoyi/workspace/cangyirobot/src/perception/yolo_detector/models/coco.names",
        description="Path to class names file",
    )
    use_gpu_arg = DeclareLaunchArgument(
        "use_gpu",
        default_value="true",
        description="Enable GPU inference",
    )
    publish_debug_image_arg = DeclareLaunchArgument(
        "publish_debug_image",
        default_value="true",
        description="Publish debug image topic",
    )
    image_topic_arg = DeclareLaunchArgument(
        "image_topic",
        default_value="/camera/color/image_raw",
        description="Input image topic",
    )
    container_name_arg = DeclareLaunchArgument(
        "container_name",
        default_value="yolo_detector_container",
        description="Composable node container name",
    )

    container = ComposableNodeContainer(
        name=LaunchConfiguration("container_name"),
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="yolo_detector",
                plugin="perception::YolosClassifierNode",
                name="yolos_classifier",
                parameters=[{
                    "model_path": LaunchConfiguration("model_path"),
                    "labels_path": LaunchConfiguration("labels_path"),
                    "use_gpu": LaunchConfiguration("use_gpu"),
                    "publish_debug_image": LaunchConfiguration("publish_debug_image"),
                }],
                remappings=[
                    ("/camera/color/image_raw", LaunchConfiguration("image_topic")),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
    )

    return LaunchDescription([
        model_path_arg,
        labels_path_arg,
        use_gpu_arg,
        publish_debug_image_arg,
        image_topic_arg,
        container_name_arg,
        container,
        LogInfo(msg=["[classifier.launch] model_path=", LaunchConfiguration("model_path")]),
        LogInfo(msg=["[classifier.launch] labels_path=", LaunchConfiguration("labels_path")]),
        LogInfo(msg=["[classifier.launch] use_gpu=", LaunchConfiguration("use_gpu")]),
        LogInfo(msg=["[classifier.launch] publish_debug_image=", LaunchConfiguration("publish_debug_image")]),
        LogInfo(msg=["[classifier.launch] image_topic=", LaunchConfiguration("image_topic")]),
        LogInfo(msg=["[classifier.launch] container_name=", LaunchConfiguration("container_name")]),
    ])
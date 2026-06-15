from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LifecycleNode
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Launch YOLO detector node as composable lifecycle node."""
    
    # Declare launch arguments
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value="/home/dingxiaoyi/workspace/cangyirobot/src/perception/yolo_detector/models/yolo26n.onnx",
        description='Path to ONNX model file'
    )
    labels_path_arg = DeclareLaunchArgument(
        'labels_path',
        default_value="/home/dingxiaoyi/workspace/cangyirobot/src/perception/yolo_detector/models/coco.names",
        description='Path to class names file'
    )
    use_gpu_arg = DeclareLaunchArgument(
        'use_gpu',
        default_value='true',
        description='Enable GPU inference'
    )
    conf_threshold_arg = DeclareLaunchArgument(
        'conf_threshold',
        default_value='0.4',
        description='Confidence threshold'
    )
    nms_threshold_arg = DeclareLaunchArgument(
        'nms_threshold',
        default_value='0.45',
        description='NMS threshold'
    )
    publish_debug_image_arg = DeclareLaunchArgument(
        'publish_debug_image',
        default_value='true',
        description='Publish debug image topic'
    )
    publish_timing_arg = DeclareLaunchArgument(
        'publish_timing',
        default_value='true',
        description='Publish timing information'
    )
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/color/image_raw',
        description='Input image topic'
    )
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/camera/color/camera_info',
        description='Camera info topic'
    )

    container_name_arg = DeclareLaunchArgument(
        "container_name",
        default_value="yolo_detector_container",
        description="Composable node container name",
    )

    # Composable node container (multi-threaded for async inference)
    container = ComposableNodeContainer(
        name=LaunchConfiguration("container_name"),
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='yolo_detector',
                plugin='perception::YolosDetectorNode',
                name='yolos_detector',
                parameters=[{
                    'model_path': LaunchConfiguration('model_path'),
                    'labels_path': LaunchConfiguration('labels_path'),
                    'use_gpu': LaunchConfiguration('use_gpu'),
                    'conf_threshold': LaunchConfiguration('conf_threshold'),
                    'nms_threshold': LaunchConfiguration('nms_threshold'),
                    'publish_debug_image': LaunchConfiguration('publish_debug_image'),
                    'publish_timing': LaunchConfiguration('publish_timing'),
                }],
                remappings=[
                    ('/camera/color/image_raw', LaunchConfiguration('image_topic')),
                    ('/camera/color/camera_info', LaunchConfiguration('camera_info_topic')),
                ],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        model_path_arg,
        labels_path_arg,
        use_gpu_arg,
        conf_threshold_arg,
        nms_threshold_arg,
        publish_debug_image_arg,
        publish_timing_arg,
        image_topic_arg,
        camera_info_topic_arg,
        container_name_arg,
        container,
        LogInfo(msg=["[detector.launch] model_path=", LaunchConfiguration("model_path")]),
        LogInfo(msg=["[detector.launch] labels_path=", LaunchConfiguration("labels_path")]),
        LogInfo(msg=["[detector.launch] use_gpu=", LaunchConfiguration("use_gpu")]),
        LogInfo(msg=["[detector.launch] conf_threshold=", LaunchConfiguration("conf_threshold")]),
        LogInfo(msg=["[detector.launch] nms_threshold=", LaunchConfiguration("nms_threshold")]),
        LogInfo(msg=["[detector.launch] publish_debug_image=", LaunchConfiguration("publish_debug_image")]),
        LogInfo(msg=["[detector.launch] publish_timing=", LaunchConfiguration("publish_timing")]),
        LogInfo(msg=["[detector.launch] image_topic=", LaunchConfiguration("image_topic")]),
        LogInfo(msg=["[detector.launch] camera_info_topic=", LaunchConfiguration("camera_info_topic")]),
    ])

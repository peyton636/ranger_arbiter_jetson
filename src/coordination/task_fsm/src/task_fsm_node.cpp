#include "task_fsm_node.hpp"

namespace coordination
{

    TaskFsmNode::TaskFsmNode(const rclcpp::NodeOptions &options)
        : rclcpp_lifecycle::LifecycleNode("task_fsm_node", options)
    {
        RCLCPP_INFO(get_logger(), "TaskFsmNode created");
        declareParameters();
    }

    void TaskFsmNode::declareParameters()
    {
        declare_parameter("plan_request_topic", "/motion/plan_request");
        declare_parameter("execute_feedback_topic", "/motion/execute_feedback");
        declare_parameter("gripper_cmd_topic", "/task/gripper_cmd");
        declare_parameter("gripper_status_topic", "/gripper/status");
        declare_parameter("object_pose_topic", "/perception/object_pose_array");
        declare_parameter("grasp_pose_topic", "/grasp/selected_pose");
        declare_parameter("nav_goal_topic", "/goal_pose");
        declare_parameter("nav_arrived_topic", "/task/nav_arrived");
        declare_parameter("output_frame", "base_link");
        declare_parameter("arm_group", "arm");
        declare_parameter("skip_navigation", true);
        declare_parameter("velocity_scaling", 0.15);
        declare_parameter("acceleration_scaling", 0.15);
        declare_parameter("approach_dist_m", 0.02);
        declare_parameter("approach_step_m", 0.005);
        declare_parameter("place_retreat_dist_m", 0.03);
        declare_parameter("named_home", "home");
        declare_parameter("named_grasp_ready", "grasp_ready");
        declare_parameter("named_pre_grasp", "pre_grasp");
        declare_parameter("named_retreat", "retreat");
        declare_parameter("max_perception_retries", 3);
        declare_parameter("max_motion_retries", 2);
        declare_parameter("max_gripper_retries", 2);
        declare_parameter("max_navigation_retries", 2);
        declare_parameter("tick_rate_hz", 10.0);
    }

    PickPlacePolicyConfig TaskFsmNode::loadPolicyConfig()
    {
        PickPlacePolicyConfig cfg;
        cfg.arm_group = get_parameter("arm_group").as_string();
        cfg.frame_id = get_parameter("output_frame").as_string();
        cfg.named_home = get_parameter("named_home").as_string();
        cfg.named_grasp_ready = get_parameter("named_grasp_ready").as_string();
        cfg.named_pre_grasp = get_parameter("named_pre_grasp").as_string();
        cfg.named_retreat = get_parameter("named_retreat").as_string();
        cfg.velocity_scaling = get_parameter("velocity_scaling").as_double();
        cfg.acceleration_scaling = get_parameter("acceleration_scaling").as_double();
        cfg.approach_dist_m = get_parameter("approach_dist_m").as_double();
        cfg.approach_step_m = get_parameter("approach_step_m").as_double();
        cfg.place_retreat_dist_m = get_parameter("place_retreat_dist_m").as_double();
        cfg.skip_navigation = get_parameter("skip_navigation").as_bool();
        cfg.retry.max_perception_retries = get_parameter("max_perception_retries").as_int();
        cfg.retry.max_motion_retries = get_parameter("max_motion_retries").as_int();
        cfg.retry.max_gripper_retries = get_parameter("max_gripper_retries").as_int();
        cfg.retry.max_navigation_retries = get_parameter("max_navigation_retries").as_int();
        return cfg;
    }

    TaskFsmNode::CallbackReturn TaskFsmNode::on_configure(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Configuring TaskFsmNode...");
        policy_config_ = loadPolicyConfig();
        policy_ = std::make_unique<PickPlacePolicy>(policy_config_);

        motion_client_ = std::make_shared<MotionClient>(
            *this,
            get_parameter("plan_request_topic").as_string(),
            get_parameter("execute_feedback_topic").as_string());
        gripper_client_ = std::make_shared<GripperClient>(
            *this,
            get_parameter("gripper_cmd_topic").as_string(),
            get_parameter("gripper_status_topic").as_string());
        navigation_client_ = std::make_shared<NavigationClient>(
            *this,
            get_parameter("nav_goal_topic").as_string(),
            get_parameter("nav_arrived_topic").as_string(),
            policy_config_.skip_navigation);
        perception_handler_ = std::make_shared<PerceptionHandler>(
            *this,
            get_parameter("object_pose_topic").as_string(),
            get_parameter("grasp_pose_topic").as_string(),
            get_parameter("output_frame").as_string());

        state_machine_ = std::make_unique<StateMachine>(
            *this,
            *policy_,
            motion_client_,
            gripper_client_,
            navigation_client_,
            perception_handler_);

        start_srv_ = create_service<std_srvs::srv::Trigger>(
            "~/start_pick_place",
            [this](
                const std_srvs::srv::Trigger::Request::SharedPtr,
                std_srvs::srv::Trigger::Response::SharedPtr response)
            {
                state_machine_->requestStartPickPlace();
                response->success = true;
                response->message = "pick-place task started";
            });
        cancel_srv_ = create_service<std_srvs::srv::Trigger>(
            "~/cancel",
            [this](
                const std_srvs::srv::Trigger::Request::SharedPtr,
                std_srvs::srv::Trigger::Response::SharedPtr response)
            {
                state_machine_->requestCancel();
                response->success = true;
                response->message = "cancel requested";
            });
        reset_srv_ = create_service<std_srvs::srv::Trigger>(
            "~/reset",
            [this](
                const std_srvs::srv::Trigger::Request::SharedPtr,
                std_srvs::srv::Trigger::Response::SharedPtr response)
            {
                state_machine_->requestReset();
                response->success = true;
                response->message = "reset requested";
            });

        RCLCPP_INFO(get_logger(), "TaskFsmNode configured");
        return CallbackReturn::SUCCESS;
    }

    TaskFsmNode::CallbackReturn TaskFsmNode::on_activate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Activating TaskFsmNode...");
        const double hz = get_parameter("tick_rate_hz").as_double();
        const auto period = std::chrono::duration<double>(1.0 / std::max(hz, 1.0));
        tick_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&TaskFsmNode::onTick, this));
        RCLCPP_INFO(get_logger(), "TaskFsmNode activated");
        return CallbackReturn::SUCCESS;
    }

    TaskFsmNode::CallbackReturn TaskFsmNode::on_deactivate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Deactivating TaskFsmNode...");
        tick_timer_.reset();
        return CallbackReturn::SUCCESS;
    }

    TaskFsmNode::CallbackReturn TaskFsmNode::on_cleanup(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Cleaning up TaskFsmNode...");
        tick_timer_.reset();
        start_srv_.reset();
        cancel_srv_.reset();
        reset_srv_.reset();
        state_machine_.reset();
        perception_handler_.reset();
        navigation_client_.reset();
        gripper_client_.reset();
        motion_client_.reset();
        policy_.reset();
        return CallbackReturn::SUCCESS;
    }

    TaskFsmNode::CallbackReturn TaskFsmNode::on_shutdown(const rclcpp_lifecycle::State &state)
    {
        return on_cleanup(state);
    }

    void TaskFsmNode::onTick()
    {
        if (state_machine_)
        {
            state_machine_->tick();
        }
    }

} // namespace coordination

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(coordination::TaskFsmNode)

#include "motion_planner_node.hpp"

#include <algorithm>
#include <cmath>
#include <thread>

#include "rclcpp_components/register_node_macro.hpp"

namespace manipulation
{

MotionPlannerNode::MotionPlannerNode(const rclcpp::NodeOptions & options)
:rclcpp_lifecycle::LifecycleNode("agx_motion_planner_node", options)
{
    RCLCPP_INFO(get_logger(), "agx_motion_planner_node created");

    //构造函数里只做轻量级工作：参数声明
    declareParameters();
}

void MotionPlannerNode::declareParameters()
{
    //moveit规划组名称，arm和gripper
    declare_parameter<std::string>("default_arm_group","arm");

    //接受上层规划请求
    declare_parameter<std::string>("plan_request_topic", "/motion/plan_request");

    //接收视觉节点输出的抓取位姿
    declare_parameter<std::string>("grasp_pose_topic", "/grasp/selected_pose");

    //关节状态话题
    declare_parameter<std::string>("joint_states_topic", "/joint_states");

    //输出给arm_controller_node的轨迹
    declare_parameter<std::string>("trajectory_topic", "/motion/trajectory");

    //给上层任务节点/app的执行轨迹返回结果
    declare_parameter<std::string>("execute_feedback_topic","/motion/execute_feedback");

    //arm_controller_node执行完轨迹后返回结果
    declare_parameter<std::string>("execute_result_topic", "/motion/execute_result");

    //等待机械臂执行结果的超时时间
    declare_parameter<double>("execute_timeout_sec", 120.0);

    default_arm_group_ = get_parameter("default_arm_group").as_string();
    execute_timeout_sec_ = get_parameter("execute_timeout_sec").as_double();  
}

MotionPlannerNode::CallbackReturn MotionPlannerNode::on_configure(const rclcpp_lifecycle::State & state)
{
    (void)state;
    RCLCPP_INFO(get_logger(), "configuring motion planner node");

    //创建发布器
    feedback_pub_ = create_publisher<ExecuteFeedback>(
        get_parameter("execute_feedback_topic").as_string(),10);
    
    trajectory_pub_ = create_publisher<MotionTrajectory>(
        get_parameter("trajectory_topic").as_string(),10);

    //创建完之后再判断是否为空
    if(!feedback_pub_ || !trajectory_pub_){
        RCLCPP_ERROR(get_logger(), "failed to create publishers");
        return CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(get_logger(), "motion planner configured");
    return CallbackReturn::SUCCESS;
}

MotionPlannerNode::CallbackReturn MotionPlannerNode::on_activate(const rclcpp_lifecycle::State & state)
{
    (void)state;

    RCLCPP_INFO(get_logger(), "activating motion planner node");

    //lifecyclepublisher需要手动激活，否则publish可能不会真正发出去
    feedback_pub_->on_activate();
    trajectory_pub_->on_activate();

    //订阅arm_controller_node的执行结果
    execute_result_sub_ = create_subscription<ExecuteResult>(
        get_parameter("execute_result_topic").as_string(),
        10,
        [this](const ExecuteResult::SharedPtr msg)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            //缓存最新执行结果
            last_execute_result_ = *msg;
            //唤醒waiForExecuteResult()
            execute_result_cv_.notify_all();
        });

    //订阅关节状态，当前只是缓存，主要用于调试或后期扩展
    joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        get_parameter("joint_states_topic").as_string(),
        10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            latest_joint_states_ = *msg;
        });

    //订阅视觉节点提供的抓取位姿
    grasp_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        get_parameter("grasp_pose_topic").as_string(),
        10,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            latest_grasp_pose_ = *msg;
        });

    //订阅规划请求
    plan_request_sub_ = create_subscription<PlanRequest>(
        get_parameter("plan_request_topic").as_string(),
        10,
        [this](const PlanRequest::SharedPtr msg)
        {
            std::thread([this, msg]()
            {
                onPlanRequest(msg);
            }).detach();
        });

    RCLCPP_INFO(
        get_logger(),
        "motion planner activate, defaut group:%s",
        default_arm_group_.c_str());
    
    return CallbackReturn::SUCCESS;
}

MotionPlannerNode::CallbackReturn MotionPlannerNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
    (void)state;
    RCLCPP_INFO(get_logger(), "deactivating motion planner node");

    //取消订阅，避免inactive后续收到请求
    plan_request_sub_.reset();
    execute_result_sub_.reset();
    joint_states_sub_.reset();
    grasp_pose_sub_.reset();

    //唤醒可能正在等待执行结果的线程
    {
        std::lock_guard<std::mutex> lock(mutex_);
        last_execute_result_.reset();
    }
    execute_result_cv_.notify_all();

    if(feedback_pub_)
    {
        feedback_pub_->on_deactivate();
    }

    if(trajectory_pub_){
        trajectory_pub_->on_deactivate();
    }

    planning_busy_ = false;

    RCLCPP_INFO(get_logger(),"motion planner deactivated");
    return CallbackReturn::SUCCESS;
}

MotionPlannerNode::CallbackReturn MotionPlannerNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
    (void)state;

    RCLCPP_INFO(get_logger(), "cleaning up motion planner node");

    plan_request_sub_.reset();
    execute_result_sub_.reset();
    joint_states_sub_.reset();
    grasp_pose_sub_.reset();

    feedback_pub_.reset();
    trajectory_pub_.reset();

    {
        std::lock_guard<std::mutex> lock(mutex_);
        last_execute_result_.reset();
        latest_grasp_pose_.reset();
        latest_joint_states_.reset();
    }

    execute_result_cv_.notify_all();
    planning_busy_ = false;

    RCLCPP_INFO(get_logger(), "motion planner cleaned up");
    return CallbackReturn::SUCCESS;
}

MotionPlannerNode::CallbackReturn MotionPlannerNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
    (void)state;

    RCLCPP_INFO(get_logger(), "cleaning up motion planner node");

    plan_request_sub_.reset();
    execute_result_sub_.reset();
    joint_states_sub_.reset();
    grasp_pose_sub_.reset();

    feedback_pub_.reset();
    trajectory_pub_.reset();

    {
        std::lock_guard<std::mutex> lock(mutex_);
        last_execute_result_.reset();
        latest_grasp_pose_.reset();
        latest_joint_states_.reset();
    }

    execute_result_cv_.notify_all();
    planning_busy_ = false;

    RCLCPP_INFO(get_logger(), "motion planner cleaned up");
    return CallbackReturn::SUCCESS;
    
}

//核心：收到规划请求后怎么处理
void MotionPlannerNode::onPlanRequest(const PlanRequest::SharedPtr msg)
{
    //如果上层没有给request_id, 就用时间戳生成一个
    const std::string request_id = 
    msg->request_id.empty()
    ?("plan_" + std::to_string(now().nanoseconds()))
    :msg->request_id;

    //防止多个规划任务同时执行
    bool expected = false;
    if(!planning_busy_.compare_exchange_strong(expected, true)){
        RCLCPP_WARN(get_logger(), "planner is busy, reject new request");
        publishFeedback(request_id, ExecuteFeedback::STATUS_FAILED, 0.0, "planner is busy");
        return;
    }

    //函数退出时自动释放busy状态
    auto reset_busy = [this]()
    {
        planning_busy_ = false;
    };

    try{
         
        publishFeedback(
            request_id,
            ExecuteFeedback::STATUS_PLANNING,
            0.0,
            "planning");

        //没有指定group_name时，使用默认规划组
        const std::string group_name = 
            msg->group_name.empty()? default_arm_group_ : msg->group_name;

        RCLCPP_INFO(
            get_logger(),
            "received plan request, id=%s, group=%s, type=%u",
            request_id.c_str(),
            group_name.c_str(),
            msg->plan_type);

        //创建Moveit2接口
        //这一版保持简单：每次请求都创建一次
        //后期可以优化为on_configure 时创建并复用
        auto moveit_node = 
            std::make_shared<rclcpp::Node>("agx_motion_planner_moveit_client");

        moveit::planning_interface::MoveGroupInterface move_group(
            moveit_node,
            group_name);

        //设置规划期id
        if(!msg->planner_id.empty()){
            move_group.setPlannerId(msg->planner_id);
        }

        //限制速度缩放范围，避免上层误发2.0、-1.0这种非法值
        const double velocity_scaling = 
            std::clamp(msg->max_velocity_scaling, 0.01, 1.0);

        const double acceleration_scaling =
            std::clamp(msg->max_acceleration_scaling, 0.01, 1.0);

        move_group.setMaxVelocityScalingFactor(velocity_scaling);
        move_group.setMaxAccelerationScalingFactor(acceleration_scaling);

        //当前机械臂状态作为规划起点
        move_group.setStartStateToCurrentState();

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool plan_ok = false;

        //根据不同plan_type进入不同规划函数
        switch(msg->plan_type){
            case PlanRequest::PLAN_TYPE_JOINT:
                plan_ok = planJointTarget(move_group, *msg, plan);
                break;

            case PlanRequest::PLAN_TYPE_NAMED_TARGET:
                plan_ok = planNamedTarget(move_group, *msg, plan);
                break;

            case PlanRequest::PLAN_TYPE_POSE:
                plan_ok = planPoseTarget(move_group, *msg, plan, request_id);
                break;

            case PlanRequest::PLAN_TYPE_CARTESIAN:
                plan_ok = planCartesianPath(move_group, *msg, plan);
                break;

            default:
                publishFeedback(
                    request_id,
                    ExecuteFeedback::STATUS_FAILED,
                    0.0,
                    "unsupported plan_type");
                reset_busy();
                return;
        }
        
        if(!plan_ok){
            publishFeedback(
                request_id,
                ExecuteFeedback::STATUS_FAILED,
                0.0,
                "planning failed");
            reset_busy();
            return;
        }

        publishFeedback(
            request_id,
            ExecuteFeedback::STATUS_PLANNED,
            0.5,
            "planning succeeded");

        //很重要：
        //如果只是规划不执行，不应该把轨迹发给执行器
        if(!msg->execute){
            publishFeedback(
                request_id,
                ExecuteFeedback::STATUS_SUCCEEDED,
                1.0,
                "plan only");
            reset_busy();
            return;
        }

        //执行前先清空旧结果，避免误匹配上一次执行结果
        {
            std::lock_guard<std::mutex> lock(mutex_);
            last_execute_result_.reset();
        }

        //封装轨迹消息
        MotionTrajectory traj_msg;
        traj_msg.header.stamp = now();
        traj_msg.request_id = request_id;
        traj_msg.trajectory = plan.trajectory_;

        //发布轨迹给arm_controller_node
        trajectory_pub_->publish(traj_msg);

        publishFeedback(
            request_id,
            ExecuteFeedback::STATUS_EXECUTING,
            0.6,
            "trajectory published, waiting for execution result");

        //等待arm_controller_node 返回执行结果
        const bool success = waitForExecuteResult(request_id);

        publishFeedback(
            request_id,
            success ? ExecuteFeedback::STATUS_SUCCEEDED : ExecuteFeedback::STATUS_FAILED,
            success ? 1.0 : 0.0,
            success ? "execution succeeded" : "excution faild");

        reset_busy();
    }
    catch(const std::exception & e){
        RCLCPP_ERROR(get_logger(), "plan request exception: %s", e.what());
        publishFeedback(request_id, ExecuteFeedback::STATUS_FAILED, 0.0, e.what());

        //这里没有request_id的话也至少打印错误
        planning_busy_ = false;
    }
}

//关节规划
bool MotionPlannerNode::planJointTarget(
    moveit::planning_interface::MoveGroupInterface & move_group,
    const PlanRequest & request,
    moveit::planning_interface::MoveGroupInterface::Plan & plan)
{
    if(request.joint_goal.empty()){
        RCLCPP_ERROR(get_logger(), "joint_goal is empty");
        return false;
    }
    //给moveit设置目标关节
    move_group.setJointValueTarget(request.joint_goal);
    //调用Moveit规划
    const auto result = move_group.plan(plan);
    return result == moveit::core::MoveItErrorCode::SUCCESS;
}

//预设姿态规划
bool MotionPlannerNode::planNamedTarget(
    moveit::planning_interface::MoveGroupInterface & move_group,
    const PlanRequest & request,
    moveit::planning_interface::MoveGroupInterface::Plan & plan)
{
    if(request.named_target.empty()){
        RCLCPP_ERROR(get_logger(), "named_target is empty");
        return false;
    }

    //named_target必须再Moveit SRFD 里提前配置
    move_group.setNamedTarget(request.named_target);
    const auto result = move_group.plan(plan);
    return result == moveit::core::MoveItErrorCode::SUCCESS;
}

//末端规划
bool MotionPlannerNode::planPoseTarget(
    moveit::planning_interface::MoveGroupInterface & move_group,
    const PlanRequest & request,
    moveit::planning_interface::MoveGroupInterface::Plan & plan,
    const std::string & request_id)
{
    geometry_msgs::msg::PoseStamped goal = request.pose_goal;

    //如果请求里没有pose_goal, 就尝试使用最近的一次视觉抓取位姿
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if(goal.header.frame_id.empty() && latest_grasp_pose_.has_value()){
            goal = latest_grasp_pose_.value();
            RCLCPP_INFO(get_logger(), "use latest grasp pose as pose target");
        }
    }

    //必须检查frame_id
    //没有frame_id moveit不知道这个目标时camera_link, arm_base,还是map下的
    if(goal.header.frame_id.empty()){
        publishFeedback(
            request_id,
            ExecuteFeedback::STATUS_FAILED,
            0.0,
            "pose goal frame_id is empty");
        
        return false;
    }
    //设置末端目标位姿
    move_group.setPoseTarget(goal);
    const auto result = move_group.plan(plan);
    //清除目标，避免影响下一次规划
    move_group.clearPoseTargets();
    return result==moveit::core::MoveItErrorCode::SUCCESS;
  
}

//笛卡尔规划
bool MotionPlannerNode::planCartesianPath(
    moveit::planning_interface::MoveGroupInterface & move_group,
    const PlanRequest & request,
    moveit::planning_interface::MoveGroupInterface::Plan & plan)
{
    //获取当前末端位姿
    const auto current_pose = move_group.getCurrentPose();

    geometry_msgs::msg::Pose target_pose = current_pose.pose;
    const auto & dir = request.cartesian_direction.vector;
    /*
        比如你要夹爪向下靠近物体，可以设置：
        direction = [0, 0, -1]
        distance = 0.05
        意思是末端沿 z 轴下降 5cm。
        注意：最好时单位方向向量，长度为1
    */ 
    //计算方向向量长度
    const double norm = std::sqrt(
        dir.x * dir.x +
        dir.y * dir.y +
        dir.z * dir.z);

    if(norm < 1e-6){
        RCLCPP_ERROR(get_logger(), "cartesian direction is zero");
        return false;
    }

    //距离默认5cm
    const double dist = 
        request.cartesian_max_dist > 0.0 ? request.cartesian_max_dist : 0.05;

    //归一化方向，避免[0,0,-10]导致移动50cm
    target_pose.position.x += dir.x / norm * dist;
    target_pose.position.y += dir.y / norm * dist;
    target_pose.position.z += dir.z / norm * dist;

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;

    //插值步长，默认1cm
    const double step = 
        request.cartesian_step_size>0.0 ? request.cartesian_step_size : 0.01;

    //原来代码中的catesian_min_dist 命名不像jump_threshold 后面要检查msg定义,下面暂时按照原文件的cartesian_min_dist
    //const double jump_threshold = 0.0;

    const double fraction = move_group.computeCartesianPath(
        waypoints,
        step,
        request.cartesian_min_dist,
        trajectory);

    RCLCPP_INFO(get_logger(),"cartesian path fraction : %.3f", fraction);

    if(fraction < 0.95){
        RCLCPP_ERROR(get_logger(), "catesian path planning incomplete");
        return false;
    }

    plan.trajectory_ = trajectory;
    return true;

}

//反馈发布函数
void MotionPlannerNode::publishFeedback(
    const std::string & request_id,
    uint8_t status,
    float progress,
    const std::string & message)
{
    if(!feedback_pub_){
        RCLCPP_WARN(get_logger(),"feedback publisher is null");
        return;
    }

    ExecuteFeedback msg;
    msg.header.stamp = now();
    msg.request_id = request_id;
    msg.status = status;
    msg.progress = progress;
    msg.message = message;

    feedback_pub_->publish(msg);
}

//等待执行结果
bool MotionPlannerNode::waitForExecuteResult(const std::string & request_id)
{
    const auto deadline =
        std::chrono::steady_clock::now() +
        std::chrono::duration<double>(execute_timeout_sec_);

    std::unique_lock<std::mutex> lock(mutex_);

    while(rclcpp::ok()){
        if(last_execute_result_.has_value()){
            const auto & result = last_execute_result_.value();

            //必须request_id一直，才认为是这次任务的执行结果
            if(result.request_id == request_id){
                return result.success;
            }
        }

        const auto wait_status = execute_result_cv_.wait_until(lock, deadline);

        if(wait_status == std::cv_status::timeout){
            RCLCPP_ERROR(
                get_logger(),
                "execute timeout, request_id=%s",
                request_id.c_str());
                return false;
        }
    }
    return false;
}

}

RCLCPP_COMPONENTS_REGISTER_NODE(manipulation::MotionPlannerNode)
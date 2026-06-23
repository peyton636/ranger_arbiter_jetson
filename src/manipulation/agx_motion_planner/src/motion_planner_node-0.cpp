/** @file motion_planner_node.cpp
 *  @brief MoveIt2 规划与执行监控节点：
 *         - 订阅规划请求 /motion/plan_request
 *         - 调用 MoveIt2 生成轨迹
 *         - 发布轨迹到 /motion/trajectory 交给 arm_controller 执行
 *         - 订阅 /motion/execute_result 并回传执行状态 /motion/execute_feedback
 */

 //支持规划的类型：对应agx_arm_msgs::msg::PlanRequest::plan_type
// JOINT      关节空间规划：给每个关节一个目标角度
// POSE       末端位姿规划：给末端一个目标空间位姿 抓夹移动到某个x/y/z位置
// NAMED      预设姿态规划
// CARTESIAN  笛卡尔直线轨迹规划

#include "motion_planner_node.hpp"

// moveit + 任务调度器
namespace manipulation
{

    MotionPlannerNode::MotionPlannerNode(const rclcpp::NodeOptions &options)
        : LifecycleNode("agx_motion_planner_node", options)//unconfigured → inactive → active → finalized
    {
        RCLCPP_INFO(get_logger(), "agx_motion_planner_node created");
        // 声明并读取节点参数
        declareParameters();
    }


    MotionPlannerNode::CallbackReturn MotionPlannerNode::on_configure(const rclcpp_lifecycle::State &state)
    {
        (void)state;
        RCLCPP_INFO(get_logger(), "agx_motion_planner_node configuring");

        // 创建执行反馈发布器和轨迹发布器
        feedback_pub_ = create_publisher<ExecuteFeedback>(
            get_parameter("execute_feedback_topic").as_string(),10);
        
        trajectory_pub_ = create_publisher<MotionTrajectory>(
            get_parameter("trajectory_topic").as_string(),10);

        if(!feedback_pub_ || !trajectory_pub_)
        {
            return CallbackReturn::FAILURE;
        }
        return CallbackReturn::SUCCESS;
    }

    //激活后创建订阅器
    MotionPlannerNode::CallbackReturn MotionPlannerNode::on_activate(const rclcpp_lifecycle::State &state)
    {
        (void)state;
        RCLCPP_INFO(get_logger(), "agx_motion_planner_node activating");

        // 订阅规划请求
        // 订阅执行结果：接收 arm_controller_node 的轨迹执行完成通知
        execute_result_sub_ = create_subscription<ExecuteResult>(
            get_parameter("execute_result_topic").as_string(), 10,
            [this](const ExecuteResult::SharedPtr msg)
            {
                std::lock_guard<std::mutex> lock(mutex_);
                last_execute_result_ = *msg;
                execute_result_cv_.notify_all();
            });

        // 订阅关节状态：接受arm_controller_node发布的关节状态，用于规划起点或调试使用
        joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            get_parameter("joint_states_topic").as_string(), 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg)
            {
                std::lock_guard<std::mutex> lock(mutex_);
                latest_joint_states_ = *msg;
            });

        // 订阅抓取位姿：如果规划请求中没有提供目标位姿，可以使用最新的抓取位姿作为规划目标
        // 订阅抓取位姿：若规划请求未显式给 pose_goal，可回退使用最近一次抓取位姿
        grasp_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            get_parameter("grasp_pose_topic").as_string(), 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
            {
                std::lock_guard<std::mutex> lock(mutex_);
                latest_grasp_pose_ = *msg;
            });

        // 订阅规划请求：收到请求后开线程执行，避免阻塞 ROS 回调线程
        plan_request_sub_ = create_subscription<PlanRequest>(
            get_parameter("plan_request_topic").as_string(), 10,
            [this](const PlanRequest::SharedPtr msg)
            {
                std::thread([this, msg]()
                            { onPlanRequest(msg); })
                    .detach();
            });

        RCLCPP_INFO(get_logger(), "motion_planner_node ready (default group: %s)", default_arm_group_.c_str());
        return CallbackReturn::SUCCESS;
    }

    //停用后断开订阅，释放发布器
    MotionPlannerNode::CallbackReturn MotionPlannerNode::on_deactivate(const rclcpp_lifecycle::State &state)
    {
        (void)state;
        RCLCPP_INFO(get_logger(), "agx_motion_planner_node deactivating");

        // 先让可能正在等待执行结果的线程尽快退出
        {
            std::lock_guard<std::mutex> lock(mutex_);
            last_execute_result_.reset();
        }
        execute_result_cv_.notify_all();

        // 断开订阅，避免 deactivate 后继续接收消息
        plan_request_sub_.reset();
        execute_result_sub_.reset();
        joint_states_sub_.reset();
        grasp_pose_sub_.reset();

        // 释放发布器
        feedback_pub_.reset();
        trajectory_pub_.reset();

        RCLCPP_INFO(get_logger(), "agx_motion_planner_node deactivated");
        return CallbackReturn::SUCCESS;
    }

    //清理后释放资源
    MotionPlannerNode::CallbackReturn MotionPlannerNode::on_cleanup(const rclcpp_lifecycle::State &state)
    {
        (void)state;
        RCLCPP_INFO(get_logger(), "agx_motion_planner_node cleaning up");

        {
            std::lock_guard<std::mutex> lock(mutex_);
            last_execute_result_.reset();
            latest_grasp_pose_.reset();
            latest_joint_states_.reset();
        }
        execute_result_cv_.notify_all();

        plan_request_sub_.reset();
        execute_result_sub_.reset();
        joint_states_sub_.reset();
        grasp_pose_sub_.reset();
        feedback_pub_.reset();
        trajectory_pub_.reset();

        RCLCPP_INFO(get_logger(), "agx_motion_planner_node cleaned up");
        return CallbackReturn::SUCCESS;
    }

    //关闭后释放资源
    MotionPlannerNode::CallbackReturn MotionPlannerNode::on_shutdown(const rclcpp_lifecycle::State &state)
    {
        (void)state;
        RCLCPP_INFO(get_logger(), "agx_motion_planner_node shutting down");

        {
            std::lock_guard<std::mutex> lock(mutex_);
            last_execute_result_.reset();
            latest_grasp_pose_.reset();
            latest_joint_states_.reset();
        }
        execute_result_cv_.notify_all();

        plan_request_sub_.reset();
        execute_result_sub_.reset();
        joint_states_sub_.reset();
        grasp_pose_sub_.reset();
        feedback_pub_.reset();
        trajectory_pub_.reset();

        RCLCPP_INFO(get_logger(), "agx_motion_planner_node shut down");
        return CallbackReturn::SUCCESS;
    }

    //声明参数
    void MotionPlannerNode::declareParameters()
    {
        declare_parameter<std::string>("default_arm_group", "arm");//MoveIt 规划组名称
        declare_parameter<std::string>("plan_request_topic", "/motion/plan_request");//规划请求话题
        declare_parameter<std::string>("grasp_pose_topic", "/grasp/selected_pose");//接收视觉抓取位姿
        declare_parameter<std::string>("joint_states_topic", "/joint/states");//接收关节状态
        declare_parameter<std::string>("trajectory_topic", "/motion/trajectory");//发布轨迹话题
        declare_parameter<std::string>("execute_feedback_topic", "/motion/execute_feedback");//发布执行反馈话题
        declare_parameter<std::string>("execute_result_topic", "/motion/execute_result");//接收执行结果话题
        declare_parameter<double>("execute_timeout_sec", 120.0);//等待执行结果超时时间

        default_arm_group_ = get_parameter("default_arm_group").as_string();
        execute_timeout_sec_ = get_parameter("execute_timeout_sec").as_double();
    }

    //核心规划函数
    void MotionPlannerNode::onPlanRequest(const agx_motion_msgs::msg::PlanRequest::SharedPtr msg)
    {
        // 生成request_id，作用是让后面的规划，执行，反馈都能对应同一个任务，如果没有，则使用时间戳
        const std::string request_id = msg->request_id.empty() ? ("plan_" + std::to_string(now().nanoseconds())) : msg->request_id;

        // 进入规划阶段,向/motion/execute_feedback发布规划中状态
        publishFeedback(request_id, ExecuteFeedback::STATUS_PLANNING, 0.0, "planning");

        // 未指定 group_name 时使用默认规划组arm
        const std::string group_name = msg->group_name.empty() ? default_arm_group_ : msg->group_name;

        try
        {
            // 创建Moveit2接口，调用Moveit2规划轨迹，当前代码每次收到请求都会创建一个新的Moveit_node和move_group,后期不推荐
            // MoveGroupInterface 需要 rclcpp::Node::SharedPtr，不能直接传 LifecycleNode::SharedPtr
            auto moveit_node = std::make_shared<rclcpp::Node>("agx_motion_planner_moveit_client");
            auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                moveit_node, group_name);

            // 可选：设置规划器 ID，具体名字取决于你的Moveit配置
            if (!msg->planner_id.empty())
            {
                move_group->setPlannerId(msg->planner_id);
            }

            // 设置速度/加速度缩放系数；若请求值非法则回退到 1.0，建议后期加限制，不要让他超过1.0
            move_group->setMaxVelocityScalingFactor(
                msg->max_velocity_scaling > 0.0 ? msg->max_velocity_scaling : 1.0);
            move_group->setMaxAccelerationScalingFactor(
                msg->max_acceleration_scaling > 0.0 ? msg->max_acceleration_scaling : 1.0);

            // 从当前姿态开始规划，而不是默认姿态
            move_group->setStartStateToCurrentState();

            moveit::planning_interface::MoveGroupInterface::Plan plan;
            bool plan_ok = false;

            switch (msg->plan_type)
            {
            case PlanRequest::PLAN_TYPE_JOINT:
                // 关节空间规划：直接设置 joint_goal
                if (!msg->joint_goal.empty())
                {
                    move_group->setJointValueTarget(msg->joint_goal);
                    plan_ok = (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
                }
                break;

            case PlanRequest::PLAN_TYPE_NAMED_TARGET:
                // 预设姿态规划：使用命名 target（需事先在 MoveIt 配置中定义好）
                if (!msg->named_target.empty())
                {
                    move_group->setNamedTarget(msg->named_target);
                    plan_ok = (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
                }
                break;

            case PlanRequest::PLAN_TYPE_POSE:
            {
                // 位姿规划：优先使用请求中的 pose_goal；
                // 若 frame_id 为空且已有缓存抓取位姿，则使用 latest_grasp_pose_ 作为规划目标
                geometry_msgs::msg::PoseStamped goal = msg->pose_goal;
                if(goal.header.frame_id.empty()){
                    publishFeedback(request_id, ExecuteFeedback::STATUS_FAILED, 0.0,"pose_goal frame_id is empty");
                    return;
                }
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (latest_grasp_pose_.has_value() && msg->pose_goal.header.frame_id.empty())
                    {
                        goal = latest_grasp_pose_.value();
                    }
                }
                move_group->setPoseTarget(goal);
                plan_ok = (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
                break;
            }

            case PlanRequest::PLAN_TYPE_CARTESIAN:
            {
                // 笛卡尔空间规划：沿指定方向移动末端执行器
                // 根据当前末端位姿与给定方向/距离，生成单段目标 waypoint 供 computeCartesianPath 计算笛卡尔轨迹
                std::vector<geometry_msgs::msg::Pose> waypoints;
                auto current = move_group->getCurrentPose();
                geometry_msgs::msg::Pose target = current.pose;

                /*
                    比如你要夹爪向下靠近物体，可以设置：
                    direction = [0, 0, -1]
                    distance = 0.05
                    意思是末端沿 z 轴下降 5cm。
                    注意：最好时单位方向向量，长度为1
                */ 

                // 修复：request -> msg
                const auto &dir = msg->cartesian_direction.vector;
                const double dist = msg->cartesian_max_dist > 0.0 ? msg->cartesian_max_dist : 0.05;
                target.position.x += dir.x * dist;
                target.position.y += dir.y * dist;
                target.position.z += dir.z * dist;
                waypoints.push_back(target);

                moveit_msgs::msg::RobotTrajectory trajectory;
                const double step = msg->cartesian_step_size > 0.0 ? msg->cartesian_step_size : 0.01;

                // 计算笛卡尔路径，fraction 越接近 1.0 表示路径覆盖越完整
                const double fraction = move_group->computeCartesianPath(
                    waypoints, step, msg->cartesian_min_dist, trajectory);
                    /*fraction =1.0 完整生成路径
                    fraction = 0.5    只生成了一半
                    fraction = 0.0    基本失败*/ 
                plan_ok = fraction >= 0.95;
                if (plan_ok)
                {
                    plan.trajectory_ = trajectory;
                }
                else
                {
                    RCLCPP_WARN(
                        get_logger(), "cartesian path fraction=%.2f (<0.95)", fraction);
                }
                break;
            }

            default:
                // 未支持的规划类型，直接失败返回
                publishFeedback(
                    request_id, ExecuteFeedback::STATUS_FAILED, 0.0,
                    "unsupported plan_type");
                return;
            }

            // 规划失败
            if (!plan_ok)
            {
                publishFeedback(request_id, ExecuteFeedback::STATUS_FAILED, 0.0, "planning failed");
                return;
            }

            // 将 MoveIt2 规划结果封装为 MotionTrajectory 消息并发布
            MotionTrajectory traj_msg;
            traj_msg.header.stamp = now();
            traj_msg.request_id = request_id;
            traj_msg.trajectory = plan.trajectory_;
            trajectory_pub_->publish(traj_msg); // 发布轨迹，让arm_controller_node执行/motion/trajectory
           
            // 若仅规划不执行，则直接返回成功
            if (!msg->execute)
            {
                publishFeedback(request_id, ExecuteFeedback::STATUS_SUCCEEDED, 1.0, "plan only");
                return;
            }

            // 清空上一次执行结果，避免误匹配
            {
                std::lock_guard<std::mutex> lock(mutex_);
                last_execute_result_.reset();
            }

            //发布轨迹，让arm_controller_node执行/motion/trajectory
            trajectory_pub_->publish(traj_msg);
            publishFeedback(request_id, ExecuteFeedback::STATUS_PLANNED, 0.5, "trajectory published");

            // 等待执行结果
            publishFeedback(request_id, ExecuteFeedback::STATUS_EXECUTING, 0.6, "waiting arm_controller");
            const bool success = waitForExecuteResult(request_id); // 等待arm_controller_node执行完成，他会监听/motion/execute_result

            // 根据执行结果发布最终反馈
            publishFeedback(
                request_id,
                success ? ExecuteFeedback::STATUS_SUCCEEDED : ExecuteFeedback::STATUS_FAILED,
                success ? 1.0 : 0.0,
                success ? "execution succeeded" : "execution failed");
        }
        catch (const std::exception &e)
        {
            // MoveIt2 初始化或规划过程中抛异常时，统一回传失败状态和异常信息
            RCLCPP_ERROR(get_logger(), "plan request failed: %s", e.what());
            publishFeedback(request_id, ExecuteFeedback::STATUS_FAILED, 0.0, e.what());
        }
    }

    //统一发布反馈
    void MotionPlannerNode::publishFeedback(const std::string &request_id, uint8_t status, float progress, const std::string &message)
    {
        // 统一封装执行反馈消息
        ExecuteFeedback msg;
        msg.header.stamp = now();
        msg.request_id = request_id;
        msg.status = status;
        msg.progress = progress;
        msg.message = message;
        feedback_pub_->publish(msg);
    }

    bool MotionPlannerNode::waitForExecuteResult(const std::string &request_id)
    {
        // 在超时时间内等待 execute_result_sub_ 收到对应 request_id 的执行结果
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::duration<double>(execute_timeout_sec_);

        std::unique_lock<std::mutex> lock(mutex_);
        //一直等，直到收到相同的request_id的执行结果，如果超时则认为失败
        while (rclcpp::ok())
        {
            if (last_execute_result_.has_value() &&
                last_execute_result_->request_id == request_id)
            {
                return last_execute_result_->success;
            }
            if (execute_result_cv_.wait_until(lock, deadline) == std::cv_status::timeout)
            {
                RCLCPP_ERROR(get_logger(), "execute timeout for request_id=%s", request_id.c_str());
                return false;
            }
        }
        return false;
    }

    // moveit_msgs::msg::RobotTrajectory MotionPlannerNode::planToPose(const geometry_msgs::msg::PoseStamped &target_pose)
    // {
    //     return moveit_msgs::msg::RobotTrajectory();
    // }

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(manipulation::MotionPlannerNode)

#include "arm_controller_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace manipulation
{

    ArmControllerNode::ArmControllerNode(const rclcpp::NodeOptions &options)
        : rclcpp_lifecycle::LifecycleNode("arm_controller_node", options)
    {
        RCLCPP_INFO(get_logger(), "Constructing ArmControllerNode...");
        // 声明并初始化节点参数，供后续生命周期回调使用
        declareParameters();
    }

    ArmControllerNode::CallbackReturn ArmControllerNode::on_configure(const rclcpp_lifecycle::State &state)
    {
        (void)state; // 避免未使用参数的编译警告
        RCLCPP_INFO(get_logger(), "Configuring ArmControllerNode...");
        // 读取参数，初始化控制与超时配置
        control_rate_hz_ = get_parameter("control_rate_hz").as_double();
        goal_tolerance_ = get_parameter("goal_tolerance").as_double();
        reach_timeout_sec_ = get_parameter("reach_timeout_sec").as_double();
        action_server_timeout_sec_ = get_parameter("action_server_timeout_sec").as_double();
        action_result_timeout_sec_ = get_parameter("action_result_timeout_sec").as_double();
        use_ros2_control_action_ = get_parameter("use_ros2_control_action").as_bool();

        // 读取参数，初始化控制与超时配置
        if (!joint_states_pub_)
            joint_states_pub_ = create_publisher<sensor_msgs::msg::JointState>(
                get_parameter("joint_states_topic").as_string(), 10);
        if (!control_pub_)
            control_pub_ = create_publisher<sensor_msgs::msg::JointState>(
                get_parameter("control_joint_states_topic").as_string(), 10);
        if (!result_pub_)
            result_pub_ = create_publisher<ExecuteResult>(
                get_parameter("execute_result_topic").as_string(), 10);

        return CallbackReturn::SUCCESS;
    }

    ArmControllerNode::CallbackReturn ArmControllerNode::on_activate(const rclcpp_lifecycle::State &state)
    {
        (void)state; // 避免未使用参数的编译警告
        RCLCPP_INFO(get_logger(), "Activating ArmControllerNode...");

        // 先激活生命周期发布器
        if (joint_states_pub_)
            joint_states_pub_->on_activate();
        if (control_pub_)
            control_pub_->on_activate();
        if (result_pub_)
            result_pub_->on_activate();

        // 订阅真实反馈，用于判断机械臂是否到位
        feedback_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            get_parameter("feedback_joint_states_topic").as_string(), 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg)
            {
                onFeedbackJointStates(msg);
            });

        // 订阅轨迹指令，收到后启动轨迹执行流程
        traj_sub_ = create_subscription<MotionTrajectory>(
            get_parameter("trajectory_topic").as_string(), 10,
            [this](const MotionTrajectory::SharedPtr msg)
            {
                onTrajectory(msg);
            });

        if (!feedback_sub_ || !traj_sub_)
        {
            RCLCPP_ERROR(get_logger(), "Failed to activate AgxArmControllerNode: subscription creation failed.");
            return CallbackReturn::ERROR;
        }

        // 根据参数决定是否使用 ros2_control action 方式执行
        if (use_ros2_control_action_)
        {
            action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
                this, get_parameter("follow_joint_trajectory_action").as_string());

            if (!action_client_)
            {
                RCLCPP_ERROR(get_logger(), "Failed to create action client.");
                return CallbackReturn::ERROR;
            }
        }

        RCLCPP_INFO(
            get_logger(), "arm_controller_node ready (use_ros2_control_action=%s)",
            use_ros2_control_action_ ? "true" : "false");
        return CallbackReturn::SUCCESS;
    }

    ArmControllerNode::CallbackReturn ArmControllerNode::on_deactivate(const rclcpp_lifecycle::State &state)
    {
        (void)state; // 避免未使用参数的编译警告
        RCLCPP_INFO(get_logger(), "Deactivating ArmControllerNode...");

        // 停止当前执行状态，避免后续误判
        {
            std::lock_guard<std::mutex> lock(exec_mutex_);
            executing_ = false;
        }

        // 生命周期发布器在 deactivate 时应关闭输出
        if (joint_states_pub_)
            joint_states_pub_->on_deactivate();
        if (control_pub_)
            control_pub_->on_deactivate();
        if (result_pub_)
            result_pub_->on_deactivate();

        // 解除订阅和 action 客户端，避免 deactivate 后继续接收/发送数据
        feedback_sub_.reset();
        traj_sub_.reset();
        action_client_.reset();

        RCLCPP_INFO(get_logger(), "ArmControllerNode deactivated.");
        return CallbackReturn::SUCCESS;
    }

    ArmControllerNode::CallbackReturn ArmControllerNode::on_cleanup(const rclcpp_lifecycle::State &state)
    {
        (void)state; // 避免未使用参数的编译警告
        RCLCPP_INFO(get_logger(), "Cleaning up ArmControllerNode...");

        // 彻底清理运行期资源
        {
            std::lock_guard<std::mutex> lock(exec_mutex_);
            executing_ = false;
        }

        {
            std::lock_guard<std::mutex> lock(feedback_mutex_);
            latest_feedback_.reset();
        }

        feedback_sub_.reset();
        traj_sub_.reset();
        action_client_.reset();
        joint_states_pub_.reset();
        control_pub_.reset();
        result_pub_.reset();

        RCLCPP_INFO(get_logger(), "ArmControllerNode cleaned up.");
        return CallbackReturn::SUCCESS;
    }

    ArmControllerNode::CallbackReturn ArmControllerNode::on_shutdown(const rclcpp_lifecycle::State &state)
    {
        (void)state; // 避免未使用参数的编译警告
        RCLCPP_INFO(get_logger(), "Shutting down ArmControllerNode...");

        // 与 cleanup 保持一致，确保退出前资源全部释放
        {
            std::lock_guard<std::mutex> lock(exec_mutex_);
            executing_ = false;
        }

        {
            std::lock_guard<std::mutex> lock(feedback_mutex_);
            latest_feedback_.reset();
        }

        feedback_sub_.reset();
        traj_sub_.reset();
        action_client_.reset();
        joint_states_pub_.reset();
        control_pub_.reset();
        result_pub_.reset();

        RCLCPP_INFO(get_logger(), "ArmControllerNode shut down.");
        return CallbackReturn::SUCCESS; // shutdown 也返回 SUCCESS，表示正常完成关闭流程
    }

    void ArmControllerNode::declareParameters()
    {
        // 轨迹输入与输出话题
        declare_parameter<std::string>("trajectory_topic", "/motion/trajectory"); // 接收轨迹
        declare_parameter<std::string>("joint_states_topic", "/joint/states");
        declare_parameter<std::string>("feedback_joint_states_topic", "/feedback/joint_states"); // 接收机械臂真实反馈
        declare_parameter<std::string>("control_joint_states_topic", "/control/joint_states");   // 下发控制指令
        declare_parameter<std::string>("execute_result_topic", "/motion/execute_result");        // 发布执行结果

        // ros2_control action 名称
        declare_parameter<std::string>(
            "follow_joint_trajectory_action", "/arm_controller/follow_joint_trajectory");

        // 执行方式开关与控制参数
        declare_parameter<bool>("use_ros2_control_action", false);
        declare_parameter<double>("control_rate_hz", 50.0);            // 控制频率
        declare_parameter<double>("goal_tolerance", 0.02);             // 到位误差容忍度
        declare_parameter<double>("reach_timeout_sec", 5.0);           // 每个目标点最多等待多久
        declare_parameter<double>("action_server_timeout_sec", 5.0);   // 动作服务器超时时间
        declare_parameter<double>("action_result_timeout_sec", 120.0); // 动作结果超时时间
    }

    // 缓存最新反馈latest_feedback_
    // 转发到 /joint/states
    // 缓存最新反馈，供执行线程判断当前关节是否已经到位
    void ArmControllerNode::onFeedbackJointStates(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->name.empty() || msg->position.empty() || msg->name.size() != msg->position.size())
        {
            RCLCPP_WARN(get_logger(), "Received invalid joint state feedback: empty or size mismatch");
            return;
        }

        {
            // 线程安全地更新latest_feedback_
            std::lock_guard<std::mutex> lock(feedback_mutex_);
            latest_feedback_ = *msg; // 更新最新反馈
        }
    }

    // 防止同时执行多条轨迹；若当前已有轨迹在执行，则直接拒绝新请求
    void ArmControllerNode::onTrajectory(const MotionTrajectory::SharedPtr msg)
    {
        if (msg->request_id.empty())
        {
            RCLCPP_WARN(get_logger(), "Received trajectory with empty request_id, ignoring");
            return;
        }
        std::lock_guard<std::mutex> lock(exec_mutex_);

        // 如果当前已经在执行轨迹，就拒绝新轨迹
        if (executing_)
        {
            publishResult(msg->request_id, false, -1, "busy: previous trajectory running");
            return;
        }
        else
        {
            executing_ = true; // 如果当前空闲
        }

        // 就开一个新线程执行轨迹，executeTrajectory会等待机械臂到位，一直等待会阻塞ros2
        std::thread([this, msg]()
                    {
      if (use_ros2_control_action_) {
        executeTrajectoryViaAction(*msg);
      } else {
        executeTrajectoryViaJointStates(*msg);
      } })
            .detach();
    }

    // 从轨迹中取关节点,把标准轨迹转成内部的数据结构std::vector<JointWaypoint>
    void ArmControllerNode::executeTrajectoryViaAction(const MotionTrajectory &msg)
    {
        struct ExecReset
        {
            ArmControllerNode *self;
            ~ExecReset()
            {
                std::lock_guard<std::mutex> lock(self->exec_mutex_);
                self->executing_ = false;
            }
        } exec_reset{this};

        const std::string request_id = msg.request_id;
        try
        {
            if (!action_client_)
            {
                publishResult(request_id, false, -5, "ros2_control action client not initialized");
                return;
            }

            const auto &joint_traj = msg.trajectory.joint_trajectory;
            if (joint_traj.points.empty())
            {
                publishResult(request_id, false, -2, "empty trajectory");
                return;
            }

            const auto server_timeout = std::chrono::duration<double>(action_server_timeout_sec_);
            if (!action_client_->wait_for_action_server(server_timeout))
            {
                publishResult(request_id, false, -5, "ros2_control action server not available");
                return;
            }

            FollowJointTrajectory::Goal goal;
            goal.trajectory.header = joint_traj.header;
            goal.trajectory.header.stamp = now();
            goal.trajectory.joint_names = joint_traj.joint_names;
            goal.trajectory.points = joint_traj.points;

            auto send_goal_future = action_client_->async_send_goal(goal);
            if (send_goal_future.wait_for(server_timeout) != std::future_status::ready)
            {
                publishResult(request_id, false, -5, "ros2_control action send goal timeout");
                return;
            }

            const auto goal_handle = send_goal_future.get();
            if (!goal_handle)
            {
                publishResult(request_id, false, -5, "ros2_control action goal rejected");
                return;
            }

            const auto result_timeout = std::chrono::duration<double>(action_result_timeout_sec_);
            auto result_future = action_client_->async_get_result(goal_handle);
            if (result_future.wait_for(result_timeout) != std::future_status::ready)
            {
                publishResult(request_id, false, -5, "ros2_control action result timeout");
                return;
            }

            const auto wrapped_result = result_future.get();
            if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED &&
                wrapped_result.result->error_code == control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL)
            {
                publishResult(request_id, true, 0, "execution succeeded");
            }
            else
            {
                publishResult(
                    request_id, false, -5,
                    "ros2_control action failed: error_code=" +
                        std::to_string(wrapped_result.result->error_code));
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "trajectory execution failed: %s", e.what());
            publishResult(request_id, false, -4, e.what());
        }
    }

    // 真机：逐点下发 /control/joint_states 并等待 /feedback/joint_states 到位
    void ArmControllerNode::executeTrajectoryViaJointStates(const MotionTrajectory &msg)
    {
        struct ExecReset
        {
            ArmControllerNode *self;
            ~ExecReset()
            {
                std::lock_guard<std::mutex> lock(self->exec_mutex_);
                self->executing_ = false;
            }
        } exec_reset{this};

        const std::string request_id = msg.request_id;
        try
        {
            const auto points = extractJointPoints(msg); // 把MotionTrajectory转成内部的JointWaypoint列表
            if (points.empty())
            {
                publishResult(request_id, false, -2, "empty trajectory");
                return;
            }

            const auto period = std::chrono::duration<double>(1.0 / std::max(control_rate_hz_, 1.0));
            for (const auto &point : points)
            {
                // 实际下发给驱动的命令
                sensor_msgs::msg::JointState cmd;
                cmd.header.stamp = now();
                cmd.name = point.names;
                cmd.position = point.positions;
                if (!point.efforts.empty())
                {
                    // 如果有力控制，就下发力控制
                    cmd.effort = point.efforts;
                }
                control_pub_->publish(cmd);

                // 等待反馈到达目标位置
                if (!waitUntilReached(point.names, point.positions))
                {
                    publishResult(request_id, false, -3, "joint goal not reached in time");
                    return;
                }
                std::this_thread::sleep_for(
                    std::chrono::duration_cast<std::chrono::milliseconds>(period));
            }

            publishResult(request_id, true, 0, "execution succeeded");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "trajectory execution failed: %s", e.what());
            publishResult(request_id, false, -4, e.what());
        }
    }

    // 从 MotionTrajectory 中提取关节点，转换为内部使用的 JointWaypoint 列表
    std::vector<JointWaypoint> ArmControllerNode::extractJointPoints(const MotionTrajectory &msg)
    {
        std::vector<JointWaypoint> points;
        const auto &joint_traj = msg.trajectory.joint_trajectory;
        if (joint_traj.points.empty())
        {
            return points;
        }

        for (const auto &point : joint_traj.points)
        {
            JointWaypoint wp;
            wp.names = joint_traj.joint_names;
            wp.positions = point.positions;
            wp.efforts = point.effort;
            points.push_back(std::move(wp));
        }
        return points;
    }

    // 轮询最新反馈，判断所有目标关节是否在容差范围内
    bool ArmControllerNode::waitUntilReached(const std::vector<std::string> &names, 
                                             const std::vector<double> &target_positions)
    {
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::duration<double>(reach_timeout_sec_);

        std::map<std::string, double> target_map;
        // 先把目标关节做成map
        for (size_t i = 0; i < names.size() && i < target_positions.size(); ++i)
        {
            target_map[names[i]] = target_positions[i];
        }

        // 然后循环读取反馈
        while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
        {
            std::optional<sensor_msgs::msg::JointState> feedback;
            {
                std::lock_guard<std::mutex> lock(feedback_mutex_);
                feedback = latest_feedback_;
            }

            if (!feedback.has_value())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                continue;
            }

            std::map<std::string, double> current;
            for (size_t i = 0; i < feedback->name.size() && i < feedback->position.size(); ++i)
            {
                current[feedback->name[i]] = feedback->position[i];
            }

            bool all_reached = true;
            for (const auto &[name, target] : target_map)
            {
                const auto it = current.find(name);
                if (it == current.end() ||
                    std::abs(it->second - target) > goal_tolerance_) // 再比较容错
                {
                    all_reached = false;
                    break;
                }
            }

            if (all_reached)
            {
                return true; // 如果所有关节都到位，就返回true
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        return false; // 如果超时，就返回false
    }

    // 发布轨迹执行结果，统一对外返回成功/失败状态
    void ArmControllerNode::publishResult(const std::string &request_id, bool success, int32_t error_code, const std::string &message)
    {
        ExecuteResult result;
        result.header.stamp = now();
        result.request_id = request_id;
        result.success = success;
        result.error_code = error_code;
        result.message = message;
        result_pub_->publish(result);
    }

} // namespace manipulation

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(manipulation::ArmControllerNode)
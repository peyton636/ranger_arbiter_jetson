/** @file gripper_controller_node.cpp
 *  @brief 夹爪闭环控制节点：
 *         - 输入：GripperCmd（开/关/停止）
 *         - 反馈：GripperStatus（宽度/力/过流）
 *         - 输出：控制关节命令 + 控制状态 GripperControlStatus
 *
 *  核心策略：
 *         1) 开/关到位：按宽度阈值判断；
 *         2) 抓取成功：力阈值或电流阈值任一满足即判定抓取成功。
 */

#include "gripper_controller_node.hpp"
#include <agx_arm_msgs/msg/gripper_status.hpp>
#include <agx_motion_msgs/msg/gripper_cmd.hpp>
#include <agx_motion_msgs/msg/gripper_control_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace manipulation
{
    GripperControllerNode::GripperControllerNode(const rclcpp::NodeOptions &options)
        : LifecycleNode("agx_gripper_controller_node", options)
    {
        RCLCPP_INFO(get_logger(), "agx_gripper_controller_node created");
        // 声明并读取参数（阈值、话题、超时等）
        declareParameters();
    }

    GripperControllerNode::CallbackReturn GripperControllerNode::on_configure(const rclcpp_lifecycle::State &state)
    {
        (void)state;
        RCLCPP_INFO(get_logger(), "on_configure() is called.");

        // 状态发布：向上层汇报当前夹爪状态（MOVING/OPEN/GRASPED/FAILED等）
        status_pub_ = create_publisher<GripperControlStatus>(
            get_parameter("status_topic").as_string(), 10);

        // 控制发布：向底层驱动发送目标宽度/力（JointState）
        control_pub_ = create_publisher<JointState>(
            get_parameter("control_joint_states_topic").as_string(), 10);


        RCLCPP_INFO(get_logger(), "gripper_controller_node ready");
        return CallbackReturn::SUCCESS;
    }

    GripperControllerNode::CallbackReturn GripperControllerNode::on_activate(const rclcpp_lifecycle::State &state)
    {
        (void)state;
        RCLCPP_INFO(get_logger(), "on_activate() is called.");

        if (status_pub_)
        {
            status_pub_->on_activate();
        }
        if (control_pub_)
        {
            control_pub_->on_activate();
        }

        // 原始反馈订阅：缓存最近一次夹爪反馈（线程安全）
        // 如果之前在 deactivate 清掉了订阅，这里重建
        if (!raw_sub_)
        {
            raw_sub_ = create_subscription<GripperStatus>(
                get_parameter("raw_status_topic").as_string(), 10,
                [this](const GripperStatus::SharedPtr msg)
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    latest_raw_ = *msg;
                });
        }

        // 指令订阅：收到命令后进入执行流程（异步线程）
        if (!cmd_sub_)
        {
            cmd_sub_ = create_subscription<GripperCmd>(
                get_parameter("gripper_cmd_topic").as_string(), 10,
                [this](const GripperCmd::SharedPtr msg)
                {
                    onGripperCmd(msg);
                });
        }

        if (!raw_sub_ || !cmd_sub_)
        {
            RCLCPP_ERROR(get_logger(), "subscriptions not ready in on_activate()");
            return CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    // 生命周期占位：当前仅返回默认状态，后续可补资源释放/复位逻辑
    GripperControllerNode::CallbackReturn GripperControllerNode::on_deactivate(const rclcpp_lifecycle::State &state)
    {
        (void)state;
        RCLCPP_INFO(get_logger(), "on_deactivate() is called.");

        // 停止接收新命令，并复位执行标志
        busy_.store(false);

        // deactive 后不再接收输入
        cmd_sub_.reset();
        raw_sub_.reset();

        RCLCPP_INFO(get_logger(), "gripper_controller_node deactivated");
        return CallbackReturn::SUCCESS;
    }

    // 生命周期占位：当前仅返回默认状态，后续可补 publisher/subscription 清理
    GripperControllerNode::CallbackReturn GripperControllerNode::on_cleanup(const rclcpp_lifecycle::State &state)
    {
        (void)state;
        RCLCPP_INFO(get_logger(), "on_cleanup() is called.");

        // 先复位运行态
        busy_.store(false);

        // 清理缓存反馈
        {
            std::lock_guard<std::mutex> lock(mutex_);
            latest_raw_.reset();
        }

        // 释放通信资源
        cmd_sub_.reset();
        raw_sub_.reset();
        control_pub_.reset();
        status_pub_.reset();

        RCLCPP_INFO(get_logger(), "gripper_controller_node cleaned up");
        return CallbackReturn::SUCCESS;
    }

    // 生命周期占位：当前仅返回默认状态，后续可补 shutdown 前安全停机逻辑
    GripperControllerNode::CallbackReturn GripperControllerNode::on_shutdown(const rclcpp_lifecycle::State &state)
    {
        (void)state;
        RCLCPP_INFO(get_logger(), "on_shutdown() is called.");

        // shutdown 前与 cleanup 保持一致，确保资源完全释放
        busy_.store(false);

        {
            std::lock_guard<std::mutex> lock(mutex_);
            latest_raw_.reset();
        }

        cmd_sub_.reset();
        raw_sub_.reset();
        control_pub_.reset();
        status_pub_.reset();

        RCLCPP_INFO(get_logger(), "gripper_controller_node shutdown completed");
        return CallbackReturn::SUCCESS;
    }

    void GripperControllerNode::declareParameters()
    {
        declare_parameter<std::string>("gripper_cmd_topic", "/task/gripper_cmd");
        declare_parameter<std::string>("raw_status_topic", "/gripper/raw_status");
        declare_parameter<std::string>("status_topic", "/gripper/status");
        declare_parameter<std::string>("control_joint_states_topic", "/control/gripper_joint_states");
        declare_parameter<double>("open_width", 0.07);
        declare_parameter<double>("close_width", 0.0);
        declare_parameter<double>("default_force", 1.5);
        declare_parameter<double>("force_grasp_threshold", 0.8);
        declare_parameter<double>("current_grasp_threshold", 0.5);
        declare_parameter<double>("grasp_timeout_sec", 8.0);
        declare_parameter<double>("width_tolerance", 0.003);
        declare_parameter<double>("control_rate_hz", 20.0);

        open_width_ = get_parameter("open_width").as_double();
        close_width_ = get_parameter("close_width").as_double();
        default_force_ = get_parameter("default_force").as_double();
        force_grasp_threshold_ = get_parameter("force_grasp_threshold").as_double();
        current_grasp_threshold_ = get_parameter("current_grasp_threshold").as_double();
        grasp_timeout_sec_ = get_parameter("grasp_timeout_sec").as_double();
        width_tolerance_ = get_parameter("width_tolerance").as_double();
        control_rate_hz_ = get_parameter("control_rate_hz").as_double();
    }

    double GripperControllerNode::clampWidth(double width)
    {
        // 将目标宽度裁剪到硬件允许范围，并取绝对值避免负宽度
        return std::max(kWidthMin, std::min(kWidthMax, std::abs(width)));
    }

    double GripperControllerNode::clampForce(double force)
    {
        // 将目标夹持力裁剪到安全范围，避免过大力导致损伤
        return std::max(kForceMin, std::min(kForceMax, force));
    }

    void GripperControllerNode::executeCmd(const GripperCmd &msg)
    {
        // 请求ID兜底：若上层未给 request_id，则本地生成一个唯一ID（基于时间戳）
        std::string request_id = msg.request_id;
        if (request_id.empty())
        {
            request_id = "gripper_" + std::to_string(now().nanoseconds());
        }

        // 停止命令：直接回报 idle/stopped
        if (msg.command == GripperCmd::CMD_STOP)
        {
            publishStatus(request_id, GripperControlStatus::STATE_IDLE, false, false, "stopped");
            return;
        }

        // 统一处理目标宽度/力的默认值与限幅
        double target_width = msg.target_width;
        const double target_force = clampForce(msg.max_force > 0.0 ? msg.max_force : default_force_);

        if (msg.command == GripperCmd::CMD_OPEN)
        {
            // OPEN：默认开到 open_width_，并等待“宽度到位”
            if (target_width <= 0.0)
            {
                target_width = open_width_;
            }
            target_width = clampWidth(target_width);
            publishStatus(request_id, GripperControlStatus::STATE_MOVING, false, false, "opening");
            if (!waitWidth(target_width, target_force, true))
            {
                publishStatus(request_id, GripperControlStatus::STATE_FAILED, false, false, "open timeout");
                return;
            }
            publishStatus(request_id, GripperControlStatus::STATE_OPEN, false, false, "opened");
            return;
        }

        if (msg.command == GripperCmd::CMD_CLOSE)
        {
            // CLOSE：默认关到 close_width_，可选“等待抓取判定”
            if (target_width < 0.0)
            {
                target_width = close_width_;
            }
            target_width = clampWidth(target_width);
            publishStatus(request_id, GripperControlStatus::STATE_MOVING, false, false, "closing");

            if (msg.wait_grasp)
            {
                // 抓取判定：力阈值或电流阈值任一成立即成功，否则直到超时失败
                bool force_ok = false;
                bool current_ok = false;
                const bool grasped = waitGrasp(target_width, target_force, force_ok, current_ok);
                if (grasped)
                {
                    publishStatus(
                        request_id, GripperControlStatus::STATE_GRASPED,
                        force_ok, current_ok, "grasped");
                }
                else
                {
                    publishStatus(
                        request_id, GripperControlStatus::STATE_FAILED,
                        force_ok, current_ok, "grasp timeout");
                }
                return;
            }

            // 不等待抓取时，仅等待宽度到位
            if (!waitWidth(target_width, target_force, false))
            {
                publishStatus(request_id, GripperControlStatus::STATE_FAILED, false, false, "close timeout");
                return;
            }
            publishStatus(request_id, GripperControlStatus::STATE_IDLE, false, false, "close sent");
        }
    }

    void GripperControllerNode::sendGripperCmd(double width, double force) const
    {
        if (!control_pub_)
        {
            return;
        }
        // 发送一次底层关节控制命令（宽度 + 力）
        sensor_msgs::msg::JointState cmd;
        cmd.header.stamp = now();
        cmd.name = {kGripperJointName};
        cmd.position = {clampWidth(width)};
        cmd.effort = {clampForce(force)};
        control_pub_->publish(cmd);
    }

    bool GripperControllerNode::isWidthReached(double current_width, double target_width, bool opening) const
    {
        // 开爪：当前宽度 >= 目标-容差
        // 关爪：当前宽度 <= 目标+容差
        if (opening)
        {
            return current_width >= target_width - width_tolerance_;
        }
        return current_width <= target_width + width_tolerance_;
    }

    bool GripperControllerNode::waitWidth(double target_width, double target_force, bool opening) const
    {
        // 在超时窗口内循环发命令 + 检查反馈是否到位
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::duration<double>(grasp_timeout_sec_);
        const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(1.0 / std::max(control_rate_hz_, 1.0)));

        while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
        {
            sendGripperCmd(target_width, target_force);

            std::optional<GripperStatus> raw;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                raw = latest_raw_;
            }
            if (raw.has_value() && isWidthReached(raw->width, target_width, opening))
            {
                return true;
            }
            std::this_thread::sleep_for(period);
        }
        return false;
    }

    bool GripperControllerNode::waitGrasp(double target_width, double target_force, bool &force_ok, bool &current_ok) const
    {
        // 在超时窗口内循环发命令 + 抓取判据检测（力/电流）
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::duration<double>(grasp_timeout_sec_);
        const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(1.0 / std::max(control_rate_hz_, 1.0)));

        force_ok = false;
        current_ok = false;

        while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
        {
            sendGripperCmd(target_width, target_force);

            std::optional<GripperStatus> raw;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                raw = latest_raw_;
            }
            if (!raw.has_value())
            {
                std::this_thread::sleep_for(period);
                continue;
            }

            force_ok = raw->force >= force_grasp_threshold_;
            current_ok = raw->driver_overcurrent || raw->force >= current_grasp_threshold_;
            if (force_ok || current_ok)
            {
                return true;
            }
            std::this_thread::sleep_for(period);
        }
        return false;
    }

    void GripperControllerNode::onGripperCmd(const agx_motion_msgs::msg::GripperCmd::SharedPtr msg)
    {
        // 防并发：已有命令执行时直接拒绝并回报 busy 状态；否则启动新线程执行命令，执行过程中持续发布状态更新
        if (busy_.exchange(true))
        {
            publishStatus(
                msg->request_id, GripperControlStatus::STATE_FAILED,
                false, false, "busy: previous command running");
            return;
        }

        // 异步执行，避免阻塞订阅回调线程
        std::thread([this, msg](){
        struct BusyReset {
            std::atomic_bool &busy;
            ~BusyReset() { busy.store(false); }
        } reset{busy_};

        try {
            executeCmd(*msg);
        } catch (const std::exception &e) {
            publishStatus(msg->request_id, GripperControlStatus::STATE_FAILED, false, false, e.what());
        } catch (...) {
            publishStatus(msg->request_id, GripperControlStatus::STATE_FAILED, false, false, "unknown error");
        } })
            .detach();
    }

    void GripperControllerNode::publishStatus(const std::string &request_id, uint8_t state, bool force_threshold_met, bool current_threshold_met, const std::string &message)
    {
        // 对外发布统一状态消息，携带当前原始反馈快照（宽度/力）供上层决策参考
        GripperControlStatus status;
        status.header.stamp = now();
        status.request_id = request_id;
        status.state = state;
        status.force_threshold_met = force_threshold_met;
        status.current_threshold_met = current_threshold_met;
        status.message = message;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (latest_raw_.has_value())
            {
                status.width = latest_raw_->width;
                status.force = latest_raw_->force;
            }
        }

        status_pub_->publish(status);
    }

} // namespace manipulation

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(manipulation::GripperControllerNode)

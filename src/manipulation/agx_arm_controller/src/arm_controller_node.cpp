/** @file arm_controller_node.cpp
 *  @brief 轨迹下发执行与结果回传：订阅 /motion/trajectory，驱动 /control/joint_states。
 */


#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <agx_motion_msgs/msg/execute_result.hpp>
#include <agx_motion_msgs/msg/motion_trajectory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace arm_controller_node
{

using agx_motion_msgs::msg::ExecuteResult;
using agx_motion_msgs::msg::MotionTrajectory;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

struct JointWaypoint
{
  std::vector<std::string> names;
  std::vector<double> positions;
  std::vector<double> efforts;
};

class ArmControllerNode : public rclcpp::Node
{
public:
  ArmControllerNode()
  : Node("arm_controller_node")
  {
    declare_parameter<std::string>("trajectory_topic", "/motion/trajectory");//接收轨迹
    declare_parameter<std::string>("joint_states_topic", "/joint/states");
    declare_parameter<std::string>("feedback_joint_states_topic", "/feedback/joint_states");//接收机械臂真实反馈
    declare_parameter<std::string>("control_joint_states_topic", "/control/joint_states");//下发控制指令
    declare_parameter<std::string>("execute_result_topic", "/motion/execute_result");//发布执行结果
    declare_parameter<std::string>(
      "follow_joint_trajectory_action", "/arm_controller/follow_joint_trajectory");
    declare_parameter<bool>("use_ros2_control_action", false);
    declare_parameter<double>("control_rate_hz", 50.0);//控制频率
    declare_parameter<double>("goal_tolerance", 0.02);//到位误差容忍度
    declare_parameter<double>("reach_timeout_sec", 5.0);//每个目标点最多等待多久
    declare_parameter<double>("action_server_timeout_sec", 5.0);
    declare_parameter<double>("action_result_timeout_sec", 120.0);

    control_rate_hz_ = get_parameter("control_rate_hz").as_double();
    goal_tolerance_ = get_parameter("goal_tolerance").as_double();
    reach_timeout_sec_ = get_parameter("reach_timeout_sec").as_double();
    action_server_timeout_sec_ = get_parameter("action_server_timeout_sec").as_double();
    action_result_timeout_sec_ = get_parameter("action_result_timeout_sec").as_double();
    use_ros2_control_action_ = get_parameter("use_ros2_control_action").as_bool();

    joint_states_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      get_parameter("joint_states_topic").as_string(), 10);
    control_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      get_parameter("control_joint_states_topic").as_string(), 10);
    result_pub_ = create_publisher<ExecuteResult>(
      get_parameter("execute_result_topic").as_string(), 10);

    feedback_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      get_parameter("feedback_joint_states_topic").as_string(), 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        onFeedbackJointStates(msg);
      });//订阅真实反馈节点，调用回调函数

    traj_sub_ = create_subscription<MotionTrajectory>(
      get_parameter("trajectory_topic").as_string(), 10,
      [this](const MotionTrajectory::SharedPtr msg) {
        onTrajectory(msg);
      });//订阅轨迹节点，调用回调函数

    if (use_ros2_control_action_) {
      action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
        this, get_parameter("follow_joint_trajectory_action").as_string());
    }

    RCLCPP_INFO(
      get_logger(), "arm_controller_node ready (use_ros2_control_action=%s)",
      use_ros2_control_action_ ? "true" : "false");
  }

private:
  //缓存最新反馈latest_feedback_
  //转发到 /joint/states
  void onFeedbackJointStates(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lock(feedback_mutex_);//lastest_feedback_会被两个线程访问，需要加锁
      latest_feedback_ = *msg;
    }
    joint_states_pub_->publish(*msg);
  }

  //防止同时执行两条轨迹
  void onTrajectory(const MotionTrajectory::SharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lock(exec_mutex_);
      if (executing_) {//如果当前已经在执行轨迹，就拒绝新轨迹
        publishResult(msg->request_id, false, -1, "busy: previous trajectory running");
        return;
      }
      executing_ = true;//如果当前空闲
    }
    //就开一个新线程执行轨迹，executeTrajectory会等待机械臂到位，一直等待会阻塞ros2
    std::thread([this, msg]() {
      if (use_ros2_control_action_) {
        executeTrajectoryViaAction(*msg);
      } else {
        executeTrajectoryViaJointStates(*msg);
      }
    }).detach();
  }

  void executeTrajectoryViaAction(const MotionTrajectory & msg)
  {
    struct ExecReset
    {
      ArmControllerNode * self;
      ~ExecReset()
      {
        std::lock_guard<std::mutex> lock(self->exec_mutex_);
        self->executing_ = false;
      }
    } exec_reset{this};

    const std::string request_id = msg.request_id;
    try {
      if (!action_client_) {
        publishResult(request_id, false, -5, "ros2_control action client not initialized");
        return;
      }

      const auto & joint_traj = msg.trajectory.joint_trajectory;
      if (joint_traj.points.empty()) {
        publishResult(request_id, false, -2, "empty trajectory");
        return;
      }

      const auto server_timeout = std::chrono::duration<double>(action_server_timeout_sec_);
      if (!action_client_->wait_for_action_server(server_timeout)) {
        publishResult(request_id, false, -5, "ros2_control action server not available");
        return;
      }

      FollowJointTrajectory::Goal goal;
      goal.trajectory.header = joint_traj.header;
      goal.trajectory.header.stamp = now();
      goal.trajectory.joint_names = joint_traj.joint_names;
      goal.trajectory.points = joint_traj.points;

      auto send_goal_future = action_client_->async_send_goal(goal);
      if (send_goal_future.wait_for(server_timeout) != std::future_status::ready) {
        publishResult(request_id, false, -5, "ros2_control action send goal timeout");
        return;
      }

      const auto goal_handle = send_goal_future.get();
      if (!goal_handle) {
        publishResult(request_id, false, -5, "ros2_control action goal rejected");
        return;
      }

      const auto result_timeout = std::chrono::duration<double>(action_result_timeout_sec_);
      auto result_future = action_client_->async_get_result(goal_handle);
      if (result_future.wait_for(result_timeout) != std::future_status::ready) {
        publishResult(request_id, false, -5, "ros2_control action result timeout");
        return;
      }

      const auto wrapped_result = result_future.get();
      if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED &&
        wrapped_result.result->error_code == control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL)
      {
        publishResult(request_id, true, 0, "execution succeeded");
      } else {
        publishResult(
          request_id, false, -5,
          "ros2_control action failed: error_code=" +
          std::to_string(wrapped_result.result->error_code));
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "trajectory execution failed: %s", e.what());
      publishResult(request_id, false, -4, e.what());
    }
  }

  //真机：逐点下发 /control/joint_states 并等待 /feedback/joint_states 到位
  void executeTrajectoryViaJointStates(const MotionTrajectory & msg)
  {
    struct ExecReset
    {
      ArmControllerNode * self;
      ~ExecReset()
      {
        std::lock_guard<std::mutex> lock(self->exec_mutex_);
        self->executing_ = false;
      }
    } exec_reset{this};

    const std::string request_id = msg.request_id;
    try {
      const auto points = extractJointPoints(msg);//把MotionTrajectory转成内部的JointWaypoint列表
      if (points.empty()) {
        publishResult(request_id, false, -2, "empty trajectory");
        return;
      }

      const auto period = std::chrono::duration<double>(1.0 / std::max(control_rate_hz_, 1.0));
      for (const auto & point : points) {
        //实际下发给驱动的命令
        sensor_msgs::msg::JointState cmd;
        cmd.header.stamp = now();
        cmd.name = point.names;
        cmd.position = point.positions;
        if (!point.efforts.empty()) {//如果有力控制，就下发力控制
          cmd.effort = point.efforts;
        }
        control_pub_->publish(cmd);

        if (!waitUntilReached(point.names, point.positions)) {
          publishResult(request_id, false, -3, "joint goal not reached in time");
          return;
        }
        std::this_thread::sleep_for(
          std::chrono::duration_cast<std::chrono::milliseconds>(period));
      }

      publishResult(request_id, true, 0, "execution succeeded");
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "trajectory execution failed: %s", e.what());
      publishResult(request_id, false, -4, e.what());
    }
  }

  //从轨迹中取关节点,把标准轨迹转成内部的数据结构std::vector<JointWaypoint>
  static std::vector<JointWaypoint> extractJointPoints(const MotionTrajectory & msg)
  {
    std::vector<JointWaypoint> points;
    const auto & joint_traj = msg.trajectory.joint_trajectory;
    if (joint_traj.points.empty()) {
      return points;
    }

    for (const auto & point : joint_traj.points) {
      JointWaypoint wp;
      wp.names = joint_traj.joint_names;
      wp.positions = point.positions;
      wp.efforts = point.effort;
      points.push_back(std::move(wp));
    }
    return points;
  }

  //判断机械臂是否到位
  bool waitUntilReached(
    const std::vector<std::string> & names,
    const std::vector<double> & target_positions) const
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(reach_timeout_sec_);

    std::map<std::string, double> target_map;
    //先把目标关节做成map
    for (size_t i = 0; i < names.size() && i < target_positions.size(); ++i) {
      target_map[names[i]] = target_positions[i];
    }

    //然后循环读取反馈
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      std::optional<sensor_msgs::msg::JointState> feedback;
      {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        feedback = latest_feedback_;
      }

      if (!feedback.has_value()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        continue;
      }

      std::map<std::string, double> current;
      for (size_t i = 0; i < feedback->name.size() && i < feedback->position.size(); ++i) {
        current[feedback->name[i]] = feedback->position[i];
      }

      bool all_reached = true;
      for (const auto & [name, target] : target_map) {
        const auto it = current.find(name);
        if (it == current.end() ||
          std::abs(it->second - target) > goal_tolerance_)//再比较容错
        {
          all_reached = false;
          break;
        }
      }

      if (all_reached) {
        return true;//如果所有关节都到位，就返回true
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;//如果超时，就返回false
  }

  //发布执行结果
  void publishResult(
    const std::string & request_id, bool success, int32_t error_code, const std::string & message)
  {
    ExecuteResult result;
    result.header.stamp = now();
    result.request_id = request_id;
    result.success = success;
    result.error_code = error_code;
    result.message = message;
    result_pub_->publish(result);
  }

  bool use_ros2_control_action_{false};
  double control_rate_hz_{50.0};
  double goal_tolerance_{0.02};
  double reach_timeout_sec_{5.0};
  double action_server_timeout_sec_{5.0};
  double action_result_timeout_sec_{120.0};

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr control_pub_;
  rclcpp::Publisher<ExecuteResult>::SharedPtr result_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr feedback_sub_;
  rclcpp::Subscription<MotionTrajectory>::SharedPtr traj_sub_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;

  mutable std::mutex feedback_mutex_;
  std::optional<sensor_msgs::msg::JointState> latest_feedback_;

  std::mutex exec_mutex_;
  bool executing_{false};
};

}  // namespace arm_controller_node

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<arm_controller_node::ArmControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

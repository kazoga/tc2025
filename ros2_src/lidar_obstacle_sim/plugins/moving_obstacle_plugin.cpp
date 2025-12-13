#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/physics/Model.hh>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <rclcpp/rclcpp.hpp>

namespace gazebo
{

class MovingObstaclePlugin : public ModelPlugin
{
public:
  MovingObstaclePlugin() = default;

  ~MovingObstaclePlugin() override
  {
    if (rclcpp::ok() && ros_node_ != nullptr)
    {
      executor_.remove_node(ros_node_);
    }
  }

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;

    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }

    use_sim_time_ = sdf->HasElement("use_sim_time") ? sdf->Get<bool>("use_sim_time") : true;
    robot_pose_topic_ = sdf->HasElement("robot_pose_topic") ?
      sdf->Get<std::string>("robot_pose_topic") : "/amcl_pose";

    if (sdf->HasElement("velocity"))
    {
      velocity_ = sdf->Get<ignition::math::Vector3d>("velocity");
    }

    if (sdf->HasElement("stop_distance"))
    {
      stop_distance_ = sdf->Get<double>("stop_distance");
    }

    if (sdf->HasElement("start_distance"))
    {
      start_distance_ = sdf->Get<double>("start_distance");
    }

    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    options.parameter_overrides({rclcpp::Parameter("use_sim_time", use_sim_time_)});
    ros_node_ = rclcpp::Node::make_shared("moving_obstacle_plugin", options);
    executor_.add_node(ros_node_);

    pose_sub_ = ros_node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      robot_pose_topic_, rclcpp::QoS(10),
      std::bind(&MovingObstaclePlugin::OnRobotPose, this, std::placeholders::_1));

    model_->SetLinearVel(ignition::math::Vector3d::Zero);

    update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&MovingObstaclePlugin::OnUpdate, this, std::placeholders::_1));

    RCLCPP_INFO(
      ros_node_->get_logger(),
      "MovingObstaclePlugin loaded for [%s]. start_distance=%.2f m, stop_distance=%.2f m, "
      "velocity=(%.2f, %.2f, %.2f), topic=%s",
      model_->GetName().c_str(), start_distance_, stop_distance_, velocity_.X(), velocity_.Y(),
      velocity_.Z(), robot_pose_topic_.c_str());
  }

private:
  void OnRobotPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    robot_pose_ = ignition::math::Pose3d(
      msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z, 0.0, 0.0,
      0.0);
    robot_pose_received_ = true;
  }

  void OnUpdate(const common::UpdateInfo &)
  {
    executor_.spin_some();

    ignition::math::Pose3d robot_pose;
    bool has_pose = false;
    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      if (robot_pose_received_)
      {
        robot_pose = robot_pose_;
        has_pose = true;
      }
    }

    if (!has_pose)
    {
      model_->SetLinearVel(ignition::math::Vector3d::Zero);
      return;
    }

    const ignition::math::Pose3d model_pose = model_->WorldPose();
    const double dx = model_pose.Pos().X() - robot_pose.Pos().X();
    const double dy = model_pose.Pos().Y() - robot_pose.Pos().Y();
    const double planar_distance = std::hypot(dx, dy);

    if (!moving_started_ && planar_distance <= start_distance_)
    {
      moving_started_ = true;
      model_->SetLinearVel(velocity_);
      RCLCPP_INFO(
        ros_node_->get_logger(), "[%s] 開始距離を検出したため走行を開始する (%.2f m)",
        model_->GetName().c_str(), planar_distance);
      return;
    }

    if (!moving_started_)
    {
      model_->SetLinearVel(ignition::math::Vector3d::Zero);
      return;
    }

    if (!stopped_ && planar_distance <= stop_distance_)
    {
      stopped_ = true;
      model_->SetLinearVel(ignition::math::Vector3d::Zero);
      RCLCPP_INFO(
        ros_node_->get_logger(), "[%s] 停止距離に到達したため停止する (%.2f m)",
        model_->GetName().c_str(), planar_distance);
      return;
    }

    if (!stopped_)
    {
      model_->SetLinearVel(velocity_);
    }
  }

  physics::ModelPtr model_;
  event::ConnectionPtr update_connection_;

  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::executors::SingleThreadedExecutor executor_;

  ignition::math::Vector3d velocity_{-0.5, 0.0, 0.0};
  double stop_distance_{1.0};
  double start_distance_{15.0};
  std::string robot_pose_topic_{"/amcl_pose"};
  bool use_sim_time_{true};

  bool robot_pose_received_{false};
  bool moving_started_{false};
  bool stopped_{false};
  ignition::math::Pose3d robot_pose_{0, 0, 0, 0, 0, 0};
  std::mutex pose_mutex_;
};

GZ_REGISTER_MODEL_PLUGIN(MovingObstaclePlugin)

}  // namespace gazebo


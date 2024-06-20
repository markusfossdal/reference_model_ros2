#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rcutils/logging_macros.h"
#include "std_msgs/msg/string.hpp"
#include "reference_model_siso.hpp"

using namespace std::chrono_literals;

class ReferenceModelVelocity : public rclcpp_lifecycle::LifecycleNode {
 public:
  explicit ReferenceModelVelocity(
      const std::string& node_name,
      bool intra_process_comms = false);

  void publish();

  void callback_subscriber(
      const geometry_msgs::msg::TwistStamped::SharedPtr msg);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State&);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State&);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State&);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State&);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State& state);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_error(const rclcpp_lifecycle::State&);

 private:
  size_t count;
  std::shared_ptr<
      rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::TwistStamped>>
      pub_state_, pub_state_dot_, pub_state_ddot_;
  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::TwistStamped>>
      sub_state_;
  std::shared_ptr<rclcpp::TimerBase> timer_;

  geometry_msgs::msg::TwistStamped desired_state;

  ReferenceFilterSiso model_x, model_y, model_z, model_phi, model_theta, model_psi; 
};

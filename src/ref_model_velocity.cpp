#include "ref_model_velocity.hpp"

explicit ReferenceModelVelocity::ReferenceModelVelocity(
    const std::string& node_name,
    bool intra_process_comms = false)
    : rclcpp_lifecycle::LifecycleNode(
          node_name,
          rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)),
      count(0) {
    
    //init desired_state
  desired_state.header.stamp = this->get_clock()->now();
  desired_state.twist.linear.x = 0.0;
  desired_state.twist.linear.y = 0.0;
  desired_state.twist.linear.z = 0.0;
  desired_state.twist.angular.x = 0.0;
  desired_state.twist.angular.y = 0.0;
  desired_state.twist.angular.z = 0.0;
}

void ReferenceModelVelocity::publish() {

  //compute reference signal
  auto model_x = ReferenceFilterSiso();
  auto model_y = ReferenceFilterSiso();
  auto model_z = ReferenceFilterSiso();
  auto model_psi = ReferenceFilterSiso();

  auto x_d = desired_state.twist.linear.x;
  auto y_d = desired_state.twist.linear.y;
  auto z_d = desired_state.twist.linear.z;
  auto psi_d = desired_state.twist.angular.z;

  auto compute_x = model_x.model_order_2();
  auto compute_y = model_y.model_order_2();
  auto compute_z = model_z.model_order_2();
  auto compute_psi = model_psi.model_order_2();

  auto ref_signal = model.get_eta();
  auto ref_signal_dot = model.get_eta_dot();

  //fetch time stamp
  auto current_time = this->get_clock()->now();

  //write message
  geometry_msgs::msg::TwistStamped msg;
  msg.header.stamp = current_time;
  msg.twist.linear.x = 1.0;
  msg.twist.linear.y = 1.0;
  msg.twist.linear.z = 1.0;
  msg.twist.angular.x = 1.0;
  msg.twist.angular.y = 1.0;
  msg.twist.angular.z = 1.0;
}

void ReferenceModelVelocity::callback_sub_data_1(
    const geometry_msgs::msg::TwistStamped::SharedPtr msg) {}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ReferenceModelVelocity::on_configure(const rclcpp_lifecycle::State&) {
  pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "reference_signal", 10);

  sub_data_1_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "subscription", 10,
      std::bind(
          &ReferenceModelVelocity::callback_sub_data_1, this,
          std::placeholders::_1));

  timer_ = this->create_wall_timer(
      1s, std::bind(&ReferenceModelVelocity::publish, this));

  RCLCPP_INFO(get_logger(), "on_configure() is called.");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ReferenceModelVelocity::on_activate(const rclcpp_lifecycle::State&) {
  pub_->on_activate();

  RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ReferenceModelVelocity::on_deactivate(const rclcpp_lifecycle::State&) {

  pub_->on_deactivate();

  RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ReferenceModelVelocity::on_cleanup(const rclcpp_lifecycle::State&) {
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ReferenceModelVelocity::on_shutdown(const rclcpp_lifecycle::State& state) {
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ReferenceModelVelocity::on_error(const rclcpp_lifecycle::State&) {
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::FAILURE;
}

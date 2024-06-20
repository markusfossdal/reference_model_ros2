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
  auto x_d = desired_state.twist.linear.x;
  auto y_d = desired_state.twist.linear.y;
  auto z_d = desired_state.twist.linear.z;
  auto phi_d = desired_state.twist.angular.x;
  auto theta_d = desired_state.twist.angular.y;
  auto psi_d = desired_state.twist.angular.z;

  model_x.model_order_2(x_d, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  model_y.model_order_2(x_d, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  model_z.model_order_2(x_d, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  model_phi.model_order_2(x_d, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  model_theta.model_order_2(x_d, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  model_psi.model_order_2(x_d, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0);

  //fetch time stamp
  auto current_time = this->get_clock()->now();

  //state_ddot
  geometry_msgs::msg::TwistStamped msg;
  msg.header.stamp = current_time;
  msg.twist.linear.x = model_x.get_state_ddot();
  msg.twist.linear.y = model_y.get_state_ddot();
  msg.twist.linear.z = model_z.get_state_ddot();
  msg.twist.angular.x = model_phi.get_state_ddot();
  msg.twist.angular.y = model_theta.get_state_ddot();
  msg.twist.angular.z = model_psi.get_state_ddot();

  pub_state_ddot_->publish(msg);

  //state_dot
  geometry_msgs::msg::TwistStamped msg;
  msg.header.stamp = current_time;
  msg.twist.linear.x = model_x.get_state_dot();
  msg.twist.linear.y = model_y.get_state_dot();
  msg.twist.linear.z = model_z.get_state_dot();
  msg.twist.angular.x = model_phi.get_state_dot();
  msg.twist.angular.y = model_theta.get_state_dot();
  msg.twist.angular.z = model_psi.get_state_dot();

  pub_state_dot_->publish(msg);

  //state
  geometry_msgs::msg::TwistStamped msg;
  msg.header.stamp = current_time;
  msg.twist.linear.x = model_x.get_state();
  msg.twist.linear.y = model_y.get_state();
  msg.twist.linear.z = model_z.get_state();
  msg.twist.angular.x = model_phi.get_state();
  msg.twist.angular.y = model_theta.get_state();
  msg.twist.angular.z = model_psi.get_state();

  pub_state_->publish(msg);
}

void ReferenceModelVelocity::callback_subscriber(
    const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
  //fetch time stamp
  auto current_time = this->get_clock()->now();

  //populate desired state
  desired_state.header.stamp = msg->header.stamp;
  desired_state.twist.linear.x = msg->twist.linear.x;
  desired_state.twist.linear.y = msg->twist.linear.y;
  desired_state.twist.linear.z = msg->twist.linear.z;
  desired_state.twist.angular.x = msg->twist.angular.x;
  desired_state.twist.angular.y = msg->twist.angular.y;
  desired_state.twist.angular.z = msg->twist.angular.z;
}


//parameter callbacks


rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ReferenceModelVelocity::on_configure(const rclcpp_lifecycle::State&) {
  pub_state_ddot_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "ref_state_ddot", 10);

  pub_state_dot_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "ref_state_dot", 10);

  pub_state_ =
      this->create_publisher<geometry_msgs::msg::TwistStamped>("ref_state", 10);

  sub_state_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "desired_state", 10,
      std::bind(
          &ReferenceModelVelocity::callback_subscriber, this,
          std::placeholders::_1));

  timer_ = this->create_wall_timer(
      1s, std::bind(&ReferenceModelVelocity::publish, this));

  RCLCPP_INFO(get_logger(), "on_configure() is called.");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ReferenceModelVelocity::on_activate(const rclcpp_lifecycle::State&) {
  pub_state_ddot_->on_activate();
  pub_state_dot_->on_activate();
  pub_state_->on_activate();

  RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ReferenceModelVelocity::on_deactivate(const rclcpp_lifecycle::State&) {

  pub_state_ddot_->on_deactivate();
  pub_state_dot_->on_deactivate();
  pub_state_->on_deactivate();

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

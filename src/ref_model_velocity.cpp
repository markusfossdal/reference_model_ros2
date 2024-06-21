#include "ref_model_velocity.hpp"

ReferenceModelVelocity::ReferenceModelVelocity(
    const std::string& node_name,
    bool intra_process_comms)
    : rclcpp_lifecycle::LifecycleNode(
          node_name,
          rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)),
      count(0) {

  //parameters
  declare_parameter<int>("param_pub_freq_hz", 100);
  get_parameter("param_pub_freq_hz", param_pub_freq_hz_);

  //pub topics
  declare_parameter<std::string>(
      "param_pub_state_ddot_topic", "ref_state_ddot");
  get_parameter("param_pub_state_ddot_topic", param_pub_state_ddot_topic_);

  declare_parameter<std::string>("param_pub_state_dot_topic", "ref_state_dot");
  get_parameter("param_pub_state_dot_topic", param_pub_state_dot_topic_);

  declare_parameter<std::string>("param_pub_state_topic", "ref_state");
  get_parameter("ref_state", param_pub_state_topic_);

  //sub topics
  declare_parameter<std::string>("param_sub_topic", "desired_state");
  get_parameter("param_sub_topic", param_sub_topic_);

  //qos buffer
  declare_parameter<int>("param_qos_buffer", 10);
  get_parameter("param_qos_buffer", param_qos_buffer_);

  pmap["rt_omega_n_x"] = 0.0;
  pmap["rt_zeta_x"] = 0.0;
  pmap["rt_sat_state_ddot_lower_x"] = 0.0;
  pmap["rt_sat_state_ddot_upper_x"] = 0.0;
  pmap["rt_sat_state_dot_lower_x"] = 0.0;
  pmap["rt_sat_state_dot_upper_x"] = 0.0;
  pmap["rt_dt_x"] = 0.0;

  pmap["rt_omega_n_y"] = 0.0;
  pmap["rt_zeta_y"] = 0.0;
  pmap["rt_sat_state_ddot_lower_y"] = 0.0;
  pmap["rt_sat_state_ddot_upper_y"] = 0.0;
  pmap["rt_sat_state_dot_lower_y"] = 0.0;
  pmap["rt_sat_state_dot_upper_y"] = 0.0;
  pmap["rt_dt_y"] = 0.0;

  pmap["rt_omega_n_z"] = 0.0;
  pmap["rt_zeta_z"] = 0.0;
  pmap["rt_sat_state_ddot_lower_z"] = 0.0;
  pmap["rt_sat_state_ddot_upper_z"] = 0.0;
  pmap["rt_sat_state_dot_lower_z"] = 0.0;
  pmap["rt_sat_state_dot_upper_z"] = 0.0;
  pmap["rt_dt_z"] = 0.0;

  pmap["rt_omega_n_phi"] = 0.0;
  pmap["rt_zeta_phi"] = 0.0;
  pmap["rt_sat_state_ddot_lower_phi"] = 0.0;
  pmap["rt_sat_state_ddot_upper_phi"] = 0.0;
  pmap["rt_sat_state_dot_lower_phi"] = 0.0;
  pmap["rt_sat_state_dot_upper_phi"] = 0.0;
  pmap["rt_dt_phi"] = 0.0;

  pmap["rt_omega_n_theta"] = 0.0;
  pmap["rt_zeta_theta"] = 0.0;
  pmap["rt_sat_state_ddot_lower_theta"] = 0.0;
  pmap["rt_sat_state_ddot_upper_theta"] = 0.0;
  pmap["rt_sat_state_dot_lower_theta"] = 0.0;
  pmap["rt_sat_state_dot_upper_theta"] = 0.0;
  pmap["rt_dt_theta"] = 0.0;

  pmap["rt_omega_n_psi"] = 0.0;
  pmap["rt_zeta_psi"] = 0.0;
  pmap["rt_sat_state_ddot_lower_psi"] = 0.0;
  pmap["rt_sat_state_ddot_upper_psi"] = 0.0;
  pmap["rt_sat_state_dot_lower_psi"] = 0.0;
  pmap["rt_sat_state_dot_upper_psi"] = 0.0;
  pmap["rt_dt_psi"] = 0.0;

  for (auto& pair : pmap) {
    declare_parameter<double>(pair.first, pair.second);
    get_parameter(pair.first, pair.second);
  }
}

void ReferenceModelVelocity::publish() {

  model_x.model_order_2(
      x_d, pmap["rt_omega_n_x"], pmap["rt_zeta_x"],
      pmap["rt_sat_state_ddot_lower_x"], pmap["rt_sat_state_ddot_upper_x"],
      pmap["rt_sat_state_dot_lower_x"], pmap["rt_sat_state_dot_upper_x"],
      pmap["rt_dt_x"]);
  model_y.model_order_2(
      y_d, pmap["rt_omega_n_y"], pmap["rt_zeta_y"],
      pmap["rt_sat_state_ddot_lower_y"], pmap["rt_sat_state_ddot_upper_y"],
      pmap["rt_sat_state_dot_lower_y"], pmap["rt_sat_state_dot_upper_y"],
      pmap["rt_dt_y"]);
  model_z.model_order_2(
      z_d, pmap["rt_omega_n_z"], pmap["rt_zeta_z"],
      pmap["rt_sat_state_ddot_lower_z"], pmap["rt_sat_state_ddot_upper_z"],
      pmap["rt_sat_state_dot_lower_z"], pmap["rt_sat_state_dot_upper_z"],
      pmap["rt_dt_z"]);
  model_phi.model_order_2(
      phi_d, pmap["rt_omega_n_phi"], pmap["rt_zeta_phi"],
      pmap["rt_sat_state_ddot_lower_phi"], pmap["rt_sat_state_ddot_upper_phi"],
      pmap["rt_sat_state_dot_lower_phi"], pmap["rt_sat_state_dot_upper_phi"],
      pmap["rt_dt_phi"]);
  model_theta.model_order_2(
      theta_d, pmap["rt_omega_n_theta"], pmap["rt_zeta_theta"],
      pmap["rt_sat_state_ddot_lower_theta"],
      pmap["rt_sat_state_ddot_upper_theta"],
      pmap["rt_sat_state_dot_lower_theta"],
      pmap["rt_sat_state_dot_upper_theta"], pmap["rt_dt_theta"]);
  model_psi.model_order_2(
      psi_d, pmap["rt_omega_n_psi"], pmap["rt_zeta_psi"],
      pmap["rt_sat_state_ddot_lower_psi"], pmap["rt_sat_state_ddot_upper_psi"],
      pmap["rt_sat_state_dot_lower_psi"], pmap["rt_sat_state_dot_upper_psi"],
      pmap["rt_dt_psi"]);

  //fetch time stamp
  auto current_time = this->get_clock()->now();

  //declare msg
  geometry_msgs::msg::TwistStamped msg;

  //publish state_ddot
  msg.header.stamp = current_time;
  msg.twist.linear.x = model_x.get_state_ddot();
  msg.twist.linear.y = model_y.get_state_ddot();
  msg.twist.linear.z = model_z.get_state_ddot();
  msg.twist.angular.x = model_phi.get_state_ddot();
  msg.twist.angular.y = model_theta.get_state_ddot();
  msg.twist.angular.z = model_psi.get_state_ddot();

  pub_state_ddot_->publish(msg);

  //reinitialize/clear msg
  msg = geometry_msgs::msg::TwistStamped();

  //publish state_dot
  msg.header.stamp = current_time;
  msg.twist.linear.x = model_x.get_state_dot();
  msg.twist.linear.y = model_y.get_state_dot();
  msg.twist.linear.z = model_z.get_state_dot();
  msg.twist.angular.x = model_phi.get_state_dot();
  msg.twist.angular.y = model_theta.get_state_dot();
  msg.twist.angular.z = model_psi.get_state_dot();

  pub_state_dot_->publish(msg);

  //reinitialize/clear msg
  msg = geometry_msgs::msg::TwistStamped();

  //publish state
  msg.header.stamp = current_time;
  msg.twist.linear.x = model_x.get_state();
  msg.twist.linear.y = model_y.get_state();
  msg.twist.linear.z = model_z.get_state();
  msg.twist.angular.x = model_phi.get_state();
  msg.twist.angular.y = model_theta.get_state();
  msg.twist.angular.z = model_psi.get_state();

  pub_state_->publish(msg);

  //reinitialize/clear msg
  msg = geometry_msgs::msg::TwistStamped();
}

void ReferenceModelVelocity::callback_subscriber(
    const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
  //populate desired states
  x_d = msg->twist.linear.x;
  y_d = msg->twist.linear.y;
  z_d = msg->twist.linear.z;
  phi_d = msg->twist.angular.x;
  theta_d = msg->twist.angular.y;
  psi_d = msg->twist.angular.z;
}

//parameter callback for parameters in pmap
//checks all parameters against the parameters in pmap, and updates the parameter value.
rcl_interfaces::msg::SetParametersResult ReferenceModelVelocity::paramsCallback(
    const std::vector<rclcpp::Parameter>& params) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  result.reason = "Not succesful!";
  for (const auto& param : params) {
    for (auto& pair : pmap) {
      if (param.get_name() == pair.first) {
        pair.second = param.as_double();
        result.reason = "Successful!";
        result.successful = true;
      } else {

        result.successful = false;
        result.reason = "Not succesful!";
      }
    }
  }
  return result;
};

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ReferenceModelVelocity::on_configure(const rclcpp_lifecycle::State&) {
  pub_state_ddot_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "ref_state_ddot", param_qos_buffer_);

  pub_state_dot_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "ref_state_dot", param_qos_buffer_);

  pub_state_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "ref_state", param_qos_buffer_);

  sub_state_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "desired_state", param_qos_buffer_,
      std::bind(
          &ReferenceModelVelocity::callback_subscriber, this,
          std::placeholders::_1));

  //hz to period in ns
  double freq_hz = param_pub_freq_hz_;
  double period_sec = 1.0 / freq_hz;
  std::chrono::nanoseconds period_ns(static_cast<long long>(period_sec * 1e9));
  timer_ = create_wall_timer(
      period_ns, std::bind(&ReferenceModelVelocity::publish, this));

  params_callback_handle_ = add_on_set_parameters_callback(std::bind(
      &ReferenceModelVelocity::paramsCallback, this, std::placeholders::_1));

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

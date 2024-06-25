# reference_model_ros2

Implements a ros2 version of a 2nd and 3rd order reference model for 6 DOF. The single input/single output implementation in cpp can be found [here](https://gitlab.com/markusfossdal/reference_model_siso). The ROS2 implementation is based on ROS2 Humble, using the Lifecycle node framework.

## Clone
```
git clone --recurse-submodules git@gitlab.com:markusfossdal/reference_model_ros2.git
```

## Input and Output

Subscribes to a desired signal with interface:
```
geometry_msgs::msg::TwistStamped msg
```

Publishes via **3x** publishers, one for the second time derivative-, the time derivative-, and the state using the interface:
```
geometry_msgs::msg::TwistStamped msg
```
## How-to

Run the model with one of the following. Note that the parameter config in /config is only read when using the *launch* script.

### ROS2 Run
If using `ros2 run` note that you need to `configure` and `activate` i.e. `ros2 lifecycle set /nodename configure`. 
```
ros2 run reference_model_ros2 ref_model_velocity
```

```
ros2 run reference_model_ros2 ref_model_position
```

### ROS2 Launch
```
ros2 launch reference_model_ros2 bringup_velocity.launch.py
```

```
ros2 launch reference_model_ros2 bringup_position.launch.py
```

## Dependencies
```
    rclcpp
    rclcpp_lifecycle
    std_msgs
    geometry_msgs
```

## Parameters and Config files
The parameters can be set offline prior to spinning the node, or online using rqt. Tuning the system online will *not* update the parameter file dynamically. 

### Example yaml-file:

```
reference_model_ros2:
  param_pub_freq_hz: 100
  param_pub_state_ddot_topic: "ref_state_ddot"
  param_pub_state_dot_topic: "ref_state_dot"
  param_pub_state_topic: "ref_state"
  param_qos_buffer: 10
  param_sub_topic: "desired_state"
  rt_dt_phi: 0.01
  rt_dt_psi: 0.01
  rt_dt_theta: 0.01
  rt_dt_x: 0.01
  rt_dt_y: 0.01
  rt_dt_z: 0.01
  rt_omega_n_phi: 1.0
  rt_omega_n_psi: 1.0
  rt_omega_n_theta: 1.0
  rt_omega_n_x: 1.0
  rt_omega_n_y: 1.0
  rt_omega_n_z: 1.0
  rt_sat_state_ddot_lower_phi: -100.0
  rt_sat_state_ddot_lower_psi: -100.0
  rt_sat_state_ddot_lower_theta: -100.0
  rt_sat_state_ddot_lower_x: -100.0
  rt_sat_state_ddot_lower_y: -100.0
  rt_sat_state_ddot_lower_z: -100.0
  rt_sat_state_ddot_upper_phi: 100.0
  rt_sat_state_ddot_upper_psi: 100.0
  rt_sat_state_ddot_upper_theta: 100.0
  rt_sat_state_ddot_upper_x: 100.0
  rt_sat_state_ddot_upper_y: 100.0
  rt_sat_state_ddot_upper_z: 100.0
  rt_sat_state_dot_lower_phi: -100.0
  rt_sat_state_dot_lower_psi: -100.0
  rt_sat_state_dot_lower_theta: -100.0
  rt_sat_state_dot_lower_x: -100.0
  rt_sat_state_dot_lower_y: -100.0
  rt_sat_state_dot_lower_z: -100.0
  rt_sat_state_dot_upper_phi: 100.0
  rt_sat_state_dot_upper_psi: 100.0
  rt_sat_state_dot_upper_theta: 100.0
  rt_sat_state_dot_upper_x: 100.0
  rt_sat_state_dot_upper_y: 100.0
  rt_sat_state_dot_upper_z: 100.0
  rt_zeta_phi: 1.0
  rt_zeta_psi: 1.0
  rt_zeta_theta: 1.0
  rt_zeta_x: 1.0
  rt_zeta_y: 1.0
  rt_zeta_z: 1.0

```

## References
1. Fossen, T. I. (2021). *Handbook of Marine Craft Hydrodynamics and Motion Control*, 2nd edition, John Wiley & Sons. Ltd., Chichester, UK. ISBN 978-1119575054.
/*
joystick_handler.
Copyright (C) 2019 Xuning Yang

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include <joystick_handler/JoystickHandler.h>
#include <joystick_handler/JoystickValues.h>

#include <ros_utils/ParameterUtils.h>

namespace planner {

JoystickHandler::JoystickHandler() {}
JoystickHandler::~JoystickHandler() {}

bool JoystickHandler::initialize(const ros::NodeHandle& n)
{
  name_ = ros::names::append(n.getNamespace(), "JoystickHandler");

  ros::NodeHandle nh(n);

  // Joystick deadband
  param_utils::get("joystick/enable_joystick_deadband", joystick_deadband_enabled_, true);
  param_utils::get("joystick/joystick_deadband", joystick_deadband_, (float)0.09);
  param_utils::get("joystick/sensitivity", sensitivity_, (float)0.00001);
  // param_utils::get("joystick/frequency", frequency_, (float)10);

  joy_sub_ = nh.subscribe("joy", 1, &JoystickHandler::joystickCallback, this);
  joy_raw_pub_ = nh.advertise<joystick_handler::JoystickValues>("joy_raw", 1);
  joy_first_order_pub_ = nh.advertise<joystick_handler::JoystickValues>("joy_first_order", 1);
  joy_filtered_pub_ =
    nh.advertise<joystick_handler::JoystickValues>("joy_filtered", 1);

  // joy_timer_ = nh.createTimer(ros::Duration(1.0/frequency_), &JoystickHandler::joystickTimer, this);

  previous_joy_input_ = Eigen::Vector4d(0.0, 0.0, 0.0, 0.0);
  previous_joy_raw_input_ = Eigen::Vector4d(0.0, 0.0, 0.0, 0.0);
  joy_input_ = Eigen::Vector4d(0.0, 0.0, 0.0, 0.0);
  previous_t_ = ros::Time::now();


  // Joy Mapper (can be replaced with default values)

  if (!joy_mapper_.initialize(n)) {
    ROS_ERROR("%s: Failed to initialize JoyMapper.", name_.c_str());
    return false;
  }

  if ((joystick_forward_ = joy_mapper_.get_axis_mapping_for("forward")) < 0)
    return false;
  if ((joystick_side_ = joy_mapper_.get_axis_mapping_for("side")) < 0)
    return false;
  if ((joystick_yaw_ = joy_mapper_.get_axis_mapping_for("yaw")) < 0)
    return false;
  if ((joystick_z_ = joy_mapper_.get_axis_mapping_for("z")) < 0)
    return false;

  // Joystick filtering
  param_utils::get("1euro_filter/enable", euro_filter_on_, false);
  param_utils::get("teleoperation/side_velocity_enabled", side_velocity_enabled_, false);

  if (euro_filter_on_) {
    joystick_filter_ = std::make_unique<JoystickFilter>();
    joystick_filter_->initialize(n);
  }

  std::cout << "=================================================" << std::endl;
  std::cout << "          Joystick Handling Parameters " << std::endl;
  std::cout << "=================================================" << std::endl;
  std::string deadband_string = (joystick_deadband_enabled_) ? "ON" : "OFF";
  std::cout << "Zero deadband: \t\t\t" << deadband_string << std::endl;
  if (joystick_deadband_enabled_)
  {
    std::cout << "   Deadband: \t\t" << joystick_deadband_ << std::endl;
  }
  std::cout << "Sensitivity: \t\t\t" << sensitivity_ << std::endl;

  std::string sv_string = (side_velocity_enabled_) ? "ON" : "OFF";
  std::cout << "Side velocity: \t\t\t" << sv_string << std::endl;

  std::string filter_string = (euro_filter_on_) ? "ON" : "OFF";
  std::cout << "1Euro Joystick Filtering: \t" << filter_string << std::endl;
  std::cout << "=================================================" << std::endl;

  return true;
}

double JoystickHandler::joyDead(double value, double deadband) {
  // Linear function with a deadzone around zero.
  // Maps [-1, 1] to [-1, -deadband], 0, [deadband, 1].
  if (value < deadband && value > -deadband) {
    return 0.0;
  }

  double m = 1 / (1 - deadband);
  if (value > 0) {
    return m * (value - deadband);
  }

  return m * (value + deadband);
}

void JoystickHandler::joystickCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  if (!control_io_.FSMModeEnabled("teleop")) return;

  if (joystick_forward_ >= static_cast<int>(msg->axes.size()) ||
      joystick_yaw_     >= static_cast<int>(msg->axes.size()) ||
      joystick_z_       >= static_cast<int>(msg->axes.size()) ||
      joystick_side_    >= static_cast<int>(msg->axes.size())) {
    ROS_ERROR("%s: not enough axes in joystick msg (size is %lu)!",
      name_.c_str(), msg->axes.size());
    return;
  }
  previous_joy_raw_input_ = joy_input_;

  joy_input_(FORWARD) = msg->axes[joystick_forward_];
  joy_input_(YAW) = msg->axes[joystick_yaw_];
  joy_input_(Z) = msg->axes[joystick_z_];
  joy_input_(SIDE) = side_velocity_enabled_ ? msg->axes[joystick_side_] : 0;

  // Publish the raw value (TOPIC NOT USED, FOR DEBUG ONLY. MESSAGE TYPE IS THE SAME AS JOY_FILTERED, SO COULD BE USED IF DESIRED TO BYPASS ANY KIND OF FILTERING.)
  joystick_handler::JoystickValues joy_raw_msg;

  joy_raw_msg.header.stamp = msg->header.stamp;
  joy_raw_msg.v_x = joy_input_(FORWARD);
  joy_raw_msg.v_side = joy_input_(SIDE);
  joy_raw_msg.v_z = joy_input_(Z);
  joy_raw_msg.omega = joy_input_(YAW);
  joy_raw_pub_.publish(joy_raw_msg);

  // Publish the first order derivative of the joystick values (TOPIC NOT USED, FOR DEBUG ONLY)
  joystick_handler::JoystickValues joy_first_order_msg;

  float dt = (msg->header.stamp - previous_t_).toSec();

  auto joy_first_order = (joy_input_ - previous_joy_raw_input_)/dt;
  auto joy_stream_diff = (joy_input_ - previous_joy_raw_input_);

  joy_first_order_msg.header.stamp = msg->header.stamp;
  joy_first_order_msg.v_x = joy_first_order(FORWARD);
  joy_first_order_msg.v_side = joy_first_order(SIDE);
  joy_first_order_msg.v_z = joy_first_order(Z);
  joy_first_order_msg.omega = joy_first_order(YAW);

  joy_first_order_pub_.publish(joy_first_order_msg);

  previous_joy_raw_input_ = joy_input_;
  previous_t_ = msg->header.stamp;

  // Joystick Filtering using 1 euro filter
  if (euro_filter_on_) {
    std::cout << "Joy input before: " << joy_input_ << std::endl;
    joy_input_ = joystick_filter_->filter(joy_input_);
    std::cout << "Joy input after: " << joy_input_ << std::endl;
  }

  // Apply deadband cancellation.
  if (joystick_deadband_enabled_)
  {
    for (int i = 0; i < 4; i++) {
      joy_input_(i) = joyDead(joy_input_(i), joystick_deadband_);
    }
  }

  // Check if the input has changed.
  bool change_in_input = false;
  change_in_input = (joy_input_ - previous_joy_input_).norm() > sensitivity_;

  // Publish the filtered input.

  ROS_DEBUG("Previous input: (%.2f, %.2f, %.2f, %.2f), current (%.2f, %.2f, %.2f, %.2f), difference: %.4f joy_first_order (%.2f, %.2f, %.2f, %.2f) with norm %.3f",
  previous_joy_input_(FORWARD), previous_joy_input_(SIDE), previous_joy_input_(Z), previous_joy_input_(YAW),
  joy_input_(FORWARD), joy_input_(SIDE), joy_input_(Z), joy_input_(YAW),
  (joy_input_ - previous_joy_input_).norm(),
  joy_first_order(FORWARD), joy_first_order(SIDE), joy_first_order(Z), joy_first_order(YAW), joy_first_order.norm());

  // if (change_in_input) {
  // place a check on the first_order norm if joy_node's autorepeat_rate is set. However, the hard-coded 2 should be a function of the autorepeat_rate parameter.
  if (change_in_input && joy_first_order.norm() < 2) { // should be equivalent as below given fixed dt.
  // if (change_in_input && joy_stream_diff.norm() < 0.05) {

    // ROS_INFO("Previous input: (%.2f, %.2f, %.2f, %.2f), current (%.2f, %.2f, %.2f, %.2f), difference: %.4f joy_first_order (%.2f, %.2f, %.2f, %.2f) with norm %.3f",
    // previous_joy_input_(FORWARD), previous_joy_input_(SIDE), previous_joy_input_(Z), previous_joy_input_(YAW),
    // joy_input_(FORWARD), joy_input_(SIDE), joy_input_(Z), joy_input_(YAW),
    // (joy_input_ - previous_joy_input_).norm(),
    // joy_first_order(FORWARD), joy_first_order(SIDE), joy_first_order(Z), joy_first_order(YAW), joy_first_order.norm());


    // Publish the mapped joystick inputs to the filtered topic.
    joystick_handler::JoystickValues joy_filtered_msg;

    joy_filtered_msg.header.stamp = msg->header.stamp;
    joy_filtered_msg.v_x = joy_input_(FORWARD);
    joy_filtered_msg.v_side = joy_input_(SIDE);
    joy_filtered_msg.v_z = joy_input_(Z);
    joy_filtered_msg.omega = joy_input_(YAW);
    joy_filtered_pub_.publish(joy_filtered_msg);
    previous_joy_input_ = joy_input_;
  }


}

} // namespace planner

/*
joystick_handler
Copyright (C) 2019 Xuning Yang

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include <iostream>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Joy.h>

#include <control_arch/FsmFlags.h>
#include <geometry_utils/GeometryUtils.h>
#include <joystick_handler/filters/JoystickFilter.h>
#include <joystick_ui/JoyMapper.h>

namespace planner {

// class JoystickHandler filters the joystick inputs and generates the appropriate input to execute teleoperation.
class JoystickHandler
{
public:
  JoystickHandler();
  ~JoystickHandler();
  bool initialize(const ros::NodeHandle &n);

private:
  void flagsCallback(const control_arch::FsmFlags::ConstPtr& msg);
  void joystickCallback(const sensor_msgs::Joy::ConstPtr& msg);

  bool flagEnabledQ(const std::string &flag);

  double joyDead(double value, double deadband);


  std::string name_;                            // Node name
  std::string fixed_frame_id_;                // Vehicle base frame ID
  std::string vehicle_frame_id_;                // Vehicle base frame ID
  std::set<std::string> flags_;                 // Flags for state of vehicle

  ros::Subscriber joy_sub_;
  ros::Subscriber flags_sub_;

  ros::Publisher joy_filtered_pub_;
  ros::Publisher joy_raw_pub_;


  geometry_utils::Vec4 joy_input_;        // Raw joystick values in [-1, 1]
  geometry_utils::Vec4 previous_joy_input_;        // Raw joystick values in [-1, 1]

  bool side_velocity_enabled_;

  // Joystick deadband
  bool joystick_deadband_enabled_;
  float joystick_deadband_;
  float sensitivity_;

  // Joystick mapping
  JoyMapper joy_mapper_;
  int joystick_forward_, joystick_side_, joystick_z_, joystick_yaw_;
  enum { FORWARD, YAW, Z, SIDE }; // Indices into the above x_input_ Vec4s

  // Joystick filtering
  bool euro_filter_on_;
  std::unique_ptr<JoystickFilter> joystick_filter_;

};

} // namespace planner

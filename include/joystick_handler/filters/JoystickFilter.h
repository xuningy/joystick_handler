/*
JoystickFilter: filters joystick inputs using the OneEuroFilter
Copyright (C) 2019 Xuning Yang

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include <chrono>
#include <memory>
#include <Eigen/Core>
#include <ros/ros.h>

#include <ros_utils/ParameterUtils.h>

#include <one_euro_filter/OneEuroFilter.h>

class JoystickFilter
{
public:
  JoystickFilter() {}
  ~JoystickFilter() {}

  bool initialize(const ros::NodeHandle& n) {

    ros::NodeHandle nh(n);

    // Frequency is the same as the joystick polling frequency.
    // This frequency is also updated internally by the OneEuroFilter.
    double period;
    param_utils::get("teleoperation/traj_gen_rate", period, 0.1);
    freq_ = 1 / period;
    if (!param_utils::get("1euro_filter/mincutoff", mincutoff_)) return false;
    if (!param_utils::get("1euro_filter/beta", beta_)) return false;
    if (!param_utils::get("1euro_filter/dcutoff", dcutoff_)) return false;

    // Start Four OneEuroFilters, one for each input.
    linear_vel_ = std::make_unique<OneEuroFilter>(
                                          freq_, mincutoff_, beta_, dcutoff_);
    angular_vel_ = std::make_unique<OneEuroFilter>(
                                          freq_, mincutoff_, beta_, dcutoff_);
    z_vel_ = std::make_unique<OneEuroFilter>(
                                          freq_, mincutoff_, beta_, dcutoff_);
    side_vel_ = std::make_unique<OneEuroFilter>(
                                          freq_, mincutoff_, beta_, dcutoff_);
    printParameters();

    return true;
  }

  Eigen::Vector4d filter(const Eigen::Vector4d& joy_input) {
    auto now = std::chrono::high_resolution_clock::now();
    Eigen::Vector4d joy_input_filtered;

    joy_input_filtered(0) = linear_vel_->filter(joy_input(0), now);
    joy_input_filtered(1) = angular_vel_->filter(joy_input(1), now);
    joy_input_filtered(2) = z_vel_->filter(joy_input(2), now);
    joy_input_filtered(3) = side_vel_->filter(joy_input(3), now);

    return joy_input_filtered;
  }

private:

  // OneEuroFilter Parameters
  double freq_;
  double mincutoff_;
  double beta_;
  double dcutoff_;

  // Filters
  std::unique_ptr<OneEuroFilter> linear_vel_;
  std::unique_ptr<OneEuroFilter> side_vel_;
  std::unique_ptr<OneEuroFilter> angular_vel_;
  std::unique_ptr<OneEuroFilter> z_vel_;

  // Print parameters
  void printParameters() {
    std::cout << "=============================" << std::endl;
    std::cout << "   Joystick 1Euro Filter" << std::endl;
    std::cout << "=============================" << std::endl;
    std::cout << "frequency: \t" << freq_ << " Hz" << std::endl;
    std::cout << "mincutoff: \t" << mincutoff_ << " Hz" << std::endl;
    std::cout << "beta: \t\t" << beta_ << std::endl;
    std::cout << "dcutoff: \t" << dcutoff_ << " Hz" << std::endl;
    std::cout << "=============================" << std::endl;
  }

};

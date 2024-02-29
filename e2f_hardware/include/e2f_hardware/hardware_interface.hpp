//  Copyright 2023 Sakai Hibiki
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "control_toolbox/pid.hpp"
#include "e2f_hardware/e2f.hpp"
#include "hardware_interface/system_interface.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "rclcpp/rclcpp.hpp"

namespace e2f_hardware
{

class E2FHardwareInterface : public hardware_interface::SystemInterface
{
  std::unique_ptr<E2F> e2f_;

  // Interfaces
  std::map<JointName, double> joint_positions_;
  double stroke_position_, stroke_velocity_, stroke_effort_;
  double stroke_position_command_, stroke_effort_command_;
  double tcp_position_;

  std::vector<std::shared_ptr<KDL::Chain>> chains_;
  std::vector<KDL::ChainFkSolverPos_recursive> fk_pos_solvers_;
  control_toolbox::Pid pid_;
  double moment_arm_;
  double reference_joint_position_;  // Angle of the finger link at the initial position

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(E2FHardwareInterface)

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  double compute_force(double actuator_position, double link_position) const;
  double stroke_ik(double stroke);
};

}  // namespace e2f_hardware

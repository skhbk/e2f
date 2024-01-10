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

#include "e2f_controllers/gripper_command_controller.hpp"

#include <algorithm>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace e2f_controllers
{

using controller_interface::CallbackReturn;
using controller_interface::InterfaceConfiguration;

CallbackReturn GripperCommandController::on_init()
{
  param_listener_ = std::make_shared<gripper_command_controller::ParamListener>(this->get_node());
  params_ = param_listener_->get_params();

  return CallbackReturn::SUCCESS;
}

InterfaceConfiguration GripperCommandController::command_interface_configuration() const
{
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {params_.joint + "/" + hardware_interface::HW_IF_POSITION}};
}

InterfaceConfiguration GripperCommandController::state_interface_configuration() const
{
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {params_.joint + "/" + hardware_interface::HW_IF_POSITION,
     params_.joint + "/" + hardware_interface::HW_IF_EFFORT}};
}

CallbackReturn GripperCommandController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();

  command_subscription_ = this->get_node()->create_subscription<CmdType>(
    "~/command", rclcpp::SensorDataQoS(),
    [this](const CmdType::SharedPtr msg) { rt_buffer_.writeFromNonRT(msg); });

  // Reset PID parameters
  pid_.initPid(
    params_.pid.p, params_.pid.i, params_.pid.d, params_.pid.i_clamp, -params_.pid.i_clamp);

  return CallbackReturn::SUCCESS;
}

CallbackReturn GripperCommandController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  rt_buffer_.reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn GripperCommandController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type GripperCommandController::update(
  const rclcpp::Time & /* time */, const rclcpp::Duration & period)
{
  const auto command = *rt_buffer_.readFromRT();

  if (!command) {
    return controller_interface::return_type::OK;
  }

  auto & command_interface = command_interfaces_.at(0);
  const auto & position = state_interfaces_.at(0).get_value();
  const auto & effort = state_interfaces_.at(1).get_value();

  const auto effort_error =
    effort - std::clamp(command->max_effort, params_.min_effort, params_.max_effort);
  const auto position_error = position - command->position;

  desired_position_ = command->position;

  if (position_error > params_.position_tolerance && effort > params_.effort_tolerance) {
    desired_position_ += pid_.computeCommand(effort_error, period.nanoseconds());
  } else {
    pid_.reset();
  }

  command_interface.set_value(desired_position_);

  return controller_interface::return_type::OK;
}

}  // namespace e2f_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  e2f_controllers::GripperCommandController, controller_interface::ControllerInterface)

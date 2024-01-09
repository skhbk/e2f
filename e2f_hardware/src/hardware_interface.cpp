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

#include "e2f_hardware/hardware_interface.hpp"

#include "control_toolbox/filters.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace e2f_hardware
{
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

CallbackReturn E2FHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  stroke_position_command_ = std::numeric_limits<double>::quiet_NaN();

  const auto device_name = info_.hardware_parameters.at("device_name");
  const auto baud_rate = std::stoi(info_.hardware_parameters.at("baud_rate"));
  const auto id = std::stoi(info_.hardware_parameters.at("dynamixel_id"));
  const auto tf_prefix = info_.hardware_parameters.at("tf_prefix");

  // Initialize device
  try {
    e2f_ = std::make_unique<E2F>(device_name, baud_rate, id);
    joint_positions_ = e2f_->read();
  } catch (std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("E2FHardwareInterface"), e.what());
    return CallbackReturn::ERROR;
  }

  // Kinematics
  {
    KDL::Tree tree;
    kdl_parser::treeFromString(info_.original_xml, tree);

    // Initialize chain and solver
    for (const auto & chain_tip : {tf_prefix + "fingertip_l", tf_prefix + "fingertip_r"}) {
      const auto & chain = chains_.emplace_back(std::make_shared<KDL::Chain>());
      tree.getChain(tf_prefix + "base_link", chain_tip, *chain);
      fk_pos_solvers_.emplace_back(*chain);
    }

    // Compute moment arm and reference joint position
    const auto lever_pose = tree.getSegment(tf_prefix + "link3_l")->second.segment.pose(0.);
    moment_arm_ = lever_pose.p.Norm();
    reference_joint_position_ = std::atan2(lever_pose.p.z(), lever_pose.p.x());
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> E2FHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  const auto tf_prefix = info_.hardware_parameters.at("tf_prefix");

  // Joint interfaces
  state_interfaces.emplace_back(
    tf_prefix + "active_joint", hardware_interface::HW_IF_POSITION,
    &joint_positions_[JointName::ACTIVE]);
  state_interfaces.emplace_back(
    tf_prefix + "passive_l_joint", hardware_interface::HW_IF_POSITION,
    &joint_positions_[JointName::PASSIVE_L]);
  state_interfaces.emplace_back(
    tf_prefix + "passive_r_joint", hardware_interface::HW_IF_POSITION,
    &joint_positions_[JointName::PASSIVE_R]);

  // Stroke interfaces
  state_interfaces.emplace_back(tf_prefix + "stroke", "width", &stroke_position_);
  state_interfaces.emplace_back(tf_prefix + "stroke", "force", &stroke_effort_);

  // TCP interface
  state_interfaces.emplace_back(
    tf_prefix + "tcp_joint", hardware_interface::HW_IF_POSITION, &tcp_position_);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> E2FHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  const auto tf_prefix = info_.hardware_parameters.at("tf_prefix");

  command_interfaces.emplace_back(tf_prefix + "stroke", "width", &stroke_position_command_);

  return command_interfaces;
}

return_type E2FHardwareInterface::prepare_command_mode_switch(
  const std::vector<std::string> & /*start_interfaces*/,
  const std::vector<std::string> & /*stop_interfaces*/)
{
  return return_type::OK;
}

return_type E2FHardwareInterface::perform_command_mode_switch(
  const std::vector<std::string> & /*start_interfaces*/,
  const std::vector<std::string> & /*stop_interfaces*/)
{
  return return_type::OK;
}

CallbackReturn E2FHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  try {
    e2f_->activate();
  } catch (std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("E2FHardwareInterface"), e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn E2FHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Currently, this method never seems to be called

  return CallbackReturn::SUCCESS;
}

return_type E2FHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  try {
    auto current_joint_positions = e2f_->read();
    current_joint_positions.at(JointName::ACTIVE) -=
      std::stod(info_.hardware_parameters.at("active_joint_offset"));

    joint_positions_.at(JointName::ACTIVE) = current_joint_positions.at(JointName::ACTIVE);

    // Apply exponential filter
    for (const auto & key : {JointName::PASSIVE_L, JointName::PASSIVE_R}) {
      joint_positions_.at(key) = filters::exponentialSmoothing(
        current_joint_positions.at(key), joint_positions_.at(key),
        std::stod(info_.hardware_parameters.at("filter_coefficient")));
    }
  } catch (std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("E2FHardwareInterface"), e.what());
    return return_type::ERROR;
  }

  // Stroke and TCP positions
  {
    std::array<JointName, 2> joint_names{JointName::PASSIVE_L, JointName::PASSIVE_R};
    std::array<KDL::Frame, 2> frames;

    for (size_t i = 0; i < 2; ++i) {
      KDL::JntArray q;
      const auto & pos = joint_positions_.at(joint_names[i]);
      q.data = Eigen::Vector2d{pos, -pos};
      fk_pos_solvers_[i].JntToCart(q, frames[i]);
    }

    stroke_position_ = frames[0].p.x() - frames[1].p.x();
    tcp_position_ = (frames[0].p.z() + frames[1].p.z()) / 2;
  }

  // Stroke effort
  {
    const auto effort_l = this->compute_force(
      joint_positions_.at(JointName::ACTIVE), joint_positions_.at(JointName::PASSIVE_L));
    const auto effort_r = this->compute_force(
      joint_positions_.at(JointName::ACTIVE), joint_positions_.at(JointName::PASSIVE_R));
    stroke_effort_ = std::max(effort_l, effort_r);
  }

  return return_type::OK;
}

return_type E2FHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!std::isfinite(stroke_position_command_)) {
    return return_type::OK;
  }

  try {
    e2f_->write(stroke_ik(stroke_position_command_));
  } catch (std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("E2FHardwareInterface"), e.what());
    return return_type::ERROR;
  }

  return return_type::OK;
}

double E2FHardwareInterface::compute_force(double actuator_position, double link_position) const
{
  const double spring_constant =
    std::stod(info_.hardware_parameters.at("spring_constant")) * 180 / M_PI / 1000;  // [Nm/rad]

  return spring_constant * (actuator_position - link_position) /
         (moment_arm_ * std::abs(std::sin(link_position + reference_joint_position_)));
}

double E2FHardwareInterface::stroke_ik(double stroke)
{
  // Compute stroke of initial position
  double reference_stroke = 0;
  for (size_t i = 0; i < 2; ++i) {
    KDL::JntArray q;
    q.data = Eigen::Vector2d{0., 0.};
    KDL::Frame frame;
    fk_pos_solvers_[i].JntToCart(q, frame);
    reference_stroke += std::abs(frame.p.x());
  }

  const double stroke_diff = reference_stroke - stroke;
  const auto joint_offset = std::stod(info_.hardware_parameters.at("active_joint_offset"));

  if (stroke_diff < 0) {
    return joint_offset;
  }

  return std::acos(std::cos(reference_joint_position_) - stroke_diff / 2 / moment_arm_) -
         reference_joint_position_ + joint_offset;
}

}  // namespace e2f_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(e2f_hardware::E2FHardwareInterface, hardware_interface::SystemInterface)

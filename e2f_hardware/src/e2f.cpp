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

#include "e2f_hardware/e2f.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace e2f_hardware
{

E2F::E2F(const std::string & device_name, uint32_t baud_rate, uint8_t id) : id_(id)
{
  if (!dxl_wb_.init(device_name.c_str(), baud_rate)) {
    throw std::runtime_error("Failed to initialize");
  }

  std::this_thread::sleep_for(std::chrono::seconds(1));

  if (!dxl_wb_.reboot(id_)) {
    throw std::runtime_error("Failed to reboot");
  }

  if (!dxl_wb_.ping(id_)) {
    throw std::runtime_error("Failed to ping");
  }

  if (!dxl_wb_.itemWrite(id_, "External_Port_Mode_1", 0)) {
    throw std::runtime_error("Failed to write to External_Port_Mode_1");
  }

  if (!dxl_wb_.itemWrite(id_, "External_Port_Mode_2", 0)) {
    throw std::runtime_error("Failed to write to External_Port_Mode_2");
  }
}

E2F::~E2F() {}

void E2F::activate()
{
  if (!dxl_wb_.jointMode(id_, 0, 0)) {
    throw std::runtime_error("Failed to set to joint mode");
  }

  if (!dxl_wb_.itemWrite(id_, "Position_P_Gain", 100)) {
    throw std::runtime_error("Failed to write to Position_P_Gain");
  }

  if (!dxl_wb_.itemWrite(id_, "Position_I_Gain", 10)) {
    throw std::runtime_error("Failed to write to Position_I_Gain");
  }
}

std::map<JointName, double> E2F::read()
{
  std::map<JointName, double> positions;
  int32_t data;

  // Active joint
  {
    float position;
    if (!dxl_wb_.itemRead(id_, "Present_Position", &data)) {
      throw std::runtime_error("Failed to read Present_Position");
    }
    positions[JointName::ACTIVE] = dxl_wb_.convertValue2Radian(id_, data);
  }

  // Passive joints
  {
    constexpr double effective_range = 333.3 * M_PI / 180;  // According to the RDC50 specification
    constexpr double gain = effective_range / 4095;         // 4095: 12 bit resolution
    constexpr double bias = -effective_range / 2;           // RDC50 center position

    if (!dxl_wb_.itemRead(id_, "External_Port_Data_2", &data)) {
      throw std::runtime_error("Failed to read External_Port_Data_2");
    }
    positions[JointName::PASSIVE_L] = -data * gain - bias;

    if (!dxl_wb_.itemRead(id_, "External_Port_Data_1", &data)) {
      throw std::runtime_error("Failed to read External_Port_Data_1");
    }
    positions[JointName::PASSIVE_R] = data * gain + bias;
  }

  return positions;
}

void E2F::write(double command)
{
  if (!dxl_wb_.goalPosition(id_, static_cast<float>(command))) {
    throw std::runtime_error("Failed to set goal position");
  }
}

}  // namespace e2f_hardware

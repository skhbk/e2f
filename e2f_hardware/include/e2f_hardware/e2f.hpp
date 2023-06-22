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
#include <string>

#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"

namespace e2f_hardware
{

enum class JointName { ACTIVE, PASSIVE_L, PASSIVE_R };

class E2F
{
  DynamixelWorkbench dxl_wb_;
  uint8_t id_;

public:
  E2F(const std::string & device_name, uint32_t baud_rate, uint8_t id);
  ~E2F();
  void activate();
  std::map<JointName, double> read();
  void write(double command);
};

}  // namespace e2f_hardware

/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/*
 * dynamixel.cpp
 *
 *  Created on: 2015. 12. 8.
 *      Author: zerom
 */

#include "robotis_device/dynamixel.h"

using namespace robotis_framework;

// Constructor của lớp Dynamixel
Dynamixel::Dynamixel(int id, std::string model_name, float protocol_version)
  : ctrl_module_name_("none"),                              // Tên module kiểm soát mặc định là "none"
    torque_to_current_value_ratio_(1.0),                   // Tỉ lệ chuyển đổi từ mô-men xoắn sang giá trị dòng điện mặc định là 1.0
    velocity_to_value_ratio_(1.0),                         // Tỉ lệ chuyển đổi từ vận tốc sang giá trị số mặc định là 1.0
    value_of_0_radian_position_(0),                        // Giá trị vị trí 0 radian mặc định là 0
    value_of_min_radian_position_(0),                      // Giá trị vị trí radian tối thiểu mặc định là 0
    value_of_max_radian_position_(0),                      // Giá trị vị trí radian tối đa mặc định là 0
    min_radian_(-3.14159265),                              // Giá trị góc radian tối thiểu mặc định là -π
    max_radian_(3.14159265),                               // Giá trị góc radian tối đa mặc định là π
    torque_enable_item_(0),                                // Thành phần kiểm soát mô-men xoắn mặc định là 0
    present_position_item_(0),                             // Thành phần kiểm soát vị trí hiện tại mặc định là 0
    present_velocity_item_(0),                             // Thành phần kiểm soát vận tốc hiện tại mặc định là 0
    present_current_item_(0),                              // Thành phần kiểm soát dòng điện hiện tại mặc định là 0
    goal_position_item_(0),                                // Thành phần kiểm soát vị trí mong muốn mặc định là 0
    goal_velocity_item_(0),                                // Thành phần kiểm soát vận tốc mong muốn mặc định là 0
    goal_current_item_(0),                                 // Thành phần kiểm soát dòng điện mong muốn mặc định là 0
    position_p_gain_item_(0),                              // Thành phần kiểm soát hệ số lợi tỷ số P của vị trí mặc định là 0
    position_i_gain_item_(0),                              // Thành phần kiểm soát hệ số lợi tỷ số I của vị trí mặc định là 0
    position_d_gain_item_(0),                              // Thành phần kiểm soát hệ số lợi tỷ số D của vị trí mặc định là 0
    velocity_p_gain_item_(0),                              // Thành phần kiểm soát hệ số lợi tỷ số P của vận tốc mặc định là 0
    velocity_i_gain_item_(0),                              // Thành phần kiểm soát hệ số lợi tỷ số I của vận tốc mặc định là 0
    velocity_d_gain_item_(0)                               // Thành phần kiểm soát hệ số lợi tỷ số D của vận tốc mặc định là 0
{
  // Gán các giá trị đầu vào cho các biến thành viên
  this->id_ = id;
  this->model_name_ = model_name;
  this->port_name_ = "";
  this->protocol_version_ = protocol_version;

  // Xóa dữ liệu trong bảng kiểm soát
  ctrl_table_.clear();

  // Khởi tạo trạng thái của động cơ Dynamixel
  dxl_state_ = new DynamixelState();

  // Xóa dữ liệu trong danh sách đọc nhiều giá trị
  bulk_read_items_.clear();
}


// Hàm chuyển đổi giá trị đọc từ động cơ sang góc radian
double Dynamixel::convertValue2Radian(int32_t value)
{
  double radian = 0.0;

  // Kiểm tra nếu giá trị đọc lớn hơn giá trị vị trí 0 radian
  if (value > value_of_0_radian_position_)
  {
    // Nếu giá trị radian tối đa không dương, trả về giá trị radian tối đa
    if (max_radian_ <= 0)
      return max_radian_;

    // Tính toán giá trị radian dựa trên giá trị đọc, giá trị vị trí 0 radian, và giá trị tối đa
    radian = (double) (value - value_of_0_radian_position_) * max_radian_
               / (double) (value_of_max_radian_position_ - value_of_0_radian_position_);
  }
  // Kiểm tra nếu giá trị đọc nhỏ hơn giá trị vị trí 0 radian
  else if (value < value_of_0_radian_position_)
  {
    // Nếu giá trị radian tối thiểu không dương, trả về giá trị radian tối thiểu
    if (min_radian_ >= 0)
      return min_radian_;

    // Tính toán giá trị radian dựa trên giá trị đọc, giá trị vị trí 0 radian, và giá trị tối thiểu
    radian = (double) (value - value_of_0_radian_position_) * min_radian_
               / (double) (value_of_min_radian_position_ - value_of_0_radian_position_);
  }

  // Kiểm tra và giới hạn giá trị radian nếu cần thiết
  // if (radian > max_radian_)
  //    return max_radian_;
  // else if (radian < min_radian_)
  //    return min_radian_;

  // Trả về giá trị radian
  return radian;
}




// Hàm chuyển đổi từ góc radian sang giá trị đọc của động cơ Dynamixel
int32_t Dynamixel::convertRadian2Value(double radian)
{
  int32_t value = 0;

  // Kiểm tra nếu góc radian lớn hơn 0
  if (radian > 0)
  {
    // Nếu giá trị vị trí radian tối đa không lớn hơn hoặc bằng giá trị vị trí 0 radian,
    // trả về giá trị vị trí radian tối đa
    if (value_of_max_radian_position_ <= value_of_0_radian_position_)
      return value_of_max_radian_position_;

    // Tính toán giá trị đọc dựa trên góc radian, giá trị vị trí 0 radian, và giá trị tối đa
    value = static_cast<int32_t>((radian * (value_of_max_radian_position_ - value_of_0_radian_position_) / max_radian_)
                + value_of_0_radian_position_);
  }
  // Kiểm tra nếu góc radian nhỏ hơn 0
  else if (radian < 0)
  {
    // Nếu giá trị vị trí radian tối thiểu không lớn hơn hoặc bằng giá trị vị trí 0 radian,
    // trả về giá trị vị trí radian tối thiểu
    if (value_of_min_radian_position_ >= value_of_0_radian_position_)
      return value_of_min_radian_position_;

    // Tính toán giá trị đọc dựa trên góc radian, giá trị vị trí 0 radian, và giá trị tối thiểu
    value = static_cast<int32_t>((radian * (value_of_min_radian_position_ - value_of_0_radian_position_) / min_radian_)
                + value_of_0_radian_position_);
  }
  // Trường hợp góc radian bằng 0
  else
    value = value_of_0_radian_position_;

  // Kiểm tra và giới hạn giá trị đọc nếu cần thiết
  // if (value > value_of_max_radian_position_)
  //    return value_of_max_radian_position_;
  // else if (value < value_of_min_radian_position_)
  //    return value_of_min_radian_position_;

  // Trả về giá trị đọc đã tính toán
  return value;
}



// Hàm chuyển đổi giá trị đọc từ động cơ Dynamixel sang giá trị vận tốc
double Dynamixel::convertValue2Velocity(int32_t value)
{
  return static_cast<double>(value) / velocity_to_value_ratio_;
}

// Hàm chuyển đổi giá trị vận tốc sang giá trị đọc của động cơ Dynamixel
int32_t Dynamixel::convertVelocity2Value(double velocity)
{
  return static_cast<int32_t>(velocity * velocity_to_value_ratio_);
}

// Hàm chuyển đổi giá trị đọc từ động cơ Dynamixel sang giá trị mô-men xoắn
double Dynamixel::convertValue2Torque(int16_t value)
{
  return static_cast<double>(value) / torque_to_current_value_ratio_;
}

// Hàm chuyển đổi giá trị mô-men xoắn sang giá trị đọc của động cơ Dynamixel
int16_t Dynamixel::convertTorque2Value(double torque)
{
  return static_cast<int16_t>(torque * torque_to_current_value_ratio_);
}

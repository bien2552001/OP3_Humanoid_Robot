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
 * dynamixel.h
 *
 *  Created on: 2015. 12. 8.
 *      Author: zerom
 */

#ifndef ROBOTIS_DEVICE_DYNAMIXEL_H_
#define ROBOTIS_DEVICE_DYNAMIXEL_H_

#include <map>
#include <vector>
#include <string>

#include "control_table_item.h"
#include "device.h"
#include "dynamixel_state.h"

namespace robotis_framework
{

  class Dynamixel : public Device
  {
  public:
    std::string ctrl_module_name_; // Tên của module kiểm soát
    DynamixelState *dxl_state_;    // Con trỏ đến đối tượng DynamixelState

    double velocity_to_value_ratio_;       // Tỉ lệ chuyển đổi từ vận tốc sang giá trị số
    double torque_to_current_value_ratio_; // Tỉ lệ chuyển đổi từ mô-men xoắn sang giá trị dòng điện

    int32_t value_of_0_radian_position_;   // Giá trị của vị trí 0 radian
    int32_t value_of_min_radian_position_; // Giá trị của vị trí radian tối thiểu
    int32_t value_of_max_radian_position_; // Giá trị của vị trí radian tối đa
    double min_radian_;                    // Giá trị của góc radian tối thiểu
    double max_radian_;                    // Giá trị của góc radian tối đa

    ControlTableItem *torque_enable_item_;    // Thành phần bảng kiểm soát cho việc bật/tắt mô-men xoắn
    ControlTableItem *present_position_item_; // Thành phần bảng kiểm soát cho vị trí hiện tại
    ControlTableItem *present_velocity_item_; // Thành phần bảng kiểm soát cho vận tốc hiện tại
    ControlTableItem *present_current_item_;  // Thành phần bảng kiểm soát cho dòng điện hiện tại
    ControlTableItem *goal_position_item_;    // Thành phần bảng kiểm soát cho vị trí mong muốn
    ControlTableItem *goal_velocity_item_;    // Thành phần bảng kiểm soát cho vận tốc mong muốn
    ControlTableItem *goal_current_item_;     // Thành phần bảng kiểm soát cho dòng điện mong muốn
    ControlTableItem *position_p_gain_item_;  // Thành phần bảng kiểm soát cho hệ số lợi tỷ số P của vị trí
    ControlTableItem *position_i_gain_item_;  // Thành phần bảng kiểm soát cho hệ số lợi tỷ số I của vị trí
    ControlTableItem *position_d_gain_item_;  // Thành phần bảng kiểm soát cho hệ số lợi tỷ số D của vị trí
    ControlTableItem *velocity_p_gain_item_;  // Thành phần bảng kiểm soát cho hệ số lợi tỷ số P của vận tốc
    ControlTableItem *velocity_i_gain_item_;  // Thành phần bảng kiểm soát cho hệ số lợi tỷ số I của vận tốc
    ControlTableItem *velocity_d_gain_item_;  // Thành phần bảng kiểm soát cho hệ số lợi tỷ số D của vận tốc

    // Constructor
    Dynamixel(int id, std::string model_name, float protocol_version);

    // Các hàm thành viên cho việc chuyển đổi giữa các đơn vị
    double convertValue2Radian(int32_t value);
    int32_t convertRadian2Value(double radian);

    double convertValue2Velocity(int32_t value);
    int32_t convertVelocity2Value(double velocity);

    double convertValue2Torque(int16_t value);
    int16_t convertTorque2Value(double torque);
  };

}

#endif /* ROBOTIS_DEVICE_DYNAMIXEL_H_ */

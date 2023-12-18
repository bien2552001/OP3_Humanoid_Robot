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
 * dynamixel_state.h
 *
 *  Created on: 2015. 12. 8.
 *      Author: zerom
 */

#ifndef ROBOTIS_DEVICE_DYNAMIXEL_STATE_H_
#define ROBOTIS_DEVICE_DYNAMIXEL_STATE_H_

#include <stdint.h>
#include <map>
#include "time_stamp.h"

#define INDIRECT_DATA_1 "indirect_data_1"
#define INDIRECT_ADDRESS_1 "indirect_address_1"

namespace robotis_framework
{

  class DynamixelState
  {
  public:
    TimeStamp update_time_stamp_; // Thời điểm cập nhật trạng thái

    double present_position_; // Vị trí hiện tại
    double present_velocity_; // Vận tốc hiện tại
    double present_torque_;   // Momen xoắn hiện tại
    double goal_position_;    // Vị trí mục tiêu
    double goal_velocity_;    // Vận tốc mục tiêu
    double goal_torque_;      // Momen xoắn mục tiêu
    int position_p_gain_;     // Hệ số tỷ lệ của vị trí
    int position_i_gain_;     // Hệ số tích phân của vị trí
    int position_d_gain_;     // Hệ số dao động của vị trí
    int velocity_p_gain_;     // Hệ số tỷ lệ của vận tốc
    int velocity_i_gain_;     // Hệ số tích phân của vận tốc
    int velocity_d_gain_;     // Hệ số dao động của vận tốc

    std::map<std::string, uint32_t> bulk_read_table_; // Bảng dữ liệu kiểu map sử dụng để đọc dữ liệu hàng loạt

    double position_offset_; // Độ lệch vị trí

    DynamixelState()
        : update_time_stamp_(0, 0), // Thời điểm cập nhật ban đầu là 0
          present_position_(0.0),   // Vị trí hiện tại của động cơ là 0.0
          present_velocity_(0.0),   // Vận tốc hiện tại của động cơ là 0.0
          present_torque_(0.0),     // Momen xoắn hiện tại của động cơ là 0.0
          goal_position_(0.0),      // Vị trí mục tiêu của động cơ là 0.0
          goal_velocity_(0.0),      // Vận tốc mục tiêu của động cơ là 0.0
          goal_torque_(0.0),        // Momen xoắn mục tiêu của động cơ là 0.0
          position_p_gain_(65535),  // Hệ số lợi suất P cho vị trí là 65535
          position_i_gain_(65535),  // Hệ số lợi suất I cho vị trí là 65535
          position_d_gain_(65535),  // Hệ số lợi suất D cho vị trí là 65535
          velocity_p_gain_(65535),  // Hệ số lợi suất P cho vận tốc là 65535
          velocity_i_gain_(65535),  // Hệ số lợi suất I cho vận tốc là 65535
          velocity_d_gain_(65535),  // Hệ số lợi suất D cho vận tốc là 65535
          position_offset_(0)       // Độ lệch vị trí là 0
    {
      bulk_read_table_.clear(); // Xóa bảng đọc hàng loạt ban đầu
    }
  };

}

#endif /* ROBOTIS_DEVICE_DYNAMIXEL_STATE_H_ */

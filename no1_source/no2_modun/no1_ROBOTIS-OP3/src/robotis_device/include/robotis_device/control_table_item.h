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
 * control_table_item.h
 *
 *  Created on: 2015. 12. 16.
 *      Author: zerom
 */

#ifndef ROBOTIS_DEVICE_CONTROL_TABLE_ITEM_H_
#define ROBOTIS_DEVICE_CONTROL_TABLE_ITEM_H_

#include <stdint.h>
#include <string>

namespace robotis_framework
{

  // Định nghĩa kiểu liệt kê cho quyền truy cập
  enum AccessType
  {
    Read,
    ReadWrite
  };

  // Định nghĩa kiểu liệt kê cho loại bộ nhớ
  enum MemoryType
  {
    EEPROM,
    RAM
  };

  // Lớp ControlTableItem đại diện cho một mục trong bảng điều khiển
  class ControlTableItem
  {
  public:
    std::string item_name_;  // Tên của mục
    uint16_t address_;       // Địa chỉ của mục trong bảng điều khiển
    AccessType access_type_; // Quyền truy cập của mục (Read hoặc ReadWrite)
    MemoryType memory_type_; // Loại bộ nhớ của mục (EEPROM hoặc RAM)
    uint8_t data_length_;    // Độ dài dữ liệu của mục
    int32_t data_min_value_; // Giá trị tối thiểu của dữ liệu
    int32_t data_max_value_; // Giá trị tối đa của dữ liệu
    bool is_signed_;         // Chỉ định xem dữ liệu có phải là số nguyên có dấu hay không

    // Constructor mặc định
    ControlTableItem()
        : item_name_(""),     // Thiết lập giá trị mặc định cho thành viên item_name_
          address_(0),        // Thiết lập giá trị mặc định cho thành viên address_
          access_type_(Read), // Thiết lập giá trị mặc định cho thành viên access_type_
          memory_type_(RAM),  // Thiết lập giá trị mặc định cho thành viên memory_type_
          data_length_(0),    // Thiết lập giá trị mặc định cho thành viên data_length_
          data_min_value_(0), // Thiết lập giá trị mặc định cho thành viên data_min_value_
          data_max_value_(0), // Thiết lập giá trị mặc định cho thành viên data_max_value_
          is_signed_(false)   // Thiết lập giá trị mặc định cho thành viên is_signed_
    {
    }
  };

}

#endif /* ROBOTIS_DEVICE_CONTROL_TABLE_ITEM_H_ */

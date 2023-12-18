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
 * device.h
 *
 *  Created on: 2016. 5. 12.
 *      Author: zerom
 */

#ifndef ROBOTIS_DEVICE_DEVICE_H_
#define ROBOTIS_DEVICE_DEVICE_H_

#include <map>
#include <string>
#include <vector>

#include "control_table_item.h"

namespace robotis_framework
{

  class Device
  {
  public:
    uint8_t id_;             // ID của thiết bị
    float protocol_version_; // Phiên bản giao thức
    std::string model_name_; // Tên mô hình của thiết bị
    std::string port_name_;  // Tên cổng kết nối

    std::map<std::string, ControlTableItem *> ctrl_table_; // Bảng điều khiển của thiết bị
    std::vector<ControlTableItem *> bulk_read_items_;      // Danh sách các mục để đọc dữ liệu hàng loạt

    // Hủy bỏ ảo cho việc kế thừa
    virtual ~Device() {}
  };

}

#endif /* ROBOTIS_DEVICE_DEVICE_H_ */

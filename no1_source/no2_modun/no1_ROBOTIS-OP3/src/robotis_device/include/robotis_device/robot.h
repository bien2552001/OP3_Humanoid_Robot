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
 * robot.h
 *
 *  Created on: 2015. 12. 11.
 *      Author: zerom
 */

#ifndef ROBOTIS_DEVICE_ROBOT_H_
#define ROBOTIS_DEVICE_ROBOT_H_

#include <vector>

#include "sensor.h"
#include "dynamixel.h"
#include "dynamixel_sdk/port_handler.h"

#define DYNAMIXEL "dynamixel"
#define SENSOR "sensor"

#define SESSION_CONTROL_INFO "control info"
#define SESSION_PORT_INFO "port info"
#define SESSION_DEVICE_INFO "device info"

#define SESSION_TYPE_INFO "type info"
#define SESSION_CONTROL_TABLE "control table"

#define DEFAULT_CONTROL_CYCLE 8 // milliseconds

namespace robotis_framework
{

  class Robot
  {
  private:
    int control_cycle_msec_; // Chu kỳ kiểm soát của robot trong mili giây

  public:
    std::map<std::string, dynamixel::PortHandler *> ports_;  // Danh sách các cổng kết nối, với tên cổng là chuỗi
    std::map<std::string, std::string> port_default_device_; // Danh sách tên thiết bị mặc định cho từng cổng, với tên cổng là chuỗi

    std::map<std::string, Dynamixel *> dxls_; // Danh sách các động cơ Dynamixel, với tên động cơ là chuỗi
    std::map<std::string, Sensor *> sensors_; // Danh sách các cảm biến, với tên cảm biến là chuỗi

    // Constructor
    Robot(std::string robot_file_path, std::string dev_desc_dir_path);

    // Phương thức trả về con trỏ đến đối tượng Sensor dựa trên các tham số đầu vào
    Sensor *getSensor(std::string path, int id, std::string port, float protocol_version);

    // Phương thức trả về con trỏ đến đối tượng Dynamixel dựa trên các tham số đầu vào
    Dynamixel *getDynamixel(std::string path, int id, std::string port, float protocol_version);

    // Phương thức trả về chu kỳ kiểm soát của robot
    int getControlCycle();
  };

}

#endif /* ROBOTIS_DEVICE_ROBOT_H_ */

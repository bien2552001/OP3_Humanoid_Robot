/*******************************************************************************
 * Copyright 2017 ROBOTIS CO., LTD.
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

/* Author: SCH */

#ifndef OP3_ONLINE_WALKING_MODULE_WHOLEBODY_CONTROL_
#define OP3_ONLINE_WALKING_MODULE_WHOLEBODY_CONTROL_

#pragma once

#include <math.h>
#include <stdint.h>
#include <string>
#include <vector>
#include <map>
#include <geometry_msgs/Pose.h>
#include <eigen3/Eigen/Eigen>
#include "robotis_math/robotis_math.h"

// Lớp WholebodyControl quản lý kiểm soát toàn bộ cơ thể
class WholebodyControl
{
public:
  // Constructor của lớp WholebodyControl, nhận tham số như nhóm kiểm soát, thời điểm khởi tạo và kết thúc,
  // cũng như thông tin về mục tiêu của robot
  WholebodyControl(std::string control_group,
                   double init_time, double fin_time,
                   geometry_msgs::Pose goal_msg);

  // Destructor của lớp WholebodyControl
  virtual ~WholebodyControl();

  // Hàm khởi tạo, nhận thông tin về vị trí, hướng và pose của cơ thể và chân khi bắt đầu
  void initialize(std::vector<double_t> init_body_pos, std::vector<double_t> init_body_rot,
                  std::vector<double_t> init_r_foot_pos, std::vector<double_t> init_r_foot_Q,
                  std::vector<double_t> init_l_foot_pos, std::vector<double_t> init_l_foot_Q);

  // Hàm cập nhật kiểm soát toàn bộ cơ thể
  void update();

  // Hàm kết thúc kiểm soát toàn bộ cơ thể
  void finalize();

  // Hàm đặt thời điểm cần kiểm soát
  void set(double time);

  // Hàm trả về vị trí các khớp (joint) tại một thời điểm cụ thể
  std::vector<double_t> getJointPosition(double time);

  // Hàm trả về vận tốc các khớp (joint) tại một thời điểm cụ thể
  std::vector<double_t> getJointVelocity(double time);

  // Hàm trả về gia tốc các khớp (joint) tại một thời điểm cụ thể
  std::vector<double_t> getJointAcceleration(double time);

  // Hàm trả về vị trí của các nhiệm vụ (task) như chân trái, chân phải và cơ thể tại một thời điểm cụ thể
  void getTaskPosition(std::vector<double_t> &l_foot_pos,
                       std::vector<double_t> &r_foot_pos,
                       std::vector<double_t> &body_pos);

  // Hàm trả về vận tốc của các nhiệm vụ (task) tại một thời điểm cụ thể
  std::vector<double_t> getTaskVelocity(double time);

  // Hàm trả về gia tốc của các nhiệm vụ (task) tại một thời điểm cụ thể
  std::vector<double_t> getTaskAcceleration(double time);

  // Hàm trả về hướng của các nhiệm vụ (task) như chân trái, chân phải và cơ thể
  void getTaskOrientation(std::vector<double_t> &l_foot_Q,
                          std::vector<double_t> &r_foot_Q,
                          std::vector<double_t> &body_Q);

  // Hàm lấy thông tin vị trí của một nhóm trong robot
  void getGroupPose(std::string name, geometry_msgs::Pose *msg);

private:
  // Đối tượng MinimumJerk để tính toán quỹ đạo nhiệm vụ
  robotis_framework::MinimumJerk *task_trajectory_;

  // Tên của nhóm kiểm soát
  std::string control_group_;

  // Liên kết cuối cùng của nhóm kiểm soát
  int end_link_;

  // Thời điểm khởi tạo và thời điểm kết thúc của quá trình kiểm soát toàn bộ cơ thể
  double init_time_, fin_time_;

  // Thông tin về mục tiêu của robot
  geometry_msgs::Pose goal_msg_;

  // Thông tin về vị trí, vận tốc và gia tốc khởi tạo của cơ thể
  std::vector<double_t> init_body_pos_, init_body_vel_, init_body_accel_;

  // Thông tin về vị trí, vận tốc và gia tốc mong muốn của cơ thể
  std::vector<double_t> des_body_pos_, des_body_vel_, des_body_accel_;

  // Thông tin về vị trí, vận tốc và gia tốc mục tiêu của cơ thể
  std::vector<double_t> goal_body_pos_, goal_body_vel_, goal_body_accel_;

  // Đối tượng Quaternion đại diện cho hướng của cơ thể
  Eigen::Quaterniond init_body_Q_, des_body_Q_, goal_body_Q_;

  // Thông tin về vị trí, vận tốc và gia tốc khởi tạo của chân trái
  std::vector<double_t> init_l_foot_pos_, init_l_foot_vel_, init_l_foot_accel_;

  // Thông tin về vị trí, vận tốc và gia tốc mong muốn của chân trái
  std::vector<double_t> des_l_foot_pos_, des_l_foot_vel_, des_l_foot_accel_;

  // Thông tin về vị trí, vận tốc và gia tốc mục tiêu của chân trái
  std::vector<double_t> goal_l_foot_pos_, goal_l_foot_vel_, goal_l_foot_accel_;

  // Đối tượng Quaternion đại diện cho hướng của chân trái
  Eigen::Quaterniond init_l_foot_Q_, des_l_foot_Q_, goal_l_foot_Q_;

  // Thông tin về vị trí, vận tốc và gia tốc khởi tạo của chân phải
  std::vector<double_t> init_r_foot_pos_, init_r_foot_vel_, init_r_foot_accel_;

  // Thông tin về vị trí, vận tốc và gia tốc mong muốn của chân phải
  std::vector<double_t> des_r_foot_pos_, des_r_foot_vel_, des_r_foot_accel_;

  // Thông tin về vị trí, vận tốc và gia tốc mục tiêu của chân phải
  std::vector<double_t> goal_r_foot_pos_, goal_r_foot_vel_, goal_r_foot_accel_;

  // Đối tượng Quaternion đại diện cho hướng của chân phải
  Eigen::Quaterniond init_r_foot_Q_, des_r_foot_Q_, goal_r_foot_Q_;

  // Thông tin về vị trí, vận tốc và gia tốc mục tiêu của nhiệm vụ (task)
  std::vector<double_t> goal_task_pos_, goal_task_vel_, goal_task_accel_;

  // Đối tượng Quaternion đại diện cho hướng của nhiệm vụ (task)
  Eigen::Quaterniond init_task_Q_, des_task_Q_, goal_task_Q_;
};

#endif

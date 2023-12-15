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

#include "op3_base_module/base_module_state.h"

namespace robotis_op
{

// Constructor của lớp BaseModuleState
BaseModuleState::BaseModuleState()
{
  is_moving_ = false;  // Khởi tạo biến kiểm tra di chuyển là false

  cnt_ = 0;  // Khởi tạo số bước đếm là 0

  mov_time_ = 1.0;      // Thời gian di chuyển mặc định là 1.0 giây
  smp_time_ = 0.008;    // Thời gian lấy mẫu mặc định là 0.008 giây
  all_time_steps_ = int(mov_time_ / smp_time_) + 1;  // Tính tổng số bước thời gian

  // Khởi tạo ma trận quỹ đạo khớp tính toán với kích thước và giá trị mặc định
  calc_joint_tra_ = Eigen::MatrixXd::Zero(all_time_steps_, MAX_JOINT_ID + 1);

  // Khởi tạo vị trí khởi tạo và vị trí hiện tại của các khớp với giá trị mặc định
  joint_ini_pose_ = Eigen::MatrixXd::Zero(MAX_JOINT_ID + 1, 1);
  joint_pose_ = Eigen::MatrixXd::Zero(MAX_JOINT_ID + 1, 1);

  via_num_ = 1;  // Số điểm thông qua mặc định là 1

  // Khởi tạo ma trận thông qua vị trí, vận tốc, và gia tốc của các khớp với giá trị mặc định
  joint_via_pose_ = Eigen::MatrixXd::Zero(via_num_, MAX_JOINT_ID + 1);
  joint_via_dpose_ = Eigen::MatrixXd::Zero(via_num_, MAX_JOINT_ID + 1);
  joint_via_ddpose_ = Eigen::MatrixXd::Zero(via_num_, MAX_JOINT_ID + 1);

  // Khởi tạo ma trận thời gian của điểm thông qua với giá trị mặc định
  via_time_ = Eigen::MatrixXd::Zero(via_num_, 1);
}

// Destructor của lớp BaseModuleState
BaseModuleState::~BaseModuleState()
{
  // Không có công việc đặc biệt nào cần thực hiện trong destructor
}


}

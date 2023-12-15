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

#ifndef BASE_MODULE_ROBOTISSTATE_H_
#define BASE_MODULE_ROBOTISSTATE_H_

#include <eigen3/Eigen/Eigen>

#include "robotis_math/robotis_math.h"
#include "op3_kinematics_dynamics/op3_kinematics_dynamics.h"

namespace robotis_op
{

  // Lớp chứa trạng thái của module cơ bản
  class BaseModuleState
  {
  public:
    BaseModuleState();  // Constructor
    ~BaseModuleState(); // Destructor

    bool is_moving_; // Biến kiểm tra xem có đang chuyển động hay không

    int cnt_; // Số lượng bước đếm (counter number)

    double mov_time_; // Thời gian di chuyển (movement time)
    double smp_time_; // Thời gian lấy mẫu (sampling time)

    int all_time_steps_; // Tổng số bước thời gian của quá trình di chuyển

    Eigen::MatrixXd calc_joint_tra_; // Quỹ đạo khớp tính toán

    Eigen::MatrixXd joint_ini_pose_; // Vị trí khởi tạo của các khớp
    Eigen::MatrixXd joint_pose_;     // Vị trí hiện tại của các khớp

    int via_num_; // Số điểm thông qua (waypoints)

    Eigen::MatrixXd joint_via_pose_;   // Vị trí thông qua của các khớp
    Eigen::MatrixXd joint_via_dpose_;  // Vận tốc thông qua của các khớp
    Eigen::MatrixXd joint_via_ddpose_; // Gia tốc thông qua của các khớp

    Eigen::MatrixXd via_time_; // Thời gian của điểm thông qua
  };

}

#endif /* BASE_MODULE_ROBOTISSTATE_H_ */

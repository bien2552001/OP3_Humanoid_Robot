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

#ifndef OP3_ONLINE_WALKING_MODULE_WALKING_CONTROL_
#define OP3_ONLINE_WALKING_MODULE_WALKING_CONTROL_

#pragma once

#include <math.h>
#include <stdint.h>
#include <string>
#include <vector>
#include <map>
#include <geometry_msgs/Pose2D.h>
#include <eigen3/Eigen/Eigen>
#include "op3_online_walking_module_msgs/FootStepCommand.h"
#include "op3_online_walking_module_msgs/FootStepArray.h"
#include "op3_online_walking_module_msgs/PreviewResponse.h"
#include "op3_online_walking_module_msgs/Step2D.h"
#include "op3_online_walking_module_msgs/Step2DArray.h"
//#include "op3_kinematics_dynamics/op3_kinematics_dynamics.h"
#include "robotis_math/robotis_math.h"

// Liệt kê WALKING_LEG định nghĩa các giá trị cho việc đại diện cho chân đi
enum WALKING_LEG {
  LEFT_LEG = 0,   // Chân trái
  RIGHT_LEG = 1,  // Chân phải
  LEG_COUNT = 2   // Tổng số chân
};

// Liệt kê WALKING_PHASE định nghĩa các giá trị cho việc đại diện cho các giai đoạn của quá trình đi bộ
enum WALKING_PHASE {
  DSP = 0,        // Giai đoạn Double Support (hỗ trợ đôi)
  SSP = 1,        // Giai đoạn Single Support (hỗ trợ đơn)
  PHASE_COUNT = 2 // Tổng số giai đoạn
};


// Định nghĩa lớp WalkingControl
class WalkingControl
{
public:
  // Constructor của lớp WalkingControl, nhận các tham số như control_cycle, dsp_ratio, lipm_height, foot_height_max, zmp_offset_x, zmp_offset_y,
  // x_lipm, y_lipm là các vector chứa các giá trị tương ứng, và foot_distance
  WalkingControl(double control_cycle,
                 double dsp_ratio, double lipm_height, double foot_height_max, double zmp_offset_x, double zmp_offset_y,
                 std::vector<double_t> x_lipm, std::vector<double_t> y_lipm,
                 double foot_distance);
                 
  // Destructor của lớp WalkingControl
  virtual ~WalkingControl();


// Hàm khởi tạo với thông tin bước chân 3D
void initialize(op3_online_walking_module_msgs::FootStepCommand foot_step_command,
                std::vector<double_t> init_body_pos, std::vector<double_t> init_body_Q,
                std::vector<double_t> init_r_foot_pos, std::vector<double_t> init_r_foot_Q,
                std::vector<double_t> init_l_foot_pos, std::vector<double_t> init_l_foot_Q);
                
// Hàm khởi tạo với mảng thông tin bước chân 2D
void initialize(op3_online_walking_module_msgs::Step2DArray foot_step_2d,
                std::vector<double_t> init_body_pos, std::vector<double_t> init_body_Q,
                std::vector<double_t> init_r_foot_pos, std::vector<double_t> init_r_foot_Q,
                std::vector<double_t> init_l_foot_pos, std::vector<double_t> init_l_foot_Q);


// Chuyển đến bước tiếp theo trong quá trình đi bộ
void next();

// Hoàn tất quá trình đi bộ
void finalize();

// Đặt giá trị thời gian, bước và kiểu bước chân (2D hoặc 3D)
void set(double time, int step, bool foot_step_2d);

// Lấy chiều cao của Lipm
double getLipmHeight();

// Tính toán tham số cho bước chân
void calcFootStepParam();

// Chuyển đổi thông tin bước chân từ 2D sang 3D
void transformFootStep2D();

// Tính toán quỹ đạo chân cho bước chân cụ thể
void calcFootTrajectory(int step);

// Tính toán vị trí chân cho bước chân cụ thể tại một thời điểm nhất định
void calcFootStepPose(double time, int step);

// Tính toán điểm ZMP tham chiếu cho bước chân cụ thể
void calcRefZMP(int step);

// Tính toán tham số cho kiểm soát dự đoán
void calcPreviewParam(std::vector<double_t> K, int K_row, int K_col,
                      std::vector<double_t> P, int P_row, int P_col);

// Tính toán kiểm soát dự đoán tại một thời điểm nhất định cho bước chân cụ thể
void calcPreviewControl(double time, int step);

// Tính toán vị trí mục tiêu cho chân
void calcGoalFootPose();

// Tính toán điểm ZMP tham chiếu theo trục x cho bước chân cụ thể
double calcRefZMPx(int step);

// Tính toán điểm ZMP tham chiếu theo trục y cho bước chân cụ thể
double calcRefZMPy(int step);

// Lấy vị trí đi bộ, bao gồm vị trí của chân trái, chân phải và cơ bắp
void getWalkingPosition(std::vector<double_t> &l_foot_pos,
                        std::vector<double_t> &r_foot_pos,
                        std::vector<double_t> &body_pos);

// Lấy vận tốc đi bộ, bao gồm vận tốc của chân trái, chân phải và cơ bắp
void getWalkingVelocity(std::vector<double_t> &l_foot_vel,
                        std::vector<double_t> &r_foot_vel,
                        std::vector<double_t> &body_vel);

// Lấy gia tốc đi bộ, bao gồm gia tốc của chân trái, chân phải và cơ bắp
void getWalkingAcceleration(std::vector<double_t> &l_foot_accel,
                            std::vector<double_t> &r_foot_accel,
                            std::vector<double_t> &body_accel);

// Lấy hướng của chân trái, chân phải và cơ bắp khi đi bộ
void getWalkingOrientation(std::vector<double_t> &l_foot_Q,
                           std::vector<double_t> &r_foot_Q,
                           std::vector<double_t> &body_Q);

// Lấy thông tin về LIPM (Linear Inverted Pendulum Model)
void getLIPM(std::vector<double_t> &x_lipm, std::vector<double_t> &y_lipm);

// Lấy trạng thái của quá trình đi bộ, bao gồm chân đi và giai đoạn đi bộ
void getWalkingState(int &walking_leg, int &walking_phase);


protected:
  // Tham chiếu đến đối tượng KinematicsDynamics của robot (đã được bình luận)
  // thormang3::KinematicsDynamics *robot_;

  // Đối tượng MinimumJerk để tính toán quỹ đạo cơ thể
  robotis_framework::MinimumJerk *body_trajectory_;

  // Đối tượng MinimumJerkViaPoint để tính toán quỹ đạo chân phải
  robotis_framework::MinimumJerkViaPoint *r_foot_tra_;

  // Đối tượng MinimumJerkViaPoint để tính toán quỹ đạo chân trái
  robotis_framework::MinimumJerkViaPoint *l_foot_tra_;

  // Đối tượng PreviewControl để thực hiện kiểm soát dự đoán
  robotis_framework::PreviewControl *preview_control_;

  // Thời điểm khởi tạo và thời điểm kết thúc của quá trình đi bộ
  double init_time_, fin_time_;

  // Chu kỳ kiểm soát (thời gian giữa các lần kiểm soát liên tiếp)
  double control_cycle_;

  // Chân đang đi (LEFT_LEG hoặc RIGHT_LEG)
  int walking_leg_;

  // Giai đoạn của quá trình đi bộ (DSP hoặc SSP)
  int walking_phase_;


  // Foot Trajectory
// Kích thước của chân robot theo hướng x và y
double foot_size_x_;
double foot_size_y_;

// Dịch chuyển gốc chân robot theo hướng x và y
double foot_origin_shift_x_;
double foot_origin_shift_y_;

// Tỷ lệ thời gian hỗ trợ đôi (DSP) trong chu kỳ đi bộ
double dsp_ratio_;

// Chiều cao tối đa của quỹ đạo chân
double foot_tra_max_z_;

// Số bước chân dự kiến trong quá trình đi bộ
int foot_step_size_;

// Cấu trúc chứa thông tin về bước chân dự kiến (FootStepCommand)
op3_online_walking_module_msgs::FootStepCommand foot_step_command_;

// Cấu trúc chứa thông tin về các tham số của bước chân (FootStepArray)
op3_online_walking_module_msgs::FootStepArray foot_step_param_;

// Cấu trúc chứa thông tin phản hồi từ kiểm soát dự đoán (PreviewResponse)
op3_online_walking_module_msgs::PreviewResponse preview_response_;

// Mảng chứa thông tin về bước chân 2D
op3_online_walking_module_msgs::Step2DArray foot_step_2d_;

  // Preview Control
// Số lượng bước xem trước trong kiểm soát dự đoán
int preview_size_;

// Thời gian xem trước trong kiểm soát dự đoán
double preview_time_;

// Chiều cao của mô hình Inverted Pendulum tuyến tính (LIPM)
double lipm_height_;

// Tổng của các giá trị ZMP theo trục x và y
double sum_of_zmp_x_, sum_of_zmp_y_;

// Tổng của các giá trị cx và cy (tham số dự đoán)
double sum_of_cx_, sum_of_cy_;

// Ma trận A, b, c trong kiểm soát dự đoán
Eigen::MatrixXd A_, b_, c_;

// Ma trận k_x cho kiểm soát dự đoán
Eigen::MatrixXd k_x_;

// Hệ số k_s trong kiểm soát dự đoán
double k_s_;

// Ma trận f trong kiểm soát dự đoán
Eigen::MatrixXd f_;

// Ma trận u_x và u_y trong kiểm soát dự đoán
Eigen::MatrixXd u_x_, u_y_;

// Ma trận x_lipm và y_lipm lưu giữ thông tin về mô hình Inverted Pendulum tuyến tính
Eigen::MatrixXd x_lipm_, y_lipm_;

// Ma trận K và P trong kiểm soát dự đoán
Eigen::MatrixXd K_, P_;

// Giá trị tham chiếu ZMP theo trục x và y
double ref_zmp_x_, ref_zmp_y_;

// Tổng của giá trị ZMP theo trục x và y dự kiến trong kiểm soát dự đoán
double preview_sum_zmp_x_, preview_sum_zmp_y_;

// Offset ZMP theo trục x và y
double zmp_offset_x_, zmp_offset_y_;

// Bộ đệm chứa thông tin về vị trí mục tiêu của chân phải và chân trái
Eigen::MatrixXd goal_r_foot_pos_buffer_, goal_l_foot_pos_buffer_;

// Bộ đệm chứa thông tin về giá trị tham chiếu ZMP
Eigen::MatrixXd ref_zmp_buffer_;


  // Pose Information
// Góc quay khởi tạo của cơ thể (yaw)
double init_body_yaw_angle_;

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



};

#endif

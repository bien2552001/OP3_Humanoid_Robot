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

/* Author: SCH */

#include "op3_balance_control/op3_balance_control.h"

#include <iostream>

using namespace robotis_op;

// Constructor mặc định của lớp DampingController
// Khởi tạo các thuộc tính với giá trị mặc định
DampingController::DampingController()
{
  // Khởi tạo thuộc tính 'desired_' với giá trị 0.0
  desired_ = 0.0;

  // Khởi tạo thuộc tính 'gain_' với giá trị 0.0
  gain_ = 0.0;

  // Khởi tạo thuộc tính 'time_constant_sec_' với giá trị 1.0
  time_constant_sec_ = 1.0;

  // Khởi tạo thuộc tính 'output_' với giá trị 0.0
  output_ = 0.0;

  // Khởi tạo thuộc tính 'control_cycle_sec_' với giá trị 0.008
  control_cycle_sec_ = 0.008;

  // Khởi tạo thuộc tính 'previous_result_' với giá trị 0.0
  previous_result_ = 0.0;
}


// Constructor với tham số của lớp DampingController
// Thiết lập giá trị mong muốn, hệ số lợi tức, hằng số thời gian, và chu kỳ kiểm soát
// Dựa trên tham số truyền vào (time_unit_sec)
DampingController::DampingController(double time_unit_sec)
{
  // Thiết lập giá trị mong muốn ban đầu là 0.0
  desired_ = 0.0;

  // Thiết lập hệ số lợi tức (gain) ban đầu là 0.0
  gain_ = 0.0;

  // Thiết lập hằng số thời gian (time constant) ban đầu là 1.0
  time_constant_sec_ = 1.0;

  // Thiết lập đầu ra (output) ban đầu là 0.0
  output_ = 0.0;

  // Thiết lập chu kỳ kiểm soát (control cycle) bằng giá trị được truyền vào (time_unit_sec)
  control_cycle_sec_ = time_unit_sec;

  // Thiết lập kết quả trước đó ban đầu là 0.0
  previous_result_ = 0.0;
}



DampingController::~DampingController()
{
  // Destructor mặc định không có công việc cụ thể trong trường hợp này.
}



// Phương thức tính toán đầu ra của bộ điều khiển dựa trên giá trị đầu vào hiện tại
// và sử dụng một bộ lọc thấp để giảm nhiễu
// Trả về giá trị đầu ra của bộ điều khiển
double DampingController::getDampingControllerOutput(double present_sensor_output)
{
  // Tính toán tần số cắt của bộ lọc thấp
  double cut_off_freq = 1.0 / time_constant_sec_;

  // Khởi tạo hệ số alpha ban đầu
  double alpha = 1.0;

  // Tính toán hệ số alpha dựa trên tần số cắt và chu kỳ kiểm soát
  alpha = (2.0 * M_PI * cut_off_freq * control_cycle_sec_) / (1.0 + 2.0 * M_PI * cut_off_freq * control_cycle_sec_);

  // Áp dụng công thức của bộ lọc thấp (Low Pass Filter) để tính toán kết quả mới
  previous_result_ = alpha * (desired_ - present_sensor_output) + (1.0 - alpha) * previous_result_;

  // Áp dụng hệ số lợi tức để có giá trị đầu ra của bộ điều khiển
  output_ = gain_ * previous_result_;

  // Trả về giá trị đầu ra của bộ điều khiển
  return output_;
}


// Constructor mặc định của BalanceLowPassFilter
BalanceLowPassFilter::BalanceLowPassFilter()
{
  // Thiết lập tần số cắt mặc định là 1.0 Hz
  cut_off_freq_ = 1.0;

  // Thiết lập chu kỳ kiểm soát mặc định là 0.008 giây
  control_cycle_sec_ = 0.008;

  // Thiết lập giá trị đầu ra trước đó mặc định là 0
  prev_output_ = 0;

  // Tính toán và thiết lập giá trị alpha dựa trên tần số cắt và chu kỳ kiểm soát
  alpha_ = (2.0 * M_PI * cut_off_freq_ * control_cycle_sec_) / (1.0 + 2.0 * M_PI * cut_off_freq_ * control_cycle_sec_);
}

// Constructor với tham số của BalanceLowPassFilter
BalanceLowPassFilter::BalanceLowPassFilter(double control_cycle_sec, double cut_off_frequency)
{
  // Thiết lập tần số cắt từ tham số đầu vào
  cut_off_freq_ = cut_off_frequency;

  // Thiết lập chu kỳ kiểm soát từ tham số đầu vào
  control_cycle_sec_ = control_cycle_sec;

  // Thiết lập giá trị đầu ra trước đó mặc định là 0
  prev_output_ = 0;

  // Tính toán và thiết lập giá trị alpha dựa trên tần số cắt và chu kỳ kiểm soát
  if (cut_off_frequency > 0)
    alpha_ = (2.0 * M_PI * cut_off_freq_ * control_cycle_sec_) / (1.0 + 2.0 * M_PI * cut_off_freq_ * control_cycle_sec_);
  else
    alpha_ = 1;  // Trường hợp tần số cắt không hợp lệ, đặt alpha là 1 để tránh chia cho 0
}

// Destructor của BalanceLowPassFilter
BalanceLowPassFilter::~BalanceLowPassFilter()
{
  // Destructor mặc định không có công việc cụ thể trong trường hợp này.
}

// Phương thức initialize để cập nhật thông số của bộ lọc thấp
void BalanceLowPassFilter::initialize(double control_cycle_sec, double cut_off_frequency)
{
  // Thiết lập tần số cắt từ tham số đầu vào
  cut_off_freq_ = cut_off_frequency;

  // Thiết lập chu kỳ kiểm soát từ tham số đầu vào
  control_cycle_sec_ = control_cycle_sec;

  // Thiết lập giá trị đầu ra trước đó mặc định là 0
  prev_output_ = 0;

  // Tính toán và thiết lập giá trị alpha dựa trên tần số cắt và chu kỳ kiểm soát
  if (cut_off_frequency > 0)
    alpha_ = (2.0 * M_PI * cut_off_freq_ * control_cycle_sec_) / (1.0 + 2.0 * M_PI * cut_off_freq_ * control_cycle_sec_);
  else
    alpha_ = 1;  // Trường hợp tần số cắt không hợp lệ, đặt alpha là 1 để tránh chia cho 0
}

// Phương thức setCutOffFrequency để cập nhật tần số cắt của bộ lọc thấp
void BalanceLowPassFilter::setCutOffFrequency(double cut_off_frequency)
{
  // Thiết lập tần số cắt từ tham số đầu vào
  cut_off_freq_ = cut_off_frequency;

  // Tính toán và thiết lập giá trị alpha dựa trên tần số cắt và chu kỳ kiểm soát
  if (cut_off_frequency > 0)
    alpha_ = (2.0 * M_PI * cut_off_freq_ * control_cycle_sec_) / (1.0 + 2.0 * M_PI * cut_off_freq_ * control_cycle_sec_);
  else
    alpha_ = 1;  // Trường hợp tần số cắt không hợp lệ, đặt alpha là 1 để tránh chia cho 0
}



// Trả về giá trị tần số cắt của bộ lọc thấp
double BalanceLowPassFilter::getCutOffFrequency(void)
{
  return cut_off_freq_;
}

// Lấy giá trị đầu ra đã lọc của bộ lọc thấp cho giá trị đầu vào hiện tại
double BalanceLowPassFilter::getFilteredOutput(double present_raw_value)
{
  // Áp dụng công thức bộ lọc thấp để tính toán giá trị đầu ra mới
  prev_output_ = alpha_ * present_raw_value + (1.0 - alpha_) * prev_output_;

  // Trả về giá trị đã lọc
  return prev_output_;
}



// Constructor mặc định của lớp BalancePDController
BalancePDController::BalancePDController()
{
  // Khởi tạo giá trị mong muốn (desired_) là 0.
  desired_ = 0;

  // Khởi tạo hệ số lợi tức P (p_gain_) là 0.
  p_gain_ = 0;

  // Khởi tạo hệ số lợi tức D (d_gain_) là 0.
  d_gain_ = 0;

  // Khởi tạo giá trị sai số hiện tại (curr_err_) là 0.
  curr_err_ = 0;

  // Khởi tạo giá trị sai số trước đó (prev_err_) là 0.
  prev_err_ = 0;
}


// Destructor của lớp BalancePDController
BalancePDController::~BalancePDController()
{
  // Destructor mặc định không có công việc cụ thể trong trường hợp này.
}

// Phương thức tính toán giá trị phản hồi của bộ điều khiển PD
double BalancePDController::getFeedBack(double present_sensor_output)
{
  // Lưu giá trị sai số trước đó (prev_err_)
  prev_err_ = curr_err_;

  // Tính toán giá trị sai số hiện tại (curr_err_)
  curr_err_ = desired_ - present_sensor_output;

  // Áp dụng công thức của bộ điều khiển PD để tính toán giá trị phản hồi mới
  double feedback = p_gain_ * curr_err_ + d_gain_ * (curr_err_ - prev_err_);

  // Trả về giá trị phản hồi của bộ điều khiển
  return feedback;
}



// Constructor của lớp BalanceControlUsingDampingConroller
BalanceControlUsingDampingConroller::BalanceControlUsingDampingConroller()
{
  // Thiết lập mã lỗi kiểm soát ban đầu là không có lỗi (NoError)
  balance_control_error_ = BalanceControlError::NoError;

  // Thiết lập chu kỳ kiểm soát là 0.008 giây
  control_cycle_sec_ = 0.008;

  // Kích hoạt cân bằng theo giảm tốc (gyro_enable_), cân bằng theo hướng (orientation_enable_), và cân bằng theo lực và mô-men lực (ft_enable_)
  gyro_enable_ = 1.0;
  orientation_enable_ = 1.0;
  ft_enable_ = 1.0;

  // Thiết lập ma trận biến đổi mong muốn cho robot_to_cob, robot_to_right_foot, và robot_to_left_foot là ma trận đơn vị
  desired_robot_to_cob_ = Eigen::MatrixXd::Identity(4, 4);
  desired_robot_to_right_foot_ = Eigen::MatrixXd::Identity(4, 4);
  desired_robot_to_left_foot_ = Eigen::MatrixXd::Identity(4, 4);

  // Thiết lập tần số cắt và hệ số alpha cho bộ lọc thấp gyro
  gyro_cut_off_freq_ = 10.0;
  gyro_lpf_alpha_ = 2.0 * M_PI * gyro_cut_off_freq_ * control_cycle_sec_ / (1.0 + 2.0 * M_PI * gyro_cut_off_freq_ * control_cycle_sec_);
  gyro_roll_filtered_ = gyro_pitch_filtered_ = 0;
  desired_gyro_roll_ = desired_gyro_pitch_ = 0;

  // Các giá trị cảm biến hiện tại (gyro, orientation, forces, và torques) được thiết lập ban đầu là 0
  current_gyro_roll_rad_per_sec_ = current_gyro_pitch_rad_per_sec_ = 0;
  current_orientation_roll_rad_ = current_orientation_pitch_rad_ = 0;
  current_right_fx_N_ = current_right_fy_N_ = current_right_fz_N_ = 0;
  current_right_tx_Nm_ = current_right_ty_Nm_ = current_right_tz_Nm_ = 0;
  current_left_fx_N_ = current_left_fy_N_ = current_left_fz_N_ = 0;
  current_left_tx_Nm_ = current_left_ty_Nm_ = current_left_tz_Nm_ = 0;

  // Kết quả cân bằng theo giảm tốc và theo hướng được thiết lập ban đầu là 0
  foot_roll_adjustment_by_gyro_roll_ = 0;
  foot_pitch_adjustment_by_gyro_pitch_ = 0;
  foot_roll_adjustment_by_orientation_roll_ = 0;
  foot_pitch_adjustment_by_orientation_pitch_ = 0;

  // Các điều chỉnh theo lực và mô-men lực được thiết lập ban đầu là 0
  foot_z_adjustment_by_force_z_difference_ = 0;
  r_foot_z_adjustment_by_force_z_ = 0;
  l_foot_z_adjustment_by_force_z_ = 0;
  r_foot_x_adjustment_by_force_x_ = 0;
  r_foot_y_adjustment_by_force_y_ = 0;
  r_foot_roll_adjustment_by_torque_roll_ = 0;
  r_foot_pitch_adjustment_by_torque_pitch_ = 0;
  l_foot_x_adjustment_by_force_x_ = 0;
  l_foot_y_adjustment_by_force_y_ = 0;
  l_foot_roll_adjustment_by_torque_roll_ = 0;
  l_foot_pitch_adjustment_by_torque_pitch_ = 0;

  // Điều chỉnh thủ công cho cob được thiết lập ban đầu là 0
  cob_x_manual_adjustment_m_ = 0;
  cob_y_manual_adjustment_m_ = 0;
  cob_z_manual_adjustment_m_ = 0;

  // Hệ số lợi tức giảm tốc (gyro_balance_gain_ratio_) và các hệ số lợi tức giảm tốc cho cân bằng theo giảm tốc (gyro_balance_roll_gain_, gyro_balance_pitch_gain_) được thiết lập ban đầu
  gyro_balance_gain_ratio_ = 0.0;
  gyro_balance_roll_gain_ = -0.10 * 0.75 * gyro_balance_gain_ratio_;
  gyro_balance_pitch_gain_ = -0.10 * 0.5 * gyro_balance_gain_ratio_;

  // Các điều chỉnh tối đa cho cob và chân được thiết lập ban đầu
  cob_x_adjustment_abs_max_m_ = 0.05;
  cob_y_adjustment_abs_max_m_ = 0.05;
  cob_z_adjustment_abs_max_m_ = 0.05;
  cob_roll_adjustment_abs_max_rad_ = 15.0 * DEGREE2RADIAN;
  cob_pitch_adjustment_abs_max_rad_ = 15.0 * DEGREE2RADIAN;
  cob_yaw_adjustment_abs_max_rad_ = 15.0 * DEGREE2RADIAN;
  foot_x_adjustment_abs_max_m_ = 0.05;
  foot_y_adjustment_abs_max_m_ = 0.05;
  foot_z_adjustment_abs_max_m_ = 0.05;
  foot_roll_adjustment_abs_max_rad_ = 15.0 * DEGREE2RADIAN;
  foot_pitch_adjustment_abs_max_rad_ = 15.0 * DEGREE2RADIAN;
  foot_yaw_adjustment_abs_max_rad_ = 15.0 * DEGREE2RADIAN;

  // Ma trận biến đổi được sửa đổi ban đầu là ma trận đơn vị
  mat_robot_to_cob_modified_ = Eigen::MatrixXd::Identity(4, 4);
  mat_robot_to_right_foot_modified_ = Eigen::MatrixXd::Identity(4, 4);
  mat_robot_to_left_foot_modified_ = Eigen::MatrixXd::Identity(4, 4);

  // Vector điều chỉnh vị trí của cob và chân được sửa đổi ban đầu là vector 0
  pose_cob_adjustment_ = Eigen::VectorXd::Zero(6);
  pose_right_foot_adjustment_ = Eigen::VectorXd::Zero(6);;
  pose_left_foot_adjustment_ = Eigen::VectorXd::Zero(6);;
}





// Destructor của lớp BalanceControlUsingDampingConroller
BalanceControlUsingDampingConroller::~BalanceControlUsingDampingConroller()
{
  // Không có công việc cụ thể nào cần thực hiện trong hàm hủy này.
}

// Phương thức khởi tạo (initialize) của lớp BalanceControlUsingDampingConroller
void BalanceControlUsingDampingConroller::initialize(const int control_cycle_msec)
{
  // Thiết lập mã lỗi kiểm soát ban đầu là không có lỗi (NoError)
  balance_control_error_ = BalanceControlError::NoError;

  // Thiết lập chu kỳ kiểm soát (control_cycle_sec_) dựa trên chu kỳ kiểm soát được truyền vào (control_cycle_msec)
  control_cycle_sec_ = control_cycle_msec * 0.001;

  // Thiết lập tần số cắt và hệ số alpha cho bộ lọc thấp gyro
  gyro_cut_off_freq_ = 10.0;
  gyro_lpf_alpha_ = 2.0 * M_PI * gyro_cut_off_freq_ * control_cycle_sec_ / (1.0 + 2.0 * M_PI * gyro_cut_off_freq_ * control_cycle_sec_);

  // Thiết lập vector điều chỉnh vị trí của cob và chân về 0
  pose_cob_adjustment_.fill(0);
  pose_right_foot_adjustment_.fill(0);
  pose_left_foot_adjustment_.fill(0);

  // Thiết lập chu kỳ kiểm soát cho các bộ điều khiển góc của chân
  foot_roll_angle_ctrl_.control_cycle_sec_ = control_cycle_sec_;
  foot_pitch_angle_ctrl_.control_cycle_sec_ = control_cycle_sec_;

  // Thiết lập chu kỳ kiểm soát cho bộ điều khiển lực và mô-men lực của chân
  foot_force_z_diff_ctrl_.control_cycle_sec_ = control_cycle_sec_;
  right_foot_force_z_ctrl_.control_cycle_sec_ = control_cycle_sec_;
  left_foot_force_z_ctrl_.control_cycle_sec_ = control_cycle_sec_;

  // Thiết lập chu kỳ kiểm soát cho bộ điều khiển lực và mô-men lực của chân phải
  right_foot_force_x_ctrl_.control_cycle_sec_ = control_cycle_sec_;
  right_foot_force_y_ctrl_.control_cycle_sec_ = control_cycle_sec_;
  right_foot_torque_roll_ctrl_.control_cycle_sec_ = control_cycle_sec_;
  right_foot_torque_pitch_ctrl_.control_cycle_sec_ = control_cycle_sec_;

  // Thiết lập chu kỳ kiểm soát cho bộ điều khiển lực và mô-men lực của chân trái
  left_foot_force_x_ctrl_.control_cycle_sec_ = control_cycle_sec_;
  left_foot_force_y_ctrl_.control_cycle_sec_ = control_cycle_sec_;
  left_foot_torque_roll_ctrl_.control_cycle_sec_ = control_cycle_sec_;
  left_foot_torque_pitch_ctrl_.control_cycle_sec_ = control_cycle_sec_;
}




// Phương thức này cập nhật trạng thái của chế độ cân bằng theo giả định từ cảm biến gyro.
// Nếu enable là true, chế độ cân bằng theo giả định sẽ được bật (gyro_enable_ = 1.0), ngược lại sẽ bị tắt (gyro_enable_ = 0.0).
void BalanceControlUsingDampingConroller::setGyroBalanceEnable(bool enable)
{
  if (enable)
    gyro_enable_ = 1.0;  // Bật chế độ cân bằng theo giả định từ cảm biến gyro
  else
    gyro_enable_ = 0.0;  // Tắt chế độ cân bằng theo giả định từ cảm biến gyro
}

// Phương thức này cập nhật trạng thái của chế độ cân bằng theo giả định từ cảm biến định hướng.
// Nếu enable là true, chế độ cân bằng theo giả định sẽ được bật (orientation_enable_ = 1.0), ngược lại sẽ bị tắt (orientation_enable_ = 0.0).
void BalanceControlUsingDampingConroller::setOrientationBalanceEnable(bool enable)
{
  if (enable)
    orientation_enable_ = 1.0;  // Bật chế độ cân bằng theo giả định từ cảm biến định hướng
  else
    orientation_enable_ = 0.0;  // Tắt chế độ cân bằng theo giả định từ cảm biến định hướng
}

// Phương thức này cập nhật trạng thái của chế độ cân bằng theo giả định từ lực và mô-men lực cảm biến.
// Nếu enable là true, chế độ cân bằng theo giả định sẽ được bật (ft_enable_ = 1.0), ngược lại sẽ bị tắt (ft_enable_ = 0.0).
void BalanceControlUsingDampingConroller::setForceTorqueBalanceEnable(bool enable)
{
  if (enable)
    ft_enable_ = 1.0;  // Bật chế độ cân bằng theo giả định từ lực và mô-men lực cảm biến
  else
    ft_enable_ = 0.0;  // Tắt chế độ cân bằng theo giả định từ lực và mô-men lực cảm biến
}



// Phương thức này thực hiện quá trình xử lý điều khiển cân bằng và cập nhật trạng thái của robot dựa trên các cảm biến và thông số cấu hình.
void BalanceControlUsingDampingConroller::process(int *balance_error, Eigen::MatrixXd *robot_to_cob_modified, Eigen::MatrixXd *robot_to_right_foot_modified, Eigen::MatrixXd *robot_to_left_foot_modified)
{
  // Đặt trạng thái lỗi cân bằng về không lỗi
  balance_control_error_ = BalanceControlError::NoError;

  // Thiết lập điều chỉnh vị trí trung tâm cơ bản (CoB) ban đầu về không
  pose_cob_adjustment_.fill(0);

  // Thiết lập điều chỉnh vị trí chân phải ban đầu về không
  pose_right_foot_adjustment_.fill(0);

  // Thiết lập điều chỉnh vị trí chân trái ban đầu về không
  pose_left_foot_adjustment_.fill(0);

  // Xử lý cân bằng theo giả định từ cảm biến gyro
  gyro_roll_filtered_ = current_gyro_roll_rad_per_sec_ * gyro_lpf_alpha_ + (1.0 - gyro_lpf_alpha_) * gyro_roll_filtered_;
  gyro_pitch_filtered_ = current_gyro_pitch_rad_per_sec_ * gyro_lpf_alpha_ + (1.0 - gyro_lpf_alpha_) * gyro_pitch_filtered_;

  // Tính điều chỉnh vị trí chân dựa trên giả định từ cảm biến gyro
  foot_roll_adjustment_by_gyro_roll_ = gyro_enable_ * (desired_gyro_roll_ - gyro_roll_filtered_) * gyro_balance_roll_gain_;
  foot_pitch_adjustment_by_gyro_pitch_ = gyro_enable_ * (desired_gyro_pitch_ - gyro_pitch_filtered_) * gyro_balance_pitch_gain_;

  // Xử lý cân bằng theo giả định từ cảm biến định hướng
  foot_roll_adjustment_by_orientation_roll_ = orientation_enable_ * foot_roll_angle_ctrl_.getDampingControllerOutput(current_orientation_roll_rad_);
  foot_pitch_adjustment_by_orientation_pitch_ = orientation_enable_ * foot_pitch_angle_ctrl_.getDampingControllerOutput(current_orientation_pitch_rad_);

  // Tính ma trận điều chỉnh vị trí chân dựa trên giả định từ cảm biến định hướng
  Eigen::MatrixXd mat_orientation_adjustment_by_imu = robotis_framework::getRotation4d(foot_roll_adjustment_by_gyro_roll_ + foot_roll_adjustment_by_orientation_roll_, foot_pitch_adjustment_by_gyro_pitch_ + foot_pitch_adjustment_by_orientation_pitch_, 0.0);
  Eigen::MatrixXd mat_r_xy, mat_l_xy;
  mat_r_xy.resize(4, 1);
  mat_l_xy.resize(4, 1);

  // Thiết lập ma trận vị trí chân phải và chân trái ban đầu
  mat_r_xy.coeffRef(0, 0) = desired_robot_to_right_foot_.coeff(0, 3) - 0.5 * (desired_robot_to_right_foot_.coeff(0, 3) + desired_robot_to_left_foot_.coeff(0, 3));
  mat_r_xy.coeffRef(1, 0) = desired_robot_to_right_foot_.coeff(1, 3) - 0.5 * (desired_robot_to_right_foot_.coeff(1, 3) + desired_robot_to_left_foot_.coeff(1, 3));
  mat_r_xy.coeffRef(2, 0) = 0.0;
  mat_r_xy.coeffRef(3, 0) = 1;

  mat_l_xy.coeffRef(0, 0) = desired_robot_to_left_foot_.coeff(0, 3) - 0.5 * (desired_robot_to_right_foot_.coeff(0, 3) + desired_robot_to_left_foot_.coeff(0, 3));
  mat_l_xy.coeffRef(1, 0) = desired_robot_to_left_foot_.coeff(1, 3) - 0.5 * (desired_robot_to_right_foot_.coeff(1, 3) + desired_robot_to_left_foot_.coeff(1, 3));
  mat_l_xy.coeffRef(2, 0) = 0.0;
  mat_l_xy.coeffRef(3, 0) = 1;

  // Ứng dụng điều chỉnh của cảm biến IMU cho vị trí của chân
  mat_r_xy = mat_orientation_adjustment_by_imu * mat_r_xy;
  mat_l_xy = mat_orientation_adjustment_by_imu * mat_l_xy;



// Xử lý cân bằng dựa trên cảm biến lực và mô-men lực (ft sensor)

// Điều chỉnh vị trí dọc của cơ bản (Center of Mass - CoM) dựa trên chênh lệch lực dọc giữa chân trái và chân phải
foot_z_adjustment_by_force_z_difference_ = ft_enable_ * 0.001 * foot_force_z_diff_ctrl_.getDampingControllerOutput(current_left_fz_N_ - current_right_fz_N_);

// Điều chỉnh vị trí dọc của chân phải và chân trái dựa trên lực dọc tương ứng
r_foot_z_adjustment_by_force_z_ = ft_enable_ * 0.001 * right_foot_force_z_ctrl_.getDampingControllerOutput(current_right_fz_N_);
l_foot_z_adjustment_by_force_z_ = ft_enable_ * 0.001 * left_foot_force_z_ctrl_.getDampingControllerOutput(current_left_fz_N_);

// Điều chỉnh vị trí ngang và góc quay của chân phải dựa trên lực nằm ngang và mô-men lực tương ứng
r_foot_x_adjustment_by_force_x_ = ft_enable_ * 0.001 * right_foot_force_x_ctrl_.getDampingControllerOutput(current_right_fx_N_);
r_foot_y_adjustment_by_force_y_ = ft_enable_ * 0.001 * right_foot_force_y_ctrl_.getDampingControllerOutput(current_right_fy_N_);
r_foot_roll_adjustment_by_torque_roll_ = ft_enable_ * right_foot_torque_roll_ctrl_.getDampingControllerOutput(current_right_tx_Nm_);
r_foot_pitch_adjustment_by_torque_pitch_ = ft_enable_ * right_foot_torque_pitch_ctrl_.getDampingControllerOutput(current_right_ty_Nm_);

// Điều chỉnh vị trí ngang và góc quay của chân trái dựa trên lực nằm ngang và mô-men lực tương ứng
l_foot_x_adjustment_by_force_x_ = ft_enable_ * 0.001 * left_foot_force_x_ctrl_.getDampingControllerOutput(current_left_fx_N_);
l_foot_y_adjustment_by_force_y_ = ft_enable_ * 0.001 * left_foot_force_y_ctrl_.getDampingControllerOutput(current_left_fy_N_);
l_foot_roll_adjustment_by_torque_roll_ = ft_enable_ * left_foot_torque_roll_ctrl_.getDampingControllerOutput(current_left_tx_Nm_);
l_foot_pitch_adjustment_by_torque_pitch_ = ft_enable_ * left_foot_torque_pitch_ctrl_.getDampingControllerOutput(current_left_ty_Nm_);

// Điều chỉnh thêm vị trí dựa trên sự chênh lệch giữa giá trị mong muốn và giá trị hiện tại của lực dọc của chân phải và chân trái
r_foot_z_adjustment_by_force_z_ += ft_enable_ * 0.001 * 0.0 * (right_foot_force_z_ctrl_.desired_ - current_right_fz_N_);
l_foot_z_adjustment_by_force_z_ += ft_enable_ * 0.001 * 0.0 * (left_foot_force_z_ctrl_.desired_ - current_left_fz_N_);

// Tổng hợp kết quả điều chỉnh từ cảm biến lực và mô-men lực

// Điều chỉnh vị trí và góc quay của cơ bản
pose_cob_adjustment_.coeffRef(0) = cob_x_manual_adjustment_m_;
pose_cob_adjustment_.coeffRef(1) = cob_y_manual_adjustment_m_;
pose_cob_adjustment_.coeffRef(2) = cob_z_manual_adjustment_m_;

// Điều chỉnh vị trí và góc quay của chân phải
pose_right_foot_adjustment_.coeffRef(0) = r_foot_x_adjustment_by_force_x_;
pose_right_foot_adjustment_.coeffRef(1) = r_foot_y_adjustment_by_force_y_;
pose_right_foot_adjustment_.coeffRef(2) = 0.5 * 0.0 * foot_z_adjustment_by_force_z_difference_ + mat_r_xy.coeff(2, 0) + r_foot_z_adjustment_by_force_z_ * 1.0;
pose_right_foot_adjustment_.coeffRef(3) = (foot_roll_adjustment_by_gyro_roll_ + foot_roll_adjustment_by_orientation_roll_ + r_foot_roll_adjustment_by_torque_roll_);
pose_right_foot_adjustment_.coeffRef(4) = (foot_pitch_adjustment_by_gyro_pitch_ + foot_pitch_adjustment_by_orientation_pitch_ + r_foot_pitch_adjustment_by_torque_pitch_);

// Điều chỉnh vị trí và góc quay của chân trái
pose_left_foot_adjustment_.coeffRef(0) = l_foot_x_adjustment_by_force_x_;
pose_left_foot_adjustment_.coeffRef(1) = l_foot_y_adjustment_by_force_y_;
pose_left_foot_adjustment_.coeffRef(2) = -0.5 * 0.0 * foot_z_adjustment_by_force_z_difference_ + mat_l_xy.coeff(2, 0) + l_foot_z_adjustment_by_force_z_ * 1.0;
pose_left_foot_adjustment_.coeffRef(3) = (foot_roll_adjustment_by_gyro_roll_ + foot_roll_adjustment_by_orientation_roll_ + l_foot_roll_adjustment_by_torque_roll_);
pose_left_foot_adjustment_.coeffRef(4) = (foot_pitch_adjustment_by_gyro_pitch_ + foot_pitch_adjustment_by_orientation_pitch_ + l_foot_pitch_adjustment_by_torque_pitch_);


  // Hạn chế giá trị điều chỉnh để không vượt quá giới hạn
  if((fabs(pose_cob_adjustment_.coeff(0)) == cob_x_adjustment_abs_max_m_      ) ||
     (fabs(pose_cob_adjustment_.coeff(1)) == cob_y_adjustment_abs_max_m_      ) ||
     (fabs(pose_cob_adjustment_.coeff(2)) == cob_z_adjustment_abs_max_m_      ) ||
     (fabs(pose_cob_adjustment_.coeff(3)) == cob_roll_adjustment_abs_max_rad_ ) ||
     (fabs(pose_cob_adjustment_.coeff(4)) == cob_pitch_adjustment_abs_max_rad_) ||
     (fabs(pose_right_foot_adjustment_.coeff(0)) == foot_x_adjustment_abs_max_m_      ) ||
     (fabs(pose_right_foot_adjustment_.coeff(1)) == foot_y_adjustment_abs_max_m_      ) ||
     (fabs(pose_right_foot_adjustment_.coeff(2)) == foot_z_adjustment_abs_max_m_      ) ||
     (fabs(pose_right_foot_adjustment_.coeff(3)) == foot_roll_adjustment_abs_max_rad_ ) ||
     (fabs(pose_right_foot_adjustment_.coeff(4)) == foot_pitch_adjustment_abs_max_rad_) ||
     (fabs(pose_left_foot_adjustment_.coeff(0)) == foot_x_adjustment_abs_max_m_      ) ||
     (fabs(pose_left_foot_adjustment_.coeff(1)) == foot_y_adjustment_abs_max_m_      ) ||
     (fabs(pose_left_foot_adjustment_.coeff(2)) == foot_z_adjustment_abs_max_m_      ) ||
     (fabs(pose_left_foot_adjustment_.coeff(3)) == foot_roll_adjustment_abs_max_rad_ ) ||
     (fabs(pose_left_foot_adjustment_.coeff(4)) == foot_pitch_adjustment_abs_max_rad_))
    balance_control_error_ &= BalanceControlError::BalanceLimit;

// Hạn chế giá trị điều chỉnh để không vượt quá giới hạn
  pose_cob_adjustment_.coeffRef(0) = copysign(fmin(fabs(pose_cob_adjustment_.coeff(0)), cob_x_adjustment_abs_max_m_      ), pose_cob_adjustment_.coeff(0));
  pose_cob_adjustment_.coeffRef(1) = copysign(fmin(fabs(pose_cob_adjustment_.coeff(1)), cob_x_adjustment_abs_max_m_      ), pose_cob_adjustment_.coeff(1));
  pose_cob_adjustment_.coeffRef(2) = copysign(fmin(fabs(pose_cob_adjustment_.coeff(2)), cob_x_adjustment_abs_max_m_      ), pose_cob_adjustment_.coeff(2));
  pose_cob_adjustment_.coeffRef(3) = copysign(fmin(fabs(pose_cob_adjustment_.coeff(3)), cob_roll_adjustment_abs_max_rad_ ), pose_cob_adjustment_.coeff(3));
  pose_cob_adjustment_.coeffRef(4) = copysign(fmin(fabs(pose_cob_adjustment_.coeff(4)), cob_pitch_adjustment_abs_max_rad_), pose_cob_adjustment_.coeff(4));
  pose_cob_adjustment_.coeffRef(5) = 0;

  pose_right_foot_adjustment_.coeffRef(0) = copysign(fmin(fabs(pose_right_foot_adjustment_.coeff(0)), foot_x_adjustment_abs_max_m_      ), pose_right_foot_adjustment_.coeff(0));
  pose_right_foot_adjustment_.coeffRef(1) = copysign(fmin(fabs(pose_right_foot_adjustment_.coeff(1)), foot_y_adjustment_abs_max_m_      ), pose_right_foot_adjustment_.coeff(1));
  pose_right_foot_adjustment_.coeffRef(2) = copysign(fmin(fabs(pose_right_foot_adjustment_.coeff(2)), foot_z_adjustment_abs_max_m_      ), pose_right_foot_adjustment_.coeff(2));
  pose_right_foot_adjustment_.coeffRef(3) = copysign(fmin(fabs(pose_right_foot_adjustment_.coeff(3)), foot_roll_adjustment_abs_max_rad_ ), pose_right_foot_adjustment_.coeff(3));
  pose_right_foot_adjustment_.coeffRef(4) = copysign(fmin(fabs(pose_right_foot_adjustment_.coeff(4)), foot_pitch_adjustment_abs_max_rad_), pose_right_foot_adjustment_.coeff(4));
  pose_right_foot_adjustment_.coeffRef(5) = 0;

  pose_left_foot_adjustment_.coeffRef(0) = copysign(fmin(fabs(pose_left_foot_adjustment_.coeff(0)), foot_x_adjustment_abs_max_m_      ), pose_left_foot_adjustment_.coeff(0));
  pose_left_foot_adjustment_.coeffRef(1) = copysign(fmin(fabs(pose_left_foot_adjustment_.coeff(1)), foot_y_adjustment_abs_max_m_      ), pose_left_foot_adjustment_.coeff(1));
  pose_left_foot_adjustment_.coeffRef(2) = copysign(fmin(fabs(pose_left_foot_adjustment_.coeff(2)), foot_z_adjustment_abs_max_m_      ), pose_left_foot_adjustment_.coeff(2));
  pose_left_foot_adjustment_.coeffRef(3) = copysign(fmin(fabs(pose_left_foot_adjustment_.coeff(3)), foot_roll_adjustment_abs_max_rad_ ), pose_left_foot_adjustment_.coeff(3));
  pose_left_foot_adjustment_.coeffRef(4) = copysign(fmin(fabs(pose_left_foot_adjustment_.coeff(4)), foot_pitch_adjustment_abs_max_rad_), pose_left_foot_adjustment_.coeff(4));
  pose_left_foot_adjustment_.coeffRef(5) = 0;

// Điều chỉnh các ma trận xoay và vị trí dựa trên giá trị điều chỉnh
  Eigen::MatrixXd cob_rotation_adj = robotis_framework::getRotationZ(pose_cob_adjustment_.coeff(5)) * robotis_framework::getRotationY(pose_cob_adjustment_.coeff(4)) * robotis_framework::getRotationX(pose_cob_adjustment_.coeff(3));
  Eigen::MatrixXd rf_rotation_adj = robotis_framework::getRotationZ(pose_right_foot_adjustment_.coeff(5)) * robotis_framework::getRotationY(pose_right_foot_adjustment_.coeff(4)) * robotis_framework::getRotationX(pose_right_foot_adjustment_.coeff(3));
  Eigen::MatrixXd lf_rotation_adj = robotis_framework::getRotationZ(pose_left_foot_adjustment_.coeff(5)) * robotis_framework::getRotationY(pose_left_foot_adjustment_.coeff(4)) * robotis_framework::getRotationX(pose_left_foot_adjustment_.coeff(3));
  mat_robot_to_cob_modified_.block<3,3>(0,0) = cob_rotation_adj * desired_robot_to_cob_.block<3,3>(0,0);
  mat_robot_to_right_foot_modified_.block<3,3>(0,0) = rf_rotation_adj * desired_robot_to_right_foot_.block<3,3>(0,0);;
  mat_robot_to_left_foot_modified_.block<3,3>(0,0) = lf_rotation_adj * desired_robot_to_left_foot_.block<3,3>(0,0);;

// Cập nhật vị trí mới cho COB và chân phải, chân trái
  mat_robot_to_cob_modified_.coeffRef(0,3) = desired_robot_to_cob_.coeff(0,3) + pose_cob_adjustment_.coeff(0);
  mat_robot_to_cob_modified_.coeffRef(1,3) = desired_robot_to_cob_.coeff(1,3) + pose_cob_adjustment_.coeff(1);
  mat_robot_to_cob_modified_.coeffRef(2,3) = desired_robot_to_cob_.coeff(2,3) + pose_cob_adjustment_.coeff(2);

  mat_robot_to_right_foot_modified_.coeffRef(0,3) = desired_robot_to_right_foot_.coeff(0,3) + pose_right_foot_adjustment_.coeff(0);
  mat_robot_to_right_foot_modified_.coeffRef(1,3) = desired_robot_to_right_foot_.coeff(1,3) + pose_right_foot_adjustment_.coeff(1);
  mat_robot_to_right_foot_modified_.coeffRef(2,3) = desired_robot_to_right_foot_.coeff(2,3) + pose_right_foot_adjustment_.coeff(2);

  mat_robot_to_left_foot_modified_.coeffRef(0,3) = desired_robot_to_left_foot_.coeff(0,3) + pose_left_foot_adjustment_.coeff(0);
  mat_robot_to_left_foot_modified_.coeffRef(1,3) = desired_robot_to_left_foot_.coeff(1,3) + pose_left_foot_adjustment_.coeff(1);
  mat_robot_to_left_foot_modified_.coeffRef(2,3) = desired_robot_to_left_foot_.coeff(2,3) + pose_left_foot_adjustment_.coeff(2);

// Truyền giá trị lỗi cân bằng nếu có
  if(balance_error != 0)
    *balance_error = balance_control_error_;
// Gán giá trị ma trận mới cho các biến con trỏ
  *robot_to_cob_modified        = mat_robot_to_cob_modified_;
  *robot_to_right_foot_modified = mat_robot_to_right_foot_modified_;
  *robot_to_left_foot_modified  = mat_robot_to_left_foot_modified_;
}






// Chuẩn bị dữ liệu vị trí mong muốn cho COB và chân phải, chân trái
// Input:
//   robot_to_cob: Ma trận vị trí mong muốn cho COB
//   robot_to_right_foot: Ma trận vị trí mong muốn cho chân phải
//   robot_to_left_foot: Ma trận vị trí mong muốn cho chân trái
void BalanceControlUsingDampingConroller::setDesiredPose(const Eigen::MatrixXd &robot_to_cob, const Eigen::MatrixXd &robot_to_right_foot, const Eigen::MatrixXd &robot_to_left_foot)
{
  // Lưu trữ giá trị vị trí mong muốn cho COB và chân phải, chân trái
  desired_robot_to_cob_        = robot_to_cob;
  desired_robot_to_right_foot_ = robot_to_right_foot;
  desired_robot_to_left_foot_  = robot_to_left_foot;
}

// Thiết lập giá trị góc gyro mong muốn cho COB
// Input:
//   gyro_roll: Góc roll mong muốn
//   gyro_pitch: Góc pitch mong muốn
void BalanceControlUsingDampingConroller::setDesiredCOBGyro(double gyro_roll, double gyro_pitch)
{
  // Lưu trữ giá trị góc gyro mong muốn cho COB
  desired_gyro_roll_  = gyro_roll;
  desired_gyro_pitch_ = gyro_pitch;
}

// Thiết lập góc độ mong muốn cho COB
// Input:
//   cob_orientation_roll: Góc roll mong muốn
//   cob_orientation_pitch: Góc pitch mong muốn
void BalanceControlUsingDampingConroller::setDesiredCOBOrientation(double cob_orientation_roll, double cob_orientation_pitch)
{
  // Lưu trữ giá trị góc độ mong muốn cho COB
  foot_roll_angle_ctrl_.desired_  = cob_orientation_roll;
  foot_pitch_angle_ctrl_.desired_ = cob_orientation_pitch;
}



// Thiết lập giá trị mong muốn cho lực và mô-men của chân phải và chân trái
// Input:
//   r_force_x_N, r_force_y_N, r_force_z_N: Lực theo trục x, y, z của chân phải
//   r_torque_roll_Nm, r_torque_pitch_Nm, r_torque_yaw_Nm: Mô-men xoay theo trục roll, pitch, yaw của chân phải
//   l_force_x_N, l_force_y_N, l_force_z_N: Lực theo trục x, y, z của chân trái
//   l_torque_roll_Nm, l_torque_pitch_Nm, l_torque_yaw_Nm: Mô-men xoay theo trục roll, pitch, yaw của chân trái
void BalanceControlUsingDampingConroller::setDesiredFootForceTorque(double r_force_x_N, double r_force_y_N, double r_force_z_N,
                                                                     double r_torque_roll_Nm, double r_torque_pitch_Nm, double r_torque_yaw_Nm,
                                                                     double l_force_x_N, double l_force_y_N, double l_force_z_N,
                                                                     double l_torque_roll_Nm, double l_torque_pitch_Nm, double l_torque_yaw_Nm)
{
  // Tính chênh lệch lực theo trục z giữa chân trái và chân phải
  foot_force_z_diff_ctrl_.desired_    = l_force_z_N - r_force_z_N;
  // Lực mong muốn cho chân phải và chân trái
  right_foot_force_z_ctrl_.desired_   = r_force_z_N;
  left_foot_force_z_ctrl_.desired_    = l_force_z_N;

  // Lực và mô-men xoay mong muốn cho chân phải
  right_foot_force_x_ctrl_.desired_      = r_force_x_N;
  right_foot_force_y_ctrl_.desired_      = r_force_y_N;
  right_foot_torque_roll_ctrl_.desired_  = r_torque_roll_Nm;
  right_foot_torque_pitch_ctrl_.desired_ = r_torque_pitch_Nm;

  // Lực và mô-men xoay mong muốn cho chân trái
  left_foot_force_x_ctrl_.desired_       = l_force_x_N;
  left_foot_force_y_ctrl_.desired_       = l_force_y_N;
  left_foot_torque_roll_ctrl_.desired_   = l_torque_roll_Nm;
  left_foot_torque_pitch_ctrl_.desired_  = l_torque_pitch_Nm;
}


// Thiết lập giá trị đầu ra cảm biến gyro hiện tại
// Input:
//   gyro_roll: Góc roll của gyro hiện tại
//   gyro_pitch: Góc pitch của gyro hiện tại
void BalanceControlUsingDampingConroller::setCurrentGyroSensorOutput(double gyro_roll, double gyro_pitch)
{
  // Lưu giữ giá trị đầu ra của cảm biến gyro hiện tại
  current_gyro_roll_rad_per_sec_  = gyro_roll;
  current_gyro_pitch_rad_per_sec_ = gyro_pitch;
}

// Thiết lập giá trị đầu ra cảm biến hướng hiện tại
// Input:
//   cob_orientation_roll: Góc roll của cảm biến hướng hiện tại
//   cob_orientation_pitch: Góc pitch của cảm biến hướng hiện tại
void BalanceControlUsingDampingConroller::setCurrentOrientationSensorOutput(double cob_orientation_roll, double cob_orientation_pitch)
{
  // Lưu giữ giá trị đầu ra của cảm biến hướng hiện tại
  current_orientation_roll_rad_  = cob_orientation_roll;
  current_orientation_pitch_rad_ = cob_orientation_pitch;
}


// Cập nhật giá trị đầu ra của cảm biến lực và mô-men của chân phải và chân trái
// Input:
//   r_force_x_N, r_force_y_N, r_force_z_N: Lực theo trục x, y, z của chân phải
//   r_torque_roll_Nm, r_torque_pitch_Nm, r_torque_yaw_Nm: Mô-men xoay theo trục roll, pitch, yaw của chân phải
//   l_force_x_N, l_force_y_N, l_force_z_N: Lực theo trục x, y, z của chân trái
//   l_torque_roll_Nm, l_torque_pitch_Nm, l_torque_yaw_Nm: Mô-men xoay theo trục roll, pitch, yaw của chân trái
void BalanceControlUsingDampingConroller::setCurrentFootForceTorqueSensorOutput(double r_force_x_N, double r_force_y_N, double r_force_z_N,
                                                                                 double r_torque_roll_Nm, double r_torque_pitch_Nm, double r_torque_yaw_Nm,
                                                                                 double l_force_x_N, double l_force_y_N, double l_force_z_N,
                                                                                 double l_torque_roll_Nm, double l_torque_pitch_Nm, double l_torque_yaw_Nm)
{
  // Lưu trữ giá trị đầu ra của cảm biến lực và mô-men cho chân phải
  current_right_fx_N_  = r_force_x_N;
  current_right_fy_N_  = r_force_y_N;
  current_right_fz_N_  = r_force_z_N;
  current_right_tx_Nm_ = r_torque_roll_Nm;
  current_right_ty_Nm_ = r_torque_pitch_Nm;
  current_right_tz_Nm_ = r_torque_yaw_Nm;

  // Lưu trữ giá trị đầu ra của cảm biến lực và mô-men cho chân trái
  current_left_fx_N_  = l_force_x_N;
  current_left_fy_N_  = l_force_y_N;
  current_left_fz_N_  = l_force_z_N;
  current_left_tx_Nm_ = l_torque_roll_Nm;
  current_left_ty_Nm_ = l_torque_pitch_Nm;
  current_left_tz_Nm_ = l_torque_yaw_Nm;
}

// Thiết lập giá trị tối đa cho việc điều chỉnh
// Input:
//   cob_x_max_adjustment_m, cob_y_max_adjustment_m, cob_z_max_adjustment_m: Giá trị tối đa cho điều chỉnh theo trục x, y, z của COB
//   cob_roll_max_adjustment_rad, cob_pitch_max_adjustment_rad, cob_yaw_max_adjustment_rad: Giá trị tối đa cho điều chỉnh theo trục roll, pitch, yaw của COB
//   foot_x_max_adjustment_m, foot_y_max_adjustment_m, foot_z_max_adjustment_m: Giá trị tối đa cho điều chỉnh theo trục x, y, z của chân
//   foot_roll_max_adjustment_rad, foot_pitch_max_adjustment_rad, foot_yaw_max_adjustment_rad: Giá trị tối đa cho điều chỉnh theo trục roll, pitch, yaw của chân
void BalanceControlUsingDampingConroller::setMaximumAdjustment(double cob_x_max_adjustment_m, double cob_y_max_adjustment_m, double cob_z_max_adjustment_m,
                                                                double cob_roll_max_adjustment_rad, double cob_pitch_max_adjustment_rad, double cob_yaw_max_adjustment_rad,
                                                                double foot_x_max_adjustment_m, double foot_y_max_adjustment_m, double foot_z_max_adjustment_m,
                                                                double foot_roll_max_adjustment_rad, double foot_pitch_max_adjustment_rad, double foot_yaw_max_adjustment_rad)
{
  // Thiết lập giá trị tối đa cho điều chỉnh của COB
  cob_x_adjustment_abs_max_m_        = cob_x_max_adjustment_m;
  cob_y_adjustment_abs_max_m_        = cob_y_max_adjustment_m;
  cob_z_adjustment_abs_max_m_        = cob_z_max_adjustment_m;
  cob_roll_adjustment_abs_max_rad_   = cob_roll_max_adjustment_rad;
  cob_pitch_adjustment_abs_max_rad_  = cob_pitch_max_adjustment_rad;
  cob_yaw_adjustment_abs_max_rad_    = cob_yaw_max_adjustment_rad;

  // Thiết lập giá trị tối đa cho điều chỉnh của chân
  foot_x_adjustment_abs_max_m_       = foot_x_max_adjustment_m;
  foot_y_adjustment_abs_max_m_       = foot_y_max_adjustment_m;
  foot_z_adjustment_abs_max_m_       = foot_z_max_adjustment_m;
  foot_roll_adjustment_abs_max_rad_  = foot_roll_max_adjustment_rad;
  foot_pitch_adjustment_abs_max_rad_ = foot_pitch_max_adjustment_rad;
  foot_yaw_adjustment_abs_max_rad_   = foot_yaw_max_adjustment_rad;
}

// Thiết lập điều chỉnh thủ công của COB
// Input:
//   cob_x_adjustment_m, cob_y_adjustment_m, cob_z_adjustment_m: Giá trị điều chỉnh thủ công theo trục x, y, z của COB
void BalanceControlUsingDampingConroller::setCOBManualAdjustment(double cob_x_adjustment_m, double cob_y_adjustment_m, double cob_z_adjustment_m)
{
  cob_x_manual_adjustment_m_ = cob_x_adjustment_m;
  cob_y_manual_adjustment_m_ = cob_y_adjustment_m;
  cob_z_manual_adjustment_m_ = cob_z_adjustment_m;
}

// Lấy giá trị điều chỉnh thủ công theo trục x của COB
double BalanceControlUsingDampingConroller::getCOBManualAdjustmentX()
{
  return cob_x_manual_adjustment_m_;
}

// Lấy giá trị điều chỉnh thủ công theo trục y của COB
double BalanceControlUsingDampingConroller::getCOBManualAdjustmentY()
{
  return cob_y_manual_adjustment_m_;
}

// Lấy giá trị điều chỉnh thủ công theo trục z của COB
double BalanceControlUsingDampingConroller::getCOBManualAdjustmentZ()
{
  return cob_z_manual_adjustment_m_;
}

// Thiết lập tỷ lệ cân bằng của gyro
// Input:
//   gyro_balance_gain_ratio: Tỷ lệ cân bằng của gyro
void BalanceControlUsingDampingConroller::setGyroBalanceGainRatio(double gyro_balance_gain_ratio)
{
  // Thiết lập tỷ lệ cân bằng của gyro
  gyro_balance_gain_ratio_ = gyro_balance_gain_ratio;

  // Tính toán các hệ số cân bằng roll và pitch dựa trên tỷ lệ cân bằng
  gyro_balance_roll_gain_  = -0.10 * 0.75 * gyro_balance_gain_ratio_;
  gyro_balance_pitch_gain_ = -0.10 * 0.5  * gyro_balance_gain_ratio_;
}

// Lấy giá trị tỷ lệ cân bằng của gyro
double BalanceControlUsingDampingConroller::getGyroBalanceGainRatio(void)
{
  return gyro_balance_gain_ratio_;
}


// Constructor của lớp BalanceControlUsingPDController
BalanceControlUsingPDController::BalanceControlUsingPDController()
{
  // Khởi tạo biến lưu trạng thái lỗi cân bằng
  balance_control_error_ = BalanceControlError::NoError;

  // Khởi tạo chu kỳ điều khiển mặc định
  control_cycle_sec_ = 0.008;

  // Khởi tạo trạng thái kích hoạt của các phương pháp cân bằng (gyro, orientation, force-torque)
  gyro_enable_ = 1.0;
  orientation_enable_ = 1.0;
  ft_enable_ = 1.0;

  // Khởi tạo ma trận biến đổi vị trí mong muốn của robot đối với COB và hai chân
  desired_robot_to_cob_         = Eigen::MatrixXd::Identity(4,4);
  desired_robot_to_right_foot_  = Eigen::MatrixXd::Identity(4,4);
  desired_robot_to_left_foot_   = Eigen::MatrixXd::Identity(4,4);

  // Khởi tạo các giá trị cảm biến hiện tại
  current_gyro_roll_rad_per_sec_ = current_gyro_pitch_rad_per_sec_ = 0;
  current_orientation_roll_rad_ = current_orientation_pitch_rad_ = 0;

  current_right_fx_N_  = current_right_fy_N_  = current_right_fz_N_  = 0;
  current_right_tx_Nm_ = current_right_ty_Nm_ = current_right_tz_Nm_ = 0;
  current_left_fx_N_   = current_left_fy_N_   = current_left_fz_N_   = 0;
  current_left_tx_Nm_  = current_left_ty_Nm_  = current_left_tz_Nm_  = 0;

  // Khởi tạo các giá trị điều chỉnh của chân dựa trên cảm biến và giới hạn cân bằng
  foot_roll_adjustment_by_gyro_roll_ = 0;
  foot_pitch_adjustment_by_gyro_pitch_ = 0;

  foot_roll_adjustment_by_orientation_roll_ = 0;
  foot_pitch_adjustment_by_orientation_pitch_ = 0;

  r_foot_z_adjustment_by_force_z_ = 0;
  l_foot_z_adjustment_by_force_z_ = 0;

  r_foot_x_adjustment_by_force_x_ = 0;
  r_foot_y_adjustment_by_force_y_ = 0;
  r_foot_roll_adjustment_by_torque_roll_ = 0;
  r_foot_pitch_adjustment_by_torque_pitch_ = 0;

  l_foot_x_adjustment_by_force_x_ = 0;
  l_foot_y_adjustment_by_force_y_ = 0;
  l_foot_roll_adjustment_by_torque_roll_ = 0;
  l_foot_pitch_adjustment_by_torque_pitch_ = 0;

  // Khởi tạo điều chỉnh thủ công của COB
  cob_x_manual_adjustment_m_ = 0;
  cob_y_manual_adjustment_m_ = 0;
  cob_z_manual_adjustment_m_ = 0;

  // Khởi tạo giá trị tối đa của việc điều chỉnh
  cob_x_adjustment_abs_max_m_ = 0.05;
  cob_y_adjustment_abs_max_m_ = 0.05;
  cob_z_adjustment_abs_max_m_ = 0.05;
  cob_roll_adjustment_abs_max_rad_  = 30.0 * DEGREE2RADIAN;
  cob_pitch_adjustment_abs_max_rad_ = 30.0 * DEGREE2RADIAN;
  cob_yaw_adjustment_abs_max_rad_   = 30.0 * DEGREE2RADIAN;
  foot_x_adjustment_abs_max_m_ = 0.1;
  foot_y_adjustment_abs_max_m_ = 0.1;
  foot_z_adjustment_abs_max_m_ = 0.1;
  foot_roll_adjustment_abs_max_rad_  = 30.0 * DEGREE2RADIAN;
  foot_pitch_adjustment_abs_max_rad_ = 30.0 * DEGREE2RADIAN;
  foot_yaw_adjustment_abs_max_rad_   = 30.0 * DEGREE2RADIAN;

  // Khởi tạo ma trận biến đổi vị trí được sửa đổi của robot đối với COB và hai chân
  mat_robot_to_cob_modified_        = Eigen::MatrixXd::Identity(4,4);
  mat_robot_to_right_foot_modified_ = Eigen::MatrixXd::Identity(4,4);
  mat_robot_to_left_foot_modified_  = Eigen::MatrixXd::Identity(4,4);

  // Khởi tạo vector điều chỉnh vị trí của COB và hai chân
  pose_cob_adjustment_         = Eigen::VectorXd::Zero(6);
  pose_right_foot_adjustment_  = Eigen::VectorXd::Zero(6);
  pose_left_foot_adjustment_   = Eigen::VectorXd::Zero(6);
}


// Hủy bỏ đối tượng BalanceControlUsingPDController
BalanceControlUsingPDController::~BalanceControlUsingPDController()
{  
  // Hàm hủy không có nội dung, vì không có tài nguyên động cần giải phóng
}

// Khởi tạo các thông số cơ bản cho bộ điều khiển cân bằng PD
void BalanceControlUsingPDController::initialize(const int control_cycle_msec)
{
  // Đặt trạng thái lỗi cân bằng về giá trị mặc định
  balance_control_error_ = BalanceControlError::NoError;

  // Xác định chu kỳ điều khiển theo mili giây
  control_cycle_sec_ = control_cycle_msec * 0.001;

  // Đặt giá trị về 0 cho các điều chỉnh vị trí của COB và chân
  pose_cob_adjustment_.fill(0);
  pose_right_foot_adjustment_.fill(0);
  pose_left_foot_adjustment_.fill(0);

  // Khởi tạo bộ lọc trung bình trượt cho dữ liệu góc Roll của gyro và cảm biến IMU
  roll_gyro_lpf_.initialize(control_cycle_sec_, 1.0);
  pitch_gyro_lpf_.initialize(control_cycle_sec_, 1.0);

  // Khởi tạo bộ lọc trung bình trượt cho góc Roll và Pitch từ cảm biến IMU
  roll_angle_lpf_.initialize(control_cycle_sec_, 1.0);
  pitch_angle_lpf_.initialize(control_cycle_sec_, 1.0);

  // Khởi tạo bộ lọc trung bình trượt cho dữ liệu lực và mômen của chân phải
  right_foot_force_x_lpf_.initialize(control_cycle_sec_, 1.0);
  right_foot_force_y_lpf_.initialize(control_cycle_sec_, 1.0);
  right_foot_force_z_lpf_.initialize(control_cycle_sec_, 1.0);
  right_foot_torque_roll_lpf_.initialize(control_cycle_sec_, 1.0);
  right_foot_torque_pitch_lpf_.initialize(control_cycle_sec_, 1.0);

  // Khởi tạo bộ lọc trung bình trượt cho dữ liệu lực và mômen của chân trái
  left_foot_force_x_lpf_.initialize(control_cycle_sec_, 1.0);
  left_foot_force_y_lpf_.initialize(control_cycle_sec_, 1.0);
  left_foot_force_z_lpf_.initialize(control_cycle_sec_, 1.0);
  left_foot_torque_roll_lpf_.initialize(control_cycle_sec_, 1.0);
  left_foot_torque_pitch_lpf_.initialize(control_cycle_sec_, 1.0);
}


// Bật/tắt tính năng cân bằng dựa trên dữ liệu từ cảm biến gyro
void BalanceControlUsingPDController::setGyroBalanceEnable(bool enable)
{
  // Nếu bật tính năng, thiết lập biến cân bằng gyro_enable_ thành 1.0, ngược lại là 0.0
  gyro_enable_ = enable ? 1.0 : 0.0;
}

// Bật/tắt tính năng cân bằng dựa trên dữ liệu từ cảm biến hướng
void BalanceControlUsingPDController::setOrientationBalanceEnable(bool enable)
{
  // Nếu bật tính năng, thiết lập biến orientation_enable_ thành 1.0, ngược lại là 0.0
  orientation_enable_ = enable ? 1.0 : 0.0;
}

// Bật/tắt tính năng cân bằng dựa trên dữ liệu từ cảm biến lực và mômen
void BalanceControlUsingPDController::setForceTorqueBalanceEnable(bool enable)
{
  // Nếu bật tính năng, thiết lập biến ft_enable_ thành 1.0, ngược lại là 0.0
  ft_enable_ = enable ? 1.0 : 0.0;
}


// Hàm xử lý cân bằng, tính toán điều chỉnh dựa trên dữ liệu cảm biến
void BalanceControlUsingPDController::process(int *balance_error, Eigen::MatrixXd *robot_to_cob_modified, Eigen::MatrixXd *robot_to_right_foot_modified, Eigen::MatrixXd *robot_to_left_foot_modified)
{
  // Khởi tạo trạng thái lỗi cân bằng
  balance_control_error_ = BalanceControlError::NoError;

  // Khởi tạo điều chỉnh của COB và chân
  pose_cob_adjustment_.fill(0);
  pose_right_foot_adjustment_.fill(0);
  pose_left_foot_adjustment_.fill(0);

  // Lấy giá trị đã lọc từ cảm biến
  double roll_gyro_filtered  = roll_gyro_lpf_.getFilteredOutput(current_gyro_roll_rad_per_sec_);
  double pitch_gyro_filtered = pitch_gyro_lpf_.getFilteredOutput(current_gyro_pitch_rad_per_sec_);

  double roll_angle_filtered  = roll_angle_lpf_.getFilteredOutput(current_orientation_roll_rad_);
  double pitch_angle_filtered = pitch_angle_lpf_.getFilteredOutput(current_orientation_pitch_rad_);

  double right_foot_force_x_filtered      = right_foot_force_x_lpf_.getFilteredOutput(current_right_fx_N_);
  double right_foot_force_y_filtered      = right_foot_force_y_lpf_.getFilteredOutput(current_right_fy_N_);
  double right_foot_force_z_filtered      = right_foot_force_z_lpf_.getFilteredOutput(current_right_fz_N_);
  double right_foot_torque_roll_filtered  = right_foot_torque_roll_lpf_.getFilteredOutput(current_right_tx_Nm_);
  double right_foot_torque_pitch_filtered = right_foot_torque_pitch_lpf_.getFilteredOutput(current_right_ty_Nm_);

  double left_foot_force_x_filtered      = left_foot_force_x_lpf_.getFilteredOutput(current_left_fx_N_);
  double left_foot_force_y_filtered      = left_foot_force_y_lpf_.getFilteredOutput(current_left_fy_N_);
  double left_foot_force_z_filtered      = left_foot_force_z_lpf_.getFilteredOutput(current_left_fz_N_);
  double left_foot_torque_roll_filtered  = left_foot_torque_roll_lpf_.getFilteredOutput(current_left_tx_Nm_);
  double left_foot_torque_pitch_filtered = left_foot_torque_pitch_lpf_.getFilteredOutput(current_left_ty_Nm_);

  // Gyro
  foot_roll_adjustment_by_gyro_roll_   = -0.1 * gyro_enable_ * foot_roll_gyro_ctrl_.getFeedBack(roll_gyro_filtered);
  foot_pitch_adjustment_by_gyro_pitch_ = -0.1 * gyro_enable_ * foot_pitch_gyro_ctrl_.getFeedBack(pitch_gyro_filtered);

  // Điều chỉnh z bằng cảm biến hướng
  foot_roll_adjustment_by_orientation_roll_   = -1.0 * orientation_enable_ * foot_roll_angle_ctrl_.getFeedBack(roll_angle_filtered);
  foot_pitch_adjustment_by_orientation_pitch_ = -1.0 * orientation_enable_ * foot_pitch_angle_ctrl_.getFeedBack(pitch_angle_filtered);

  // Ma trận điều chỉnh hướng cảm biến từ IMU
  Eigen::MatrixXd mat_orientation_adjustment_by_imu = robotis_framework::getRotation4d(foot_roll_adjustment_by_gyro_roll_ + foot_roll_adjustment_by_orientation_roll_, foot_pitch_adjustment_by_gyro_pitch_ + foot_pitch_adjustment_by_orientation_pitch_, 0.0);
  Eigen::MatrixXd mat_r_xy, mat_l_xy;
  mat_r_xy.resize(4, 1);
  mat_l_xy.resize(4, 1);
  mat_r_xy.coeffRef(0, 0) = desired_robot_to_right_foot_.coeff(0, 3) - 0.5 * (desired_robot_to_right_foot_.coeff(0, 3) + desired_robot_to_left_foot_.coeff(0, 3));
  mat_r_xy.coeffRef(1, 0) = desired_robot_to_right_foot_.coeff(1, 3) - 0.5 * (desired_robot_to_right_foot_.coeff(1, 3) + desired_robot_to_left_foot_.coeff(1, 3));
  mat_r_xy.coeffRef(2, 0) = 0.0;
  mat_r_xy.coeffRef(3, 0) = 1;

  mat_l_xy.coeffRef(0, 0) = desired_robot_to_left_foot_.coeff(0, 3) - 0.5 * (desired_robot_to_right_foot_.coeff(0, 3) + desired_robot_to_left_foot_.coeff(0, 3));
  mat_l_xy.coeffRef(1, 0) = desired_robot_to_left_foot_.coeff(1, 3) - 0.5 * (desired_robot_to_right_foot_.coeff(1, 3) + desired_robot_to_left_foot_.coeff(1, 3));
  mat_l_xy.coeffRef(2, 0) = 0.0;
  mat_l_xy.coeffRef(3, 0) = 1;

  mat_r_xy = mat_orientation_adjustment_by_imu * mat_r_xy;
  mat_l_xy = mat_orientation_adjustment_by_imu * mat_l_xy;

  // FT sensor
  r_foot_x_adjustment_by_force_x_ = ft_enable_ * 0.001 * right_foot_force_x_ctrl_.getFeedBack(right_foot_force_x_filtered);
  r_foot_y_adjustment_by_force_y_ = ft_enable_ * 0.001 * right_foot_force_y_ctrl_.getFeedBack(right_foot_force_y_filtered);
  r_foot_z_adjustment_by_force_z_ = ft_enable_ * 0.001 * right_foot_force_z_ctrl_.getFeedBack(right_foot_force_z_filtered);
  r_foot_roll_adjustment_by_torque_roll_   = ft_enable_ * right_foot_torque_roll_ctrl_.getFeedBack(right_foot_torque_roll_filtered);
  r_foot_pitch_adjustment_by_torque_pitch_ = ft_enable_ * right_foot_torque_pitch_ctrl_.getFeedBack(right_foot_torque_pitch_filtered);

  l_foot_x_adjustment_by_force_x_ = ft_enable_ * 0.001 * left_foot_force_x_ctrl_.getFeedBack(left_foot_force_x_filtered);
  l_foot_y_adjustment_by_force_y_ = ft_enable_ * 0.001 * left_foot_force_y_ctrl_.getFeedBack(left_foot_force_y_filtered);
  l_foot_z_adjustment_by_force_z_ = ft_enable_ * 0.001 * left_foot_force_z_ctrl_.getFeedBack(left_foot_force_z_filtered);
  l_foot_roll_adjustment_by_torque_roll_   = ft_enable_ * left_foot_torque_roll_ctrl_.getFeedBack(left_foot_torque_roll_filtered);
  l_foot_pitch_adjustment_by_torque_pitch_ = ft_enable_ * left_foot_torque_pitch_ctrl_.getFeedBack(left_foot_torque_pitch_filtered);

// Tổng hợp kết quả cân bằng từ cảm biến
// Thu thập và tổng hợp kết quả cân bằng dựa trên dữ liệu từ các cảm biến

// Các điều chỉnh cho Trung tâm Cơ thể (COB)
pose_cob_adjustment_.coeffRef(0) = cob_x_manual_adjustment_m_;
pose_cob_adjustment_.coeffRef(1) = cob_y_manual_adjustment_m_;
pose_cob_adjustment_.coeffRef(2) = cob_z_manual_adjustment_m_;

// Các điều chỉnh cho tư thế chân phải
pose_right_foot_adjustment_.coeffRef(0) = r_foot_x_adjustment_by_force_x_;
pose_right_foot_adjustment_.coeffRef(1) = r_foot_y_adjustment_by_force_y_;
pose_right_foot_adjustment_.coeffRef(2) = mat_r_xy.coeff(2, 0) + r_foot_z_adjustment_by_force_z_ * 1.0;
pose_right_foot_adjustment_.coeffRef(3) = (foot_roll_adjustment_by_gyro_roll_ + foot_roll_adjustment_by_orientation_roll_ + r_foot_roll_adjustment_by_torque_roll_);
pose_right_foot_adjustment_.coeffRef(4) = (foot_pitch_adjustment_by_gyro_pitch_ + foot_pitch_adjustment_by_orientation_pitch_ + r_foot_pitch_adjustment_by_torque_pitch_);

// Các điều chỉnh cho tư thế chân trái
pose_left_foot_adjustment_.coeffRef(0) = l_foot_x_adjustment_by_force_x_;
pose_left_foot_adjustment_.coeffRef(1) = l_foot_y_adjustment_by_force_y_;
pose_left_foot_adjustment_.coeffRef(2) = mat_l_xy.coeff(2, 0) + l_foot_z_adjustment_by_force_z_ * 1.0;
pose_left_foot_adjustment_.coeffRef(3) = (foot_roll_adjustment_by_gyro_roll_ + foot_roll_adjustment_by_orientation_roll_ + l_foot_roll_adjustment_by_torque_roll_);
pose_left_foot_adjustment_.coeffRef(4) = (foot_pitch_adjustment_by_gyro_pitch_ + foot_pitch_adjustment_by_orientation_pitch_ + l_foot_pitch_adjustment_by_torque_pitch_);

  // Kiểm tra giới hạn và đặt cờ lỗi nếu vượt quá
  if((fabs(pose_cob_adjustment_.coeff(0)) == cob_x_adjustment_abs_max_m_      ) ||
     (fabs(pose_cob_adjustment_.coeff(1)) == cob_y_adjustment_abs_max_m_      ) ||
     (fabs(pose_cob_adjustment_.coeff(2)) == cob_z_adjustment_abs_max_m_      ) ||
     (fabs(pose_cob_adjustment_.coeff(3)) == cob_roll_adjustment_abs_max_rad_ ) ||
     (fabs(pose_cob_adjustment_.coeff(4)) == cob_pitch_adjustment_abs_max_rad_) ||
     (fabs(pose_right_foot_adjustment_.coeff(0)) == foot_x_adjustment_abs_max_m_      ) ||
     (fabs(pose_right_foot_adjustment_.coeff(1)) == foot_y_adjustment_abs_max_m_      ) ||
     (fabs(pose_right_foot_adjustment_.coeff(2)) == foot_z_adjustment_abs_max_m_      ) ||
     (fabs(pose_right_foot_adjustment_.coeff(3)) == foot_roll_adjustment_abs_max_rad_ ) ||
     (fabs(pose_right_foot_adjustment_.coeff(4)) == foot_pitch_adjustment_abs_max_rad_) ||
     (fabs(pose_left_foot_adjustment_.coeff(0)) == foot_x_adjustment_abs_max_m_      ) ||
     (fabs(pose_left_foot_adjustment_.coeff(1)) == foot_y_adjustment_abs_max_m_      ) ||
     (fabs(pose_left_foot_adjustment_.coeff(2)) == foot_z_adjustment_abs_max_m_      ) ||
     (fabs(pose_left_foot_adjustment_.coeff(3)) == foot_roll_adjustment_abs_max_rad_ ) ||
     (fabs(pose_left_foot_adjustment_.coeff(4)) == foot_pitch_adjustment_abs_max_rad_))
    balance_control_error_ &= BalanceControlError::BalanceLimit;


// Áp dụng giới hạn cho các điều chỉnh
  pose_cob_adjustment_.coeffRef(0) = copysign(fmin(fabs(pose_cob_adjustment_.coeff(0)), cob_x_adjustment_abs_max_m_      ), pose_cob_adjustment_.coeff(0));
  pose_cob_adjustment_.coeffRef(1) = copysign(fmin(fabs(pose_cob_adjustment_.coeff(1)), cob_x_adjustment_abs_max_m_      ), pose_cob_adjustment_.coeff(1));
  pose_cob_adjustment_.coeffRef(2) = copysign(fmin(fabs(pose_cob_adjustment_.coeff(2)), cob_x_adjustment_abs_max_m_      ), pose_cob_adjustment_.coeff(2));
  pose_cob_adjustment_.coeffRef(3) = copysign(fmin(fabs(pose_cob_adjustment_.coeff(3)), cob_roll_adjustment_abs_max_rad_ ), pose_cob_adjustment_.coeff(3));
  pose_cob_adjustment_.coeffRef(4) = copysign(fmin(fabs(pose_cob_adjustment_.coeff(4)), cob_pitch_adjustment_abs_max_rad_), pose_cob_adjustment_.coeff(4));
  pose_cob_adjustment_.coeffRef(5) = 0;

  pose_right_foot_adjustment_.coeffRef(0) = copysign(fmin(fabs(pose_right_foot_adjustment_.coeff(0)), foot_x_adjustment_abs_max_m_      ), pose_right_foot_adjustment_.coeff(0));
  pose_right_foot_adjustment_.coeffRef(1) = copysign(fmin(fabs(pose_right_foot_adjustment_.coeff(1)), foot_y_adjustment_abs_max_m_      ), pose_right_foot_adjustment_.coeff(1));
  pose_right_foot_adjustment_.coeffRef(2) = copysign(fmin(fabs(pose_right_foot_adjustment_.coeff(2)), foot_z_adjustment_abs_max_m_      ), pose_right_foot_adjustment_.coeff(2));
  pose_right_foot_adjustment_.coeffRef(3) = copysign(fmin(fabs(pose_right_foot_adjustment_.coeff(3)), foot_roll_adjustment_abs_max_rad_ ), pose_right_foot_adjustment_.coeff(3));
  pose_right_foot_adjustment_.coeffRef(4) = copysign(fmin(fabs(pose_right_foot_adjustment_.coeff(4)), foot_pitch_adjustment_abs_max_rad_), pose_right_foot_adjustment_.coeff(4));
  pose_right_foot_adjustment_.coeffRef(5) = 0;

  pose_left_foot_adjustment_.coeffRef(0) = copysign(fmin(fabs(pose_left_foot_adjustment_.coeff(0)), foot_x_adjustment_abs_max_m_      ), pose_left_foot_adjustment_.coeff(0));
  pose_left_foot_adjustment_.coeffRef(1) = copysign(fmin(fabs(pose_left_foot_adjustment_.coeff(1)), foot_y_adjustment_abs_max_m_      ), pose_left_foot_adjustment_.coeff(1));
  pose_left_foot_adjustment_.coeffRef(2) = copysign(fmin(fabs(pose_left_foot_adjustment_.coeff(2)), foot_z_adjustment_abs_max_m_      ), pose_left_foot_adjustment_.coeff(2));
  pose_left_foot_adjustment_.coeffRef(3) = copysign(fmin(fabs(pose_left_foot_adjustment_.coeff(3)), foot_roll_adjustment_abs_max_rad_ ), pose_left_foot_adjustment_.coeff(3));
  pose_left_foot_adjustment_.coeffRef(4) = copysign(fmin(fabs(pose_left_foot_adjustment_.coeff(4)), foot_pitch_adjustment_abs_max_rad_), pose_left_foot_adjustment_.coeff(4));
  pose_left_foot_adjustment_.coeffRef(5) = 0;

  // Tính toán ma trận xoay cho COB và điều chỉnh chân
Eigen::MatrixXd cob_rotation_adj = robotis_framework::getRotationZ(pose_cob_adjustment_.coeff(5)) * robotis_framework::getRotationY(pose_cob_adjustment_.coeff(4)) * robotis_framework::getRotationX(pose_cob_adjustment_.coeff(3));
Eigen::MatrixXd rf_rotation_adj = robotis_framework::getRotationZ(pose_right_foot_adjustment_.coeff(5)) * robotis_framework::getRotationY(pose_right_foot_adjustment_.coeff(4)) * robotis_framework::getRotationX(pose_right_foot_adjustment_.coeff(3));
Eigen::MatrixXd lf_rotation_adj = robotis_framework::getRotationZ(pose_left_foot_adjustment_.coeff(5)) * robotis_framework::getRotationY(pose_left_foot_adjustment_.coeff(4)) * robotis_framework::getRotationX(pose_left_foot_adjustment_.coeff(3));

// Cập nhật ma trận đã điều chỉnh cho COB và chân
mat_robot_to_cob_modified_.block<3, 3>(0, 0) = cob_rotation_adj * desired_robot_to_cob_.block<3, 3>(0, 0);
mat_robot_to_right_foot_modified_.block<3, 3>(0, 0) = rf_rotation_adj * desired_robot_to_right_foot_.block<3, 3>(0, 0);
mat_robot_to_left_foot_modified_.block<3, 3>(0, 0) = lf_rotation_adj * desired_robot_to_left_foot_.block<3, 3>(0, 0);

// Cập nhật thành phần dịch chuyển của ma trận đã điều chỉnh
mat_robot_to_cob_modified_.coeffRef(0, 3) = desired_robot_to_cob_.coeff(0, 3) + pose_cob_adjustment_.coeff(0);
mat_robot_to_cob_modified_.coeffRef(1, 3) = desired_robot_to_cob_.coeff(1, 3) + pose_cob_adjustment_.coeff(1);
mat_robot_to_cob_modified_.coeffRef(2, 3) = desired_robot_to_cob_.coeff(2, 3) + pose_cob_adjustment_.coeff(2);

mat_robot_to_right_foot_modified_.coeffRef(0, 3) = desired_robot_to_right_foot_.coeff(0, 3) + pose_right_foot_adjustment_.coeff(0);
mat_robot_to_right_foot_modified_.coeffRef(1, 3) = desired_robot_to_right_foot_.coeff(1, 3) + pose_right_foot_adjustment_.coeff(1);
mat_robot_to_right_foot_modified_.coeffRef(2, 3) = desired_robot_to_right_foot_.coeff(2, 3) + pose_right_foot_adjustment_.coeff(2);

mat_robot_to_left_foot_modified_.coeffRef(0, 3) = desired_robot_to_left_foot_.coeff(0, 3) + pose_left_foot_adjustment_.coeff(0);
mat_robot_to_left_foot_modified_.coeffRef(1, 3) = desired_robot_to_left_foot_.coeff(1, 3) + pose_left_foot_adjustment_.coeff(1);
mat_robot_to_left_foot_modified_.coeffRef(2, 3) = desired_robot_to_left_foot_.coeff(2, 3) + pose_left_foot_adjustment_.coeff(2);

// Cập nhật lỗi cân bằng nếu có
if (balance_error != 0)
    *balance_error = balance_control_error_;

// Gán ma trận đã điều chỉnh cho các con trỏ đầu ra
*robot_to_cob_modified = mat_robot_to_cob_modified_;
*robot_to_right_foot_modified = mat_robot_to_right_foot_modified_;
*robot_to_left_foot_modified = mat_robot_to_left_foot_modified_;
}


// Thiết lập tư thế mong muốn cho robot
void BalanceControlUsingPDController::setDesiredPose(const Eigen::MatrixXd &robot_to_cob, const Eigen::MatrixXd &robot_to_right_foot, const Eigen::MatrixXd &robot_to_left_foot)
{
  // Lưu trữ tư thế mong muốn cho trung tâm cơ thể, chân phải và chân trái
  desired_robot_to_cob_        = robot_to_cob;
  desired_robot_to_right_foot_ = robot_to_right_foot;
  desired_robot_to_left_foot_  = robot_to_left_foot;
}

// Đặt giá trị mong muốn cho góc quay của trung tâm cơ thể từ giroscope
void BalanceControlUsingPDController::setDesiredCOBGyro(double gyro_roll, double gyro_pitch)
{
  // Gán giá trị mong muốn cho góc quay roll và pitch
  foot_roll_gyro_ctrl_.desired_  = gyro_roll;
  foot_pitch_gyro_ctrl_.desired_ = gyro_pitch;
}

// Đặt giá trị mong muốn cho góc quay của trung tâm cơ thể từ dữ liệu cảm biến
void BalanceControlUsingPDController::setDesiredCOBOrientation(double cob_orientation_roll, double cob_orientation_pitch)
{
  // Gán giá trị mong muốn cho góc quay roll và pitch của trung tâm cơ thể
  foot_roll_angle_ctrl_.desired_  = cob_orientation_roll;
  foot_pitch_angle_ctrl_.desired_ = cob_orientation_pitch;
}

// Đặt giá trị mong muốn cho lực và moment tác động lên chân phải và chân trái
void BalanceControlUsingPDController::setDesiredFootForceTorque(double r_force_x_N, double r_force_y_N, double r_force_z_N,
                                                                double r_torque_roll_Nm, double r_torque_pitch_Nm, double r_torque_yaw_Nm,
                                                                double l_force_x_N, double l_force_y_N, double l_force_z_N,
                                                                double l_torque_roll_Nm, double l_torque_pitch_Nm, double l_torque_yaw_Nm)
{
  // Gán giá trị mong muốn cho lực và moment tác động lên chân phải
  right_foot_force_x_ctrl_.desired_      = r_force_x_N;
  right_foot_force_y_ctrl_.desired_      = r_force_y_N;
  right_foot_force_z_ctrl_.desired_      = r_force_z_N;
  right_foot_torque_roll_ctrl_.desired_  = r_torque_roll_Nm;
  right_foot_torque_pitch_ctrl_.desired_ = r_torque_pitch_Nm;

  // Gán giá trị mong muốn cho lực và moment tác động lên chân trái
  left_foot_force_x_ctrl_.desired_      = l_force_x_N;
  left_foot_force_y_ctrl_.desired_      = l_force_y_N;
  left_foot_force_z_ctrl_.desired_      = l_force_z_N;
  left_foot_torque_roll_ctrl_.desired_  = l_torque_roll_Nm;
  left_foot_torque_pitch_ctrl_.desired_ = l_torque_pitch_Nm;
}


// Cập nhật dữ liệu đầu ra từ cảm biến giroscope
void BalanceControlUsingPDController::setCurrentGyroSensorOutput(double gyro_roll, double gyro_pitch)
{
  // Lưu trữ dữ liệu giroscope hiện tại về góc quay
  current_gyro_roll_rad_per_sec_  = gyro_roll;
  current_gyro_pitch_rad_per_sec_ = gyro_pitch;
}

// Cập nhật dữ liệu đầu ra từ cảm biến góc quay của trung tâm cơ thể
void BalanceControlUsingPDController::setCurrentOrientationSensorOutput(double cob_orientation_roll, double cob_orientation_pitch)
{
  // Lưu trữ dữ liệu hiện tại về góc quay của trung tâm cơ thể
  current_orientation_roll_rad_  = cob_orientation_roll;
  current_orientation_pitch_rad_ = cob_orientation_pitch;
}

// Cập nhật dữ liệu đầu ra từ cảm biến lực và moment tác động lên chân phải và chân trái
void BalanceControlUsingPDController::setCurrentFootForceTorqueSensorOutput(double r_force_x_N, double r_force_y_N, double r_force_z_N,
                                                           double r_torque_roll_Nm, double r_torque_pitch_Nm, double r_torque_yaw_Nm,
                                                           double l_force_x_N, double l_force_y_N, double l_force_z_N,
                                                           double l_torque_roll_Nm, double l_torque_pitch_Nm, double l_torque_yaw_Nm)
{
  // Lưu trữ dữ liệu hiện tại về lực và moment tác động lên chân phải
  current_right_fx_N_  = r_force_x_N;
  current_right_fy_N_  = r_force_y_N;
  current_right_fz_N_  = r_force_z_N;
  current_right_tx_Nm_ = r_torque_roll_Nm;
  current_right_ty_Nm_ = r_torque_pitch_Nm;
  current_right_tz_Nm_ = r_torque_yaw_Nm;

  // Lưu trữ dữ liệu hiện tại về lực và moment tác động lên chân trái
  current_left_fx_N_  = l_force_x_N;
  current_left_fy_N_  = l_force_y_N;
  current_left_fz_N_  = l_force_z_N;
  current_left_tx_Nm_ = l_torque_roll_Nm;
  current_left_ty_Nm_ = l_torque_pitch_Nm;
  current_left_tz_Nm_ = l_torque_yaw_Nm;
}

// Thiết lập giá trị tối đa cho việc điều chỉnh
void BalanceControlUsingPDController::setMaximumAdjustment(double cob_x_max_adjustment_m,  double cob_y_max_adjustment_m,  double cob_z_max_adjustment_m,
                                          double cob_roll_max_adjustment_rad, double cob_pitch_max_adjustment_rad, double cob_yaw_max_adjustment_rad,
                                          double foot_x_max_adjustment_m, double foot_y_max_adjustment_m, double foot_z_max_adjustment_m,
                                          double foot_roll_max_adjustment_rad, double foot_pitch_max_adjustment_rad, double foot_yaw_max_adjustment_rad)
{
  // Lưu trữ giá trị tối đa cho điều chỉnh của trung tâm cơ thể
  cob_x_adjustment_abs_max_m_        = cob_x_max_adjustment_m;
  cob_y_adjustment_abs_max_m_        = cob_y_max_adjustment_m;
  cob_z_adjustment_abs_max_m_        = cob_z_max_adjustment_m;
  cob_roll_adjustment_abs_max_rad_   = cob_roll_max_adjustment_rad;
  cob_pitch_adjustment_abs_max_rad_  = cob_pitch_max_adjustment_rad;
  cob_yaw_adjustment_abs_max_rad_    = cob_yaw_max_adjustment_rad;

  // Lưu trữ giá trị tối đa cho điều chỉnh của chân phải và chân trái
  foot_x_adjustment_abs_max_m_       = foot_x_max_adjustment_m;
  foot_y_adjustment_abs_max_m_       = foot_y_max_adjustment_m;
  foot_z_adjustment_abs_max_m_       = foot_z_max_adjustment_m;
  foot_roll_adjustment_abs_max_rad_  = foot_roll_max_adjustment_rad;
  foot_pitch_adjustment_abs_max_rad_ = foot_pitch_max_adjustment_rad;
  foot_yaw_adjustment_abs_max_rad_   = foot_yaw_max_adjustment_rad;
}

// Đặt giá trị điều chỉnh thủ công cho trung tâm cơ thể
void BalanceControlUsingPDController::setCOBManualAdjustment(double cob_x_adjustment_m, double cob_y_adjustment_m, double cob_z_adjustment_m)
{
  // Lưu trữ giá trị điều chỉnh thủ công cho trung tâm cơ thể
  cob_x_manual_adjustment_m_ = cob_x_adjustment_m;
  cob_y_manual_adjustment_m_ = cob_y_adjustment_m;
  cob_z_manual_adjustment_m_ = cob_z_adjustment_m;
}

// Lấy giá trị điều chỉnh thủ công theo trục X cho trung tâm cơ thể
double BalanceControlUsingPDController::getCOBManualAdjustmentX()
{
  return cob_x_manual_adjustment_m_;
}

// Lấy giá trị điều chỉnh thủ công theo trục Y cho trung tâm cơ thể
double BalanceControlUsingPDController::getCOBManualAdjustmentY()
{
  return cob_y_manual_adjustment_m_;
}

// Lấy giá trị điều chỉnh thủ công theo trục Z cho trung tâm cơ thể
double BalanceControlUsingPDController::getCOBManualAdjustmentZ()
{
  return cob_z_manual_adjustment_m_;
}


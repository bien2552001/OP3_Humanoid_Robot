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

#ifndef OP3_BALANCE_CONTROL_OP3_BALANCE_CONTROL_H_
#define OP3_BALANCE_CONTROL_OP3_BALANCE_CONTROL_H_

#include <eigen3/Eigen/Eigen>

#include "robotis_math/robotis_math.h"

namespace robotis_op
{

// ---------------------------------Lớp định nghĩa các mã lỗi cho kiểm soát cân bằng
class BalanceControlError
{
public:
  // Mã lỗi không có
  static const int NoError = 0;

  // Mã lỗi chỉ ra rằng đã đạt đến giới hạn cân bằng
  static const int BalanceLimit = 2;
};



// ---------------------------------Lớp đại diện cho Bộ Điều Khiển Damping
class DampingController
{
public:
  // Constructor mặc định
  DampingController();

  // Constructor với tham số thời gian
  DampingController(double time_unit_sec);

  // Destructor
  ~DampingController();

  // Phương thức tính toán đầu ra của Bộ Điều Khiển Damping dựa trên đầu ra cảm biến hiện tại
  double getDampingControllerOutput(double present_sensor_output);

  // Giá trị mong muốn cho bộ điều khiển
  double desired_;

  // Tham số tăng cho bộ điều khiển
  double gain_;

  // Tham số hằng thời gian cho bộ điều khiển
  double time_constant_sec_;

  // Giá trị đầu ra của bộ điều khiển
  double output_;

  // Thời gian chu kỳ kiểm soát cho bộ điều khiển
  double control_cycle_sec_;

private:
  // Kết quả trước đó được sử dụng nội bộ trong bộ điều khiển
  double previous_result_;
};


// ---------------------------------Lớp đại diện cho Bộ Điều Khiển PD Cân Bằng
class BalancePDController
{
public:
  // Constructor mặc định
  BalancePDController();

  // Destructor
  ~BalancePDController();

  // Giá trị mong muốn cho bộ điều khiển
  double desired_;

  // Hệ số tăng P cho bộ điều khiển
  double p_gain_;

  // Hệ số tăng D cho bộ điều khiển
  double d_gain_;

  // Phương thức để nhận phản hồi từ cảm biến hiện tại
  double getFeedBack(double present_sensor_output);

private:
  // Lỗi hiện tại
  double curr_err_;

  // Lỗi trước đó
  double prev_err_;
};




// ---------------------------------Lớp đại diện cho Bộ Lọc Thấp Điều Khiển Cân Bằng
class BalanceLowPassFilter
{
public:
  // Constructor mặc định
  BalanceLowPassFilter();

  // Constructor với tham số thời gian kiểm soát và tần số cắt
  BalanceLowPassFilter(double control_cycle_sec, double cut_off_frequency);

  // Destructor
  ~BalanceLowPassFilter();

  // Phương thức để khởi tạo lớp với các giá trị thời gian kiểm soát và tần số cắt mới
  void initialize(double control_cycle_sec_, double cut_off_frequency);

  // Phương thức để đặt giá trị tần số cắt mới
  void setCutOffFrequency(double cut_off_frequency);

  // Phương thức để nhận giá trị tần số cắt hiện tại
  double getCutOffFrequency(void);

  // Phương thức để nhận giá trị được lọc dựa trên giá trị nguyên bản hiện tại
  double getFilteredOutput(double present_raw_value);

private:
  // Giá trị tần số cắt
  double cut_off_freq_;

  // Thời gian kiểm soát
  double control_cycle_sec_;

  // Hệ số alpha được tính từ tần số cắt và thời gian kiểm soát
  double alpha_;

  // Giá trị đầu ra trước đó
  double prev_output_;
};



// ---------------------------------Lớp đại diện cho Kiểm Soát Cân Bằng sử dụng Bộ Điều Khiển Damping
class BalanceControlUsingDampingConroller
{
public:
  // Constructor mặc định
  BalanceControlUsingDampingConroller();

  // Destructor
  ~BalanceControlUsingDampingConroller();

  // Phương thức để khởi tạo với chu kỳ kiểm soát
  void initialize(const int control_cycle_msec);

  // Các phương thức để kích hoạt hoặc vô hiệu hóa kiểm soát cân bằng cho giảm xóc giữa các thành phần
  void setGyroBalanceEnable(bool enable);
  void setOrientationBalanceEnable(bool enable);
  void setForceTorqueBalanceEnable(bool enable);

  // Phương thức để xử lý quá trình kiểm soát cân bằng dựa trên các thông số đầu vào
  void process(int *balance_error, Eigen::MatrixXd *robot_to_cob_modified, Eigen::MatrixXd *robot_to_right_foot_modified, Eigen::MatrixXd *robot_to_left_foot_modified);

  // Phương thức để đặt giá trị mong muốn cho các vị trí cơ bản của robot
  void setDesiredPose(const Eigen::MatrixXd &robot_to_cob, const Eigen::MatrixXd &robot_to_right_foot, const Eigen::MatrixXd &robot_to_left_foot);

  // Phương thức để đặt giá trị mong muốn cho cảm biến giro
  void setDesiredCOBGyro(double gyro_roll, double gyro_pitch);

  // Phương thức để đặt giá trị mong muốn cho góc xoay của COB
  void setDesiredCOBOrientation(double cob_orientation_roll, double cob_orientation_pitch);

  // Phương thức để đặt giá trị mong muốn cho lực và mô-men xoắn của chân
  void setDesiredFootForceTorque(double r_force_x_N, double r_force_y_N, double r_force_z_N,
                                 double r_torque_roll_Nm, double r_torque_pitch_Nm, double r_torque_yaw_Nm,
                                 double l_force_x_N, double l_force_y_N, double l_force_z_N,
                                 double l_torque_roll_Nm, double l_torque_pitch_Nm, double l_torque_yaw_Nm);

  // Phương thức để đặt giá trị hiện tại của cảm biến giro
  void setCurrentGyroSensorOutput(double gyro_roll, double gyro_pitch);

  // Phương thức để đặt giá trị hiện tại của cảm biến xoay của COB
  void setCurrentOrientationSensorOutput(double cob_orientation_roll, double cob_orientation_pitch);

  // Phương thức để đặt giá trị hiện tại của cảm biến lực và mô-men xoắn của chân
  void setCurrentFootForceTorqueSensorOutput(double r_force_x_N, double r_force_y_N, double r_force_z_N,
                                             double r_torque_roll_Nm, double r_torque_pitch_Nm, double r_torque_yaw_Nm,
                                             double l_force_x_N, double l_force_y_N, double l_force_z_N,
                                             double l_torque_roll_Nm, double l_torque_pitch_Nm, double l_torque_yaw_Nm);

  // Phương thức để đặt giá trị điều chỉnh tối đa
  void setMaximumAdjustment(double cob_x_max_adjustment_m, double cob_y_max_adjustment_m, double cob_z_max_adjustment_m,
                            double cob_roll_max_adjustment_rad, double cob_pitch_max_adjustment_rad, double cob_yaw_max_adjustment_rad,
                            double foot_x_max_adjustment_m, double foot_y_max_adjustment_m, double foot_z_max_adjustment_m,
                            double foot_roll_max_adjustment_rad, double foot_pitch_max_adjustment_rad, double foot_yaw_max_adjustment_rad);

  // Phương thức để đặt giá trị điều chỉnh thủ công cho COB
  void setCOBManualAdjustment(double cob_x_adjustment_m, double cob_y_adjustment_m, double cob_z_adjustment_m);
  double getCOBManualAdjustmentX();
  double getCOBManualAdjustmentY();
  double getCOBManualAdjustmentZ();

  // Phương thức để đặt hệ số tăng cân bằng giro
  void setGyroBalanceGainRatio(double gyro_balance_gain_ratio);
  double getGyroBalanceGainRatio(void);

  // Bộ điều khiển Damping
  DampingController foot_roll_angle_ctrl_;
  DampingController foot_pitch_angle_ctrl_;

  DampingController foot_force_z_diff_ctrl_;
  DampingController right_foot_force_z_ctrl_;
  DampingController left_foot_force_z_ctrl_;

  DampingController right_foot_force_x_ctrl_;
  DampingController right_foot_force_y_ctrl_;
  DampingController right_foot_torque_roll_ctrl_;
  DampingController right_foot_torque_pitch_ctrl_;

  DampingController left_foot_force_x_ctrl_;
  DampingController left_foot_force_y_ctrl_;
  DampingController left_foot_torque_roll_ctrl_;
  DampingController left_foot_torque_pitch_ctrl_;

private:
  int balance_control_error_;
  double control_cycle_sec_;

  // Kích hoạt kiểm soát cân bằng cho từng thành phần
  double gyro_enable_;
  double orientation_enable_;
  double ft_enable_;

  // Giá trị mong muốn
  Eigen::MatrixXd desired_robot_to_cob_;
  Eigen::MatrixXd desired_robot_to_right_foot_;
  Eigen::MatrixXd desired_robot_to_left_foot_;

  // Cân bằng giro
  double gyro_balance_gain_ratio_;
  double gyro_balance_roll_gain_;
  double gyro_balance_pitch_gain_;
  double gyro_cut_off_freq_;
  double gyro_lpf_alpha_;
  double gyro_roll_filtered_, gyro_pitch_filtered_;
  double desired_gyro_roll_, desired_gyro_pitch_;

  // Giá trị cảm biến hiện tại
  double current_gyro_roll_rad_per_sec_, current_gyro_pitch_rad_per_sec_;
  double current_orientation_roll_rad_, current_orientation_pitch_rad_;
  double current_right_fx_N_, current_right_fy_N_, current_right_fz_N_;
  double current_right_tx_Nm_, current_right_ty_Nm_, current_right_tz_Nm_;
  double current_left_fx_N_, current_left_fy_N_, current_left_fz_N_;
  double current_left_tx_Nm_, current_left_ty_Nm_, current_left_tz_Nm_;

  // Điều chỉnh COB thủ công
  double cob_x_manual_adjustment_m_;
  double cob_y_manual_adjustment_m_;
  double cob_z_manual_adjustment_m_;

  // Kết quả kiểm soát cân bằng
  double foot_roll_adjustment_by_gyro_roll_;
  double foot_pitch_adjustment_by_gyro_pitch_;
  double foot_roll_adjustment_by_orientation_roll_;
  double foot_pitch_adjustment_by_orientation_pitch_;
  double foot_z_adjustment_by_force_z_difference_;
  double r_foot_z_adjustment_by_force_z_;
  double l_foot_z_adjustment_by_force_z_;
  double r_foot_x_adjustment_by_force_x_;
  double r_foot_y_adjustment_by_force_y_;
  double r_foot_roll_adjustment_by_torque_roll_;
  double r_foot_pitch_adjustment_by_torque_pitch_;
  double l_foot_x_adjustment_by_force_x_;
  double l_foot_y_adjustment_by_force_y_;
  double l_foot_roll_adjustment_by_torque_roll_;
  double l_foot_pitch_adjustment_by_torque_pitch_;

  // Tổng kết quả kiểm soát cân bằng
  Eigen::VectorXd pose_cob_adjustment_;
  Eigen::VectorXd pose_right_foot_adjustment_;
  Eigen::VectorXd pose_left_foot_adjustment_;

  Eigen::MatrixXd mat_robot_to_cob_modified_;
  Eigen::MatrixXd mat_robot_to_right_foot_modified_;
  Eigen::MatrixXd mat_robot_to_left_foot_modified_;

  // Điều chỉnh tối đa
  double cob_x_adjustment_abs_max_m_;
  double cob_y_adjustment_abs_max_m_;
  double cob_z_adjustment_abs_max_m_;
  double cob_roll_adjustment_abs_max_rad_;
  double cob_pitch_adjustment_abs_max_rad_;
  double cob_yaw_adjustment_abs_max_rad_;

  double foot_x_adjustment_abs_max_m_;
  double foot_y_adjustment_abs_max_m_;
  double foot_z_adjustment_abs_max_m_;
  double foot_roll_adjustment_abs_max_rad_;
  double foot_pitch_adjustment_abs_max_rad_;
  double foot_yaw_adjustment_abs_max_rad_;
};


// ---------------------------------Lớp đại diện cho Kiểm Soát Cân Bằng sử dụng Bộ Điều Khiển PD (Proportional-Derivative)
class BalanceControlUsingPDController
{
public:
  // Constructor mặc định
  BalanceControlUsingPDController();

  // Destructor
  ~BalanceControlUsingPDController();

  // Phương thức để khởi tạo với chu kỳ kiểm soát
  void initialize(const int control_cycle_msec);

  // Các phương thức để kích hoạt hoặc vô hiệu hóa kiểm soát cân bằng cho giảm xóc giữa các thành phần
  void setGyroBalanceEnable(bool enable);
  void setOrientationBalanceEnable(bool enable);
  void setForceTorqueBalanceEnable(bool enable);

  // Phương thức để xử lý quá trình kiểm soát cân bằng dựa trên các thông số đầu vào
  void process(int *balance_error, Eigen::MatrixXd *robot_to_cob_modified, Eigen::MatrixXd *robot_to_right_foot_modified, Eigen::MatrixXd *robot_to_left_foot_modified);

  // Phương thức để đặt giá trị mong muốn cho các vị trí cơ bản của robot
  void setDesiredPose(const Eigen::MatrixXd &robot_to_cob, const Eigen::MatrixXd &robot_to_right_foot, const Eigen::MatrixXd &robot_to_left_foot);

  // Phương thức để đặt giá trị mong muốn cho cảm biến giro
  void setDesiredCOBGyro(double gyro_roll, double gyro_pitch);

  // Phương thức để đặt giá trị mong muốn cho góc xoay của COB
  void setDesiredCOBOrientation(double cob_orientation_roll, double cob_orientation_pitch);

  // Phương thức để đặt giá trị mong muốn cho lực và mô-men xoắn của chân
  void setDesiredFootForceTorque(double r_force_x_N, double r_force_y_N, double r_force_z_N,
                                 double r_torque_roll_Nm, double r_torque_pitch_Nm, double r_torque_yaw_Nm,
                                 double l_force_x_N, double l_force_y_N, double l_force_z_N,
                                 double l_torque_roll_Nm, double l_torque_pitch_Nm, double l_torque_yaw_Nm);

  // Phương thức để đặt giá trị hiện tại của cảm biến giro
  void setCurrentGyroSensorOutput(double gyro_roll, double gyro_pitch);

  // Phương thức để đặt giá trị hiện tại của cảm biến xoay của COB
  void setCurrentOrientationSensorOutput(double cob_orientation_roll, double cob_orientation_pitch);

  // Phương thức để đặt giá trị hiện tại của cảm biến lực và mô-men xoắn của chân
  void setCurrentFootForceTorqueSensorOutput(double r_force_x_N, double r_force_y_N, double r_force_z_N,
                                             double r_torque_roll_Nm, double r_torque_pitch_Nm, double r_torque_yaw_Nm,
                                             double l_force_x_N, double l_force_y_N, double l_force_z_N,
                                             double l_torque_roll_Nm, double l_torque_pitch_Nm, double l_torque_yaw_Nm);

  // Phương thức để đặt giá trị điều chỉnh tối đa
  void setMaximumAdjustment(double cob_x_max_adjustment_m,  double cob_y_max_adjustment_m,  double cob_z_max_adjustment_m,
                            double cob_roll_max_adjustment_rad, double cob_pitch_max_adjustment_rad, double cob_yaw_max_adjustment_rad,
                            double foot_x_max_adjustment_m, double foot_y_max_adjustment_m, double foot_z_max_adjustment_m,
                            double foot_roll_max_adjustment_rad, double foot_pitch_max_adjustment_rad, double foot_yaw_max_adjustment_rad);

  // Phương thức để đặt giá trị điều chỉnh thủ công cho COB
  void setCOBManualAdjustment(double cob_x_adjustment_m, double cob_y_adjustment_m, double cob_z_adjustment_m);
  double getCOBManualAdjustmentX();
  double getCOBManualAdjustmentY();
  double getCOBManualAdjustmentZ();

  // Bộ điều khiển PD (Proportional-Derivative)
  BalancePDController foot_roll_gyro_ctrl_;
  BalancePDController foot_pitch_gyro_ctrl_;
  BalancePDController foot_roll_angle_ctrl_;
  BalancePDController foot_pitch_angle_ctrl_;

  BalancePDController right_foot_force_z_ctrl_;
  BalancePDController left_foot_force_z_ctrl_;

  BalancePDController right_foot_force_x_ctrl_;
  BalancePDController right_foot_force_y_ctrl_;
  BalancePDController right_foot_torque_roll_ctrl_;
  BalancePDController right_foot_torque_pitch_ctrl_;

  BalancePDController left_foot_force_x_ctrl_;
  BalancePDController left_foot_force_y_ctrl_;
  BalancePDController left_foot_torque_roll_ctrl_;
  BalancePDController left_foot_torque_pitch_ctrl_;

  // Bộ lọc thấp (Low Pass Filter) cho cảm biến giro và cảm biến lực và mô-men xoắn của chân
  BalanceLowPassFilter roll_gyro_lpf_;
  BalanceLowPassFilter pitch_gyro_lpf_;

  BalanceLowPassFilter roll_angle_lpf_;
  BalanceLowPassFilter pitch_angle_lpf_;

  BalanceLowPassFilter right_foot_force_x_lpf_;
  BalanceLowPassFilter right_foot_force_y_lpf_;
  BalanceLowPassFilter right_foot_force_z_lpf_;
  BalanceLowPassFilter right_foot_torque_roll_lpf_;
  BalanceLowPassFilter right_foot_torque_pitch_lpf_;

  BalanceLowPassFilter left_foot_force_x_lpf_;
  BalanceLowPassFilter left_foot_force_y_lpf_;
  BalanceLowPassFilter left_foot_force_z_lpf_;
  BalanceLowPassFilter left_foot_torque_roll_lpf_;
  BalanceLowPassFilter left_foot_torque_pitch_lpf_;

private:
  int balance_control_error_;
  double control_cycle_sec_;

  // Kích hoạt kiểm soát cân bằng cho từng thành phần
  double gyro_enable_;
  double orientation_enable_;
  double ft_enable_;

  // Giá trị mong muốn
  Eigen::MatrixXd desired_robot_to_cob_;
  Eigen::MatrixXd desired_robot_to_right_foot_;
  Eigen::MatrixXd desired_robot_to_left_foot_;

  // Giá trị cảm biến hiện tại
  double current_gyro_roll_rad_per_sec_, current_gyro_pitch_rad_per_sec_;
  double current_orientation_roll_rad_, current_orientation_pitch_rad_;
  double current_right_fx_N_, current_right_fy_N_, current_right_fz_N_;
  double current_right_tx_Nm_, current_right_ty_Nm_, current_right_tz_Nm_;
  double current_left_fx_N_, current_left_fy_N_, current_left_fz_N_;
  double current_left_tx_Nm_, current_left_ty_Nm_, current_left_tz_Nm_;

  // Điều chỉnh COB thủ công
  double cob_x_manual_adjustment_m_;
  double cob_y_manual_adjustment_m_;
  double cob_z_manual_adjustment_m_;

  // Kết quả kiểm soát cân bằng
  double foot_roll_adjustment_by_gyro_roll_;
  double foot_pitch_adjustment_by_gyro_pitch_;

  double foot_roll_adjustment_by_orientation_roll_;
  double foot_pitch_adjustment_by_orientation_pitch_;

  double r_foot_z_adjustment_by_force_z_;
  double l_foot_z_adjustment_by_force_z_;

  double r_foot_x_adjustment_by_force_x_;
  double r_foot_y_adjustment_by_force_y_;
  double r_foot_roll_adjustment_by_torque_roll_;
  double r_foot_pitch_adjustment_by_torque_pitch_;

  double l_foot_x_adjustment_by_force_x_;
  double l_foot_y_adjustment_by_force_y_;
  double l_foot_roll_adjustment_by_torque_roll_;
  double l_foot_pitch_adjustment_by_torque_pitch_;

  // Tổng kết quả kiểm soát cân bằng
  Eigen::VectorXd pose_cob_adjustment_;
  Eigen::VectorXd pose_right_foot_adjustment_;
  Eigen::VectorXd pose_left_foot_adjustment_;

  Eigen::MatrixXd mat_robot_to_cob_modified_;
  Eigen::MatrixXd mat_robot_to_right_foot_modified_;
  Eigen::MatrixXd mat_robot_to_left_foot_modified_;

  // Điều chỉnh tối đa
  double cob_x_adjustment_abs_max_m_;
  double cob_y_adjustment_abs_max_m_;
  double cob_z_adjustment_abs_max_m_;
  double cob_roll_adjustment_abs_max_rad_;
  double cob_pitch_adjustment_abs_max_rad_;
  double cob_yaw_adjustment_abs_max_rad_;

  double foot_x_adjustment_abs_max_m_;
  double foot_y_adjustment_abs_max_m_;
  double foot_z_adjustment_abs_max_m_;
  double foot_roll_adjustment_abs_max_rad_;
  double foot_pitch_adjustment_abs_max_rad_;
  double foot_yaw_adjustment_abs_max_rad_;
};

}

#endif

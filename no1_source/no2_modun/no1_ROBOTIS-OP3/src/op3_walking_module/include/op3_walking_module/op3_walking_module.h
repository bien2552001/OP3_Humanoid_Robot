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

/* Author: Kayman */

#ifndef OP3_WALKING_MODULE_H_
#define OP3_WALKING_MODULE_H_

#include "op3_walking_parameter.h"

#include <stdio.h>
#include <math.h>
#include <fstream>
#include <boost/thread.hpp>
#include <eigen3/Eigen/Eigen>
#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <eigen_conversions/eigen_msg.h>

#include "robotis_controller_msgs/StatusMsg.h"
#include "op3_walking_module_msgs/WalkingParam.h"
#include "op3_walking_module_msgs/GetWalkingParam.h"
#include "op3_walking_module_msgs/SetWalkingParam.h"

#include "robotis_framework_common/motion_module.h"
#include "robotis_math/robotis_math.h"
#include "robotis_math/robotis_trajectory_calculator.h"
#include "op3_kinematics_dynamics/op3_kinematics_dynamics.h"

namespace robotis_op
{

// Định nghĩa cấu trúc Position3D
typedef struct
{
  double x, y, z;
} Position3D;

// Định nghĩa cấu trúc Pose3D
typedef struct
{
  double x, y, z, roll, pitch, yaw;
} Pose3D;

// Lớp WalkingModule kế thừa từ MotionModule và là lớp Singleton
class WalkingModule : public robotis_framework::MotionModule, public robotis_framework::Singleton<WalkingModule>
{
public:

// Enum định nghĩa các trạng thái của bước đi
enum
{
  PHASE0 = 0,  // Bước đi ở trạng thái 0
  PHASE1 = 1,  // Bước đi ở trạng thái 1
  PHASE2 = 2,  // Bước đi ở trạng thái 2
  PHASE3 = 3   // Bước đi ở trạng thái 3
};


  // Constructor
  WalkingModule();
  // Destructor
  virtual ~WalkingModule();

  // Hàm khởi tạo
  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  // Hàm xử lý chính của module
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);
  // Hàm dừng module
  void stop();
  // Hàm kiểm tra xem module có đang chạy không
  bool isRunning();
  // Hàm được gọi khi module được kích hoạt
  void onModuleEnable();
  // Hàm được gọi khi module bị vô hiệu hóa
  void onModuleDisable();

  // Getter cho phase hiện tại
  int getCurrentPhase()
  {
    return phase_;
  }

  // Getter cho biến body_swing_y
  double getBodySwingY()
  {
    return body_swing_y;
  }

  // Getter cho biến body_swing_z
  double getBodySwingZ()
  {
    return body_swing_z;
  }

private:
// Enum định nghĩa các trạng thái của WalkingModule
enum
{
  WalkingDisable = 0,    // Module đi bộ bị vô hiệu hóa
  WalkingEnable = 1,     // Module đi bộ được kích hoạt
  WalkingInitPose = 2,   // Module đi bộ đang ở trạng thái chuẩn bị pose ban đầu
  WalkingReady = 3       // Module đi bộ đã sẵn sàng để bắt đầu đi bộ
};


  const bool DEBUG;

  // Hàm xử lý trong thread của module
  void queueThread();

  /* ROS Topic Callback Functions */
  // Callback khi nhận được lệnh đi bộ từ topic
  void walkingCommandCallback(const std_msgs::String::ConstPtr &msg);
  // Callback khi nhận được tham số đi bộ từ topic
  void walkingParameterCallback(const op3_walking_module_msgs::WalkingParam::ConstPtr &msg);
  // Callback khi nhận được yêu cầu lấy tham số đi bộ từ service
  bool getWalkigParameterCallback(op3_walking_module_msgs::GetWalkingParam::Request &req,
                                  op3_walking_module_msgs::GetWalkingParam::Response &res);

  /* ROS Service Callback Functions */
  // Hàm xử lý cho từng phase
  void processPhase(const double &time_unit);
  // Hàm tính góc của chân
  bool computeLegAngle(double *leg_angle);
  // Hàm tính góc của cánh tay
  void computeArmAngle(double *arm_angle);
  // Hàm tự cân bằng
  void sensoryFeedback(const double &rlGyroErr, const double &fbGyroErr, double *balance_angle);

  // Hàm xuất thông báo trạng thái
  void publishStatusMsg(unsigned int type, std::string msg);
  // Hàm tính giá trị sin với các thông số đã cho
  double wSin(double time, double period, double period_shift, double mag, double mag_shift);
  // Hàm tính giải thuật nghịch đảo cho Inverse Kinematics
  bool computeIK(double *out, double x, double y, double z, double a, double b, double c);
  // Hàm cập nhật tham số thời gian
  void updateTimeParam();
  // Hàm cập nhật tham số chuyển động
  void updateMovementParam();
  // Hàm cập nhật tham số pose
  void updatePoseParam();
  // Hàm bắt đầu đi bộ
  void startWalking();
  // Hàm tải tham số đi bộ từ file
  void loadWalkingParam(const std::string &path);
  // Hàm lưu tham số đi bộ vào file
  void saveWalkingParam(std::string &path);
  // Hàm tạo trajectory cho Initial Pose
  void iniPoseTraGene(double mov_time);

// Con trỏ tới lớp OP3KinematicsDynamics để sử dụng các hàm của nó
OP3KinematicsDynamics *op3_kd_;

// Biến lưu giữ chu kỳ kiểm soát trong miligiây
int control_cycle_msec_;

// Đường dẫn tới thư mục chứa tham số
std::string param_path_;

// Luồng dành cho hàng đợi
boost::thread queue_thread_;

// Mutex để đồng bộ hóa việc xuất bản
boost::mutex publish_mutex_;


/* ROS Topic Publish Functions */
// Publisher cho topic robot_pose
ros::Publisher robot_pose_pub_;

// Publisher cho topic status_msg
ros::Publisher status_msg_pub_;

// Ma trận lưu trữ dữ liệu về quỹ đạo của các khớp trong suốt quá trình chuyển động
Eigen::MatrixXd calc_joint_tra_;

// Ma trận lưu trữ vị trí mục tiêu (target) của các khớp
Eigen::MatrixXd target_position_;

// Ma trận lưu trữ vị trí mục tiêu (goal) của các khớp
Eigen::MatrixXd goal_position_;

// Ma trận lưu trữ vị trí khởi tạo của các khớp
Eigen::MatrixXd init_position_;

// Ma trận lưu trữ hướng trục của các khớp
Eigen::MatrixXi joint_axis_direction_;

// Bảng ánh xạ tên khớp đến chỉ số của khớp
std::map<std::string, int> joint_table_;

// Trạng thái của quá trình đi bộ
int walking_state_;

// Số lần khởi tạo vị trí ban đầu
int init_pose_count_;

// Cấu trúc lưu trữ tham số của quá trình đi bộ
op3_walking_module_msgs::WalkingParam walking_param_;

// Biến lưu trữ biên độ di chuyển x trước đó
double previous_x_move_amplitude_;


// Biến cho việc đi bộ
double period_time_;           // Thời gian một chu kỳ đi bộ
double dsp_ratio_;              // Tỷ lệ thời gian Double Support Phase (DSP)
double ssp_ratio_;              // Tỷ lệ thời gian Single Support Phase (SSP)
double x_swap_period_time_;     // Thời gian chuyển đổi (swap) hướng di chuyển theo trục x
double x_move_period_time_;     // Thời gian di chuyển theo trục x
double y_swap_period_time_;     // Thời gian chuyển đổi (swap) hướng di chuyển theo trục y
double y_move_period_time_;     // Thời gian di chuyển theo trục y
double z_swap_period_time_;     // Thời gian chuyển đổi (swap) hướng di chuyển theo trục z
double z_move_period_time_;     // Thời gian di chuyển theo trục z
double a_move_period_time_;     // Thời gian di chuyển theo trục xoay a (yaw)
double ssp_time_;               // Thời gian SSP
double l_ssp_start_time_;       // Thời điểm bắt đầu SSP của chân trái
double l_ssp_end_time_;         // Thời điểm kết thúc SSP của chân trái
double r_ssp_start_time_;       // Thời điểm bắt đầu SSP của chân phải
double r_ssp_end_time_;         // Thời điểm kết thúc SSP của chân phải
double phase1_time_;            // Thời gian của Phase 1
double phase2_time_;            // Thời gian của Phase 2
double phase3_time_;            // Thời gian của Phase 3

// Các tham số vị trí và hướng chuyển động
double x_offset_;  // Offset vị trí theo trục x
double y_offset_;  // Offset vị trí theo trục y
double z_offset_;  // Offset vị trí theo trục z
double r_offset_;  // Offset hướng xoay roll
double p_offset_;  // Offset hướng xoay pitch
double a_offset_;  // Offset hướng xoay a (yaw)


// Các tham số chuyển động theo phương x, y, z, và góc quay
double x_swap_phase_shift_;        // Pha dịch chuyển cho chuyển động x
double x_swap_amplitude_;          // Biên độ chuyển động x
double x_swap_amplitude_shift_;    // Dịch chuyển biên độ cho chuyển động x
double x_move_phase_shift_;        // Pha dịch chuyển cho di chuyển x
double x_move_amplitude_;          // Biên độ di chuyển x
double x_move_amplitude_shift_;    // Dịch chuyển biên độ cho di chuyển x
double y_swap_phase_shift_;        // Pha dịch chuyển cho chuyển động y
double y_swap_amplitude_;          // Biên độ chuyển động y
double y_swap_amplitude_shift_;    // Dịch chuyển biên độ cho chuyển động y
double y_move_phase_shift_;        // Pha dịch chuyển cho di chuyển y
double y_move_amplitude_;          // Biên độ di chuyển y
double y_move_amplitude_shift_;    // Dịch chuyển biên độ cho di chuyển y
double z_swap_phase_shift_;        // Pha dịch chuyển cho chuyển động z
double z_swap_amplitude_;          // Biên độ chuyển động z
double z_swap_amplitude_shift_;    // Dịch chuyển biên độ cho chuyển động z
double z_move_phase_shift_;        // Pha dịch chuyển cho di chuyển z
double z_move_amplitude_;          // Biên độ di chuyển z
double z_move_amplitude_shift_;    // Dịch chuyển biên độ cho di chuyển z
double a_move_phase_shift_;        // Pha dịch chuyển cho xoay góc a (yaw)
double a_move_amplitude_;          // Biên độ xoay góc a (yaw)
double a_move_amplitude_shift_;    // Dịch chuyển biên độ cho xoay góc a (yaw)


// Các tham số về vị trí của cơ thể
double pelvis_offset_;      // Phần bù vị trí của phần lưng
double pelvis_swing_;       // Biên độ chuyển động (swing) của phần lưng
double hit_pitch_offset_;   // Phần bù vị trí của hông khi chạm mặt đất
double arm_swing_gain_;     // Hệ số kiểm soát chuyển động của cánh tay

// Biến kiểm soát việc chạy của module
bool ctrl_running_;    // Kiểm tra xem module có đang chạy hay không
bool real_running_;    // Kiểm tra xem chạy thử nghiệm thực tế hay không
double time_;          // Thời gian chạy của module


  // Biến theo dõi phase hiện tại
  int phase_;
  // Biến theo dõi biên độ swing của cơ thể theo trục y
  double body_swing_y;
  // Biến theo dõi biên độ swing của cơ thể theo trục z
  double body_swing_z;
};


}

#endif /* OP3_WALKING_MODULE_H_ */

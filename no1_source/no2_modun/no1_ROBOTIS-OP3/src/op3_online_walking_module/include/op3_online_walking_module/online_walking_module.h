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

#ifndef OP3_ONLINE_WALKING_MODULE_ONLINE_WALKING_MODULE_H_
#define OP3_ONLINE_WALKING_MODULE_ONLINE_WALKING_MODULE_H_

#include <map>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <boost/thread.hpp>
#include <eigen3/Eigen/Eigen>
#include <yaml-cpp/yaml.h>

#include "joint_control.h"
#include "wholebody_control.h"
#include "walking_control.h"
#include "op3_kdl.h"

#include "robotis_controller_msgs/JointCtrlModule.h"
#include "robotis_controller_msgs/StatusMsg.h"
#include "robotis_framework_common/motion_module.h"
#include "robotis_math/robotis_math.h"

//#include "op3_kinematics_dynamics/op3_kinematics_dynamics.h"
#include "op3_balance_control/op3_balance_control.h"

#include "op3_online_walking_module_msgs/JointPose.h"
#include "op3_online_walking_module_msgs/KinematicsPose.h"
#include "op3_online_walking_module_msgs/FootStepCommand.h"
#include "op3_online_walking_module_msgs/PreviewRequest.h"
#include "op3_online_walking_module_msgs/PreviewResponse.h"
#include "op3_online_walking_module_msgs/WalkingParam.h"

#include "op3_online_walking_module_msgs/GetJointPose.h"
#include "op3_online_walking_module_msgs/GetKinematicsPose.h"
#include "op3_online_walking_module_msgs/GetPreviewMatrix.h"

#include "op3_online_walking_module_msgs/Step2D.h"
#include "op3_online_walking_module_msgs/Step2DArray.h"

namespace robotis_op
{

enum CONTROL_TYPE {
  JOINT_CONTROL,
  WHOLEBODY_CONTROL,
  WALKING_CONTROL,
  OFFSET_CONTROL,
  NONE
};

enum BALANCE_TYPE {
  ON,
  OFF
};

// Định nghĩa lớp OnlineWalkingModule
class OnlineWalkingModule : public robotis_framework::MotionModule,
                            public robotis_framework::Singleton<OnlineWalkingModule>
{
public:
  // Constructor (hàm khởi tạo)
  OnlineWalkingModule();

  // Destructor (hàm hủy)
  virtual ~OnlineWalkingModule();


  /* ROS Topic Callback Functions */

  // Đặt lại cơ thể khi nhận được thông điệp đặt lại (reset_body=true).
  void setResetBodyCallback(const std_msgs::Bool::ConstPtr& msg);

// Thiết lập cân bằng toàn bộ cơ thể dựa trên thông điệp chuỗi.
  void setWholebodyBalanceMsgCallback(const std_msgs::String::ConstPtr& msg);

  // Đặt vị trí và hướng cơ thể dựa trên thông điệp vị trí (geometry_msgs::Pose).
  void setBodyOffsetCallback(const geometry_msgs::Pose::ConstPtr& msg);

  // Thiết lập khoảng cách giữa các chân dựa trên thông điệp số thực (std_msgs::Float64).
  void setFootDistanceCallback(const std_msgs::Float64::ConstPtr& msg);



  void goalJointPoseCallback(const op3_online_walking_module_msgs::JointPose &msg);
  void goalKinematicsPoseCallback(const op3_online_walking_module_msgs::KinematicsPose& msg);
  void footStepCommandCallback(const op3_online_walking_module_msgs::FootStepCommand& msg);
  
  


// Xử lý các lệnh đi bộ như bước chân thông qua ROS topic.  void footStepCommandCallback(const op3_online_walking_module_msgs::FootStepCommand& msg);
  void walkingParamCallback(const op3_online_walking_module_msgs::WalkingParam& msg);
// Xử lý dãy bước chân 2D thông qua ROS topic.
  void footStep2DCallback(const op3_online_walking_module_msgs::Step2DArray& msg);
// Xử lý dữ liệu IMU thông qua ROS topic.
  void imuDataCallback(const sensor_msgs::Imu::ConstPtr& msg);

// Xử lý dữ liệu lực và moment chân trái thông qua ROS topic.
  void leftFootForceTorqueOutputCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg);
  // Xử lý dữ liệu lực và moment chân phải thông qua ROS topic.
  void rightFootForceTorqueOutputCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg);









  /* ROS Service Functions */

  // Xử lý yêu cầu lấy vị trí cảm biến của các khớp thông qua ROS service.
  bool getJointPoseCallback(op3_online_walking_module_msgs::GetJointPose::Request &req,
                            op3_online_walking_module_msgs::GetJointPose::Response &res);
  
  // Xử lý yêu cầu lấy vị trí hình học của robot thông qua ROS service.
  bool getKinematicsPoseCallback(op3_online_walking_module_msgs::GetKinematicsPose::Request &req,
                                 op3_online_walking_module_msgs::GetKinematicsPose::Response &res);
  
  // Xử lý yêu cầu nhận ma trận xem trước thông qua ROS service.
  bool getPreviewMatrix(op3_online_walking_module_msgs::PreviewRequest msg);
  
  // Xử lý định nghĩa ma trận xem trước thông qua ROS service.
  bool definePreviewMatrix();





  /* ROS Framework Functions */
  // Khởi tạo các thông số của module và robot khi được gọi từ ROS framework.
  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  
  // Xử lý dữ liệu từ các cảm biến và động cơ khi được gọi từ ROS framework.
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);
 // Dừng hoạt động của module khi được gọi từ ROS framework.
  void stop();
 // Kiểm tra xem module có đang hoạt động hay không khi được gọi từ ROS framework.
  bool isRunning();



  /* yaml Functions */

// Đọc và xử lý dữ liệu cấu hình cân bằng từ file YAML được chỉ định bởi đường dẫn.
void parseBalanceGainData(const std::string &path);

// Đọc và xử lý dữ liệu phản hồi cảm biến từ file YAML được chỉ định bởi đường dẫn.
void parseJointFeedbackGainData(const std::string &path);

// Đọc và xử lý dữ liệu phản hồi trước cảm biến từ file YAML được chỉ định bởi đường dẫn.
void parseJointFeedforwardGainData(const std::string &path);



  /* ROS Publish Functions */
// Xuất thông điệp trạng thái tới các topic ROS với kiểu và nội dung cụ thể.
void publishStatusMsg(unsigned int type, std::string msg);


  /* Parameter */
// Biến để lưu trữ các đối tượng kiểm soát toàn bộ cơ thể và đi bộ.
WholebodyControl *wholebody_control_;
WalkingControl *walking_control_;

// Đối tượng thực hiện tính toán học của OP3.
OP3Kinematics *op3_kdl_;

private:
  // Hàm chạy trong một thread riêng biệt để xử lý các sự kiện từ ROS.
  void queueThread();

  // Hàm khởi tạo và tính toán kiểm soát cho các chế độ khác nhau.
  void initJointControl(); // Khởi tạo kiểm soát theo các khớp cụ thể.
  void calcJointControl(); // Tính toán kiểm soát theo các khớp cụ thể.
  
  void initWholebodyControl(); // Khởi tạo kiểm soát toàn bộ cơ thể.
  void calcWholebodyControl(); // Tính toán kiểm soát toàn bộ cơ thể.
  
  void initOffsetControl(); // Khởi tạo kiểm soát độ lệch.
  void calcOffsetControl(); // Tính toán kiểm soát độ lệch.
  
  void initWalkingControl(); // Khởi tạo kiểm soát cho chế độ đi bộ.
  void calcWalkingControl(); // Tính toán kiểm soát cho chế độ đi bộ.
  
  void initBalanceControl(); // Khởi tạo kiểm soát cân bằng.
  void calcBalanceControl(); // Tính toán kiểm soát cân bằng.


  // Hàm khởi tạo kiểm soát feedforward.
  void initFeedforwardControl();
  // Hàm thiết lập kiểm soát feedforward.
  void setFeedforwardControl();

  // Hàm thiết lập phản hồi từ cảm biến giúp cân bằng.
  void sensoryFeedback(const double &rlGyroErr, const double &fbGyroErr, double *balance_angle);

  // Hàm tính toán tọa độ của robot.
  void calcRobotPose();

  // Các hàm thiết lập lực/torque mục tiêu và các thông số kiểm soát cân bằng.
  void setTargetForceTorque();
  void setBalanceControlGain();
  bool setBalanceControl();
  
  // Hàm thiết lập kiểm soát phản hồi.
  void setFeedbackControl();
  
  // Hàm đặt lại tư thế cơ thể về tư thế ban đầu.
  void resetBodyPose();





  // Map lưu trữ ánh xạ giữa tên và ID của các động cơ.
  std::map<std::string, int> joint_name_to_id_;

  // Thời gian kiểm soát trong chu kỳ, luôn được cập nhật.
  double control_cycle_sec_;

  // Luồng xử lý hàng đợi cho các chức năng ROS.
  boost::thread queue_thread_;

  // Mutex bảo vệ tài nguyên được chia sẻ giữa các luồng.
  boost::mutex queue_mutex_;
  boost::mutex imu_data_mutex_lock_;

  // Tin nhắn ROS để thông báo trạng thái chuyển động đã hoàn thành.
  std_msgs::String movement_done_msg_;

  // Các publishers ROS để gửi dữ liệu đến các chủ đề khác nhau.
  ros::Publisher status_msg_pub_;
  ros::Publisher movement_done_pub_;
  ros::Publisher goal_joint_state_pub_;
  ros::Publisher pelvis_pose_pub_;




//  ros::ServiceClient get_preview_matrix_client_;

  // Kiểu kiểm soát hiện tại (JOINT_CONTROL, WHOLEBODY_CONTROL, WALKING_CONTROL, OFFSET_CONTROL, NONE).
  CONTROL_TYPE control_type_;

  // Biến trạng thái đang chuyển động hay không.
  bool is_moving_;

  // Kích thước và bước của chuyển động.
  int mov_size_, mov_step_;
  double mov_time_;

  // Biến kiểm soát việc khởi tạo mục tiêu.
  bool goal_initialize_;
  bool joint_control_initialize_;
  bool wholebody_initialize_;
  bool walking_initialize_;
  bool balance_control_initialize_;
  bool body_offset_initialize_;

  // Thông tin về chân đang di chuyển và giai đoạn của bước đi.
  int walking_leg_, walking_phase_;
  int walking_size_, walking_step_;

  // Các đối tượng kiểm soát chuyển động sử dụng hàm MinimumJerk.
  robotis_framework::MinimumJerk *joint_tra_;
  robotis_framework::MinimumJerk *balance_tra_;
  robotis_framework::MinimumJerk *body_offset_tra_;
  robotis_framework::MinimumJerkViaPoint *feed_forward_tra_;

  // Số lượng các động cơ trong robot.
  size_t number_of_joints_;

  // Tên của các động cơ.
  std::vector<std::string> joint_name_;

  // Nhóm kiểm soát toàn bộ cơ thể.
  std::string wholegbody_control_group_;




  // Joint Command
   // Command Information for Joints
  std::vector<double_t> curr_joint_accel_, curr_joint_vel_, curr_joint_pos_;  // Thông tin vận tốc, gia tốc và vị trí hiện tại của các động cơ.
  std::vector<double_t> des_joint_accel_,  des_joint_vel_,  des_joint_pos_;    // Thông tin vận tốc, gia tốc và vị trí mong muốn cho các động cơ.
  std::vector<double_t> goal_joint_accel_, goal_joint_vel_, goal_joint_pos_;  // Thông tin vận tốc, gia tốc và vị trí mục tiêu cho các động cơ.

  std::vector<double_t> des_joint_feedback_;         // Thông tin phản hồi mong muốn cho các động cơ.
  std::vector<double_t> des_joint_feedforward_;      // Thông tin feedforward mong muốn cho các động cơ.
  std::vector<double_t> des_joint_pos_to_robot_;     // Thông tin vị trí mong muốn của các động cơ so với robot.

  // Command Information for Arms, Legs, and Body
  std::vector<double_t> des_l_arm_pos_, des_l_arm_vel_, des_l_arm_accel_, des_l_arm_Q_;  // Thông tin vận tốc, gia tốc, vị trí và quaternion mong muốn cho cánh tay trái.
  std::vector<double_t> des_r_arm_pos_, des_r_arm_vel_, des_r_arm_accel_, des_r_arm_Q_;  // Thông tin vận tốc, gia tốc, vị trí và quaternion mong muốn cho cánh tay phải.
  std::vector<double_t> des_l_leg_pos_, des_l_leg_vel_, des_l_leg_accel_, des_l_leg_Q_;  // Thông tin vận tốc, gia tốc, vị trí và quaternion mong muốn cho chân trái.
  std::vector<double_t> des_r_leg_pos_, des_r_leg_vel_, des_r_leg_accel_, des_r_leg_Q_;  // Thông tin vận tốc, gia tốc, vị trí và quaternion mong muốn cho chân phải.
  std::vector<double_t> des_body_pos_,  des_body_vel_,  des_body_accel_,  des_body_Q_;    // Thông tin vận tốc, gia tốc, vị trí và quaternion mong muốn cho cơ thể.





  // Walking Control
  std::vector<double_t> x_lipm_, y_lipm_;  // Thông tin về vị trí x và y của Linear Inverted Pendulum Mode (LIPM).

  op3_online_walking_module_msgs::FootStepCommand foot_step_command_;  // Lệnh bước chân cho điều khiển bước chân.
  op3_online_walking_module_msgs::PreviewRequest preview_request_;     // Yêu cầu xem trước cho điều khiển.
  op3_online_walking_module_msgs::PreviewResponse preview_response_;   // Phản hồi xem trước cho điều khiển.
  op3_online_walking_module_msgs::WalkingParam walking_param_;         // Tham số điều khiển cho chế độ đi bộ.

  op3_online_walking_module_msgs::Step2DArray foot_step_2d_;  // Thông tin về bước chân trong không gian 2D.
  bool is_foot_step_2d_;                                      // Biến kiểm tra xem có phải là bước chân trong không gian 2D không.

  std::vector<double_t> preview_response_K_;  // Thông tin về ma trận K từ phản hồi xem trước.
  int preview_response_K_row_, preview_response_K_col_;  // Kích thước của ma trận K.

  std::vector<double_t> preview_response_P_;  // Thông tin về ma trận P từ phản hồi xem trước.
  int preview_response_P_row_, preview_response_P_col_;  // Kích thước của ma trận P.



  // Wholebody Control
  geometry_msgs::Pose wholebody_goal_msg_;  // Thông tin về mục tiêu điều khiển toàn bộ cơ thể.



  // Balance Control
  BALANCE_TYPE balance_type_;  // Loại cân bằng (Ví dụ: PID, PD, ...)

  bool is_balancing_;  // Biến kiểm tra trạng thái cân bằng

  int balance_step_, balance_size_;  // Bước và kích thước cân bằng

  BalanceControlUsingPDController balance_control_;  // Đối tượng điều khiển cân bằng sử dụng bộ điều khiển PD

  BalancePDController joint_feedback_[12];  // Bộ điều khiển PD cho các khớp cơ thể

  std::vector<double_t> joint_feedforward_gain_;  // Hệ số feedforward cho các khớp cơ thể

  std::vector<double_t> des_balance_gain_ratio_;  // Hệ số cân bằng mong muốn
  std::vector<double_t> goal_balance_gain_ratio_;  // Hệ số cân bằng mục tiêu




  // Body Offset
  std::vector<double_t> des_body_offset_;   // Độ lệch cơ thể mong muốn
  std::vector<double_t> goal_body_offset_;  // Độ lệch cơ thể mục tiêu

  bool is_offset_updating_;  // Biến kiểm tra cập nhật độ lệch cơ thể

  int body_offset_step_, body_offset_size_;  // Bước và kích thước độ lệch cơ thể

  double foot_distance_;  // Khoảng cách giữa hai chân



  // Balance Gain
  double foot_roll_gyro_p_gain_;    // Hệ số P cho độ lệch roll dựa trên giảm tốc góc
  double foot_roll_gyro_d_gain_;    // Hệ số D cho độ lệch roll dựa trên giảm tốc góc
  double foot_pitch_gyro_p_gain_;   // Hệ số P cho độ lệch pitch dựa trên giảm tốc góc
  double foot_pitch_gyro_d_gain_;   // Hệ số D cho độ lệch pitch dựa trên giảm tốc góc


  double foot_roll_angle_p_gain_;    // Hệ số P cho độ lệch roll dựa trên góc đo được
  double foot_roll_angle_d_gain_;    // Hệ số D cho độ lệch roll dựa trên giảm tốc góc đo được
  double foot_pitch_angle_p_gain_;   // Hệ số P cho độ lệch pitch dựa trên góc đo được
  double foot_pitch_angle_d_gain_;   // Hệ số D cho độ lệch pitch dựa trên giảm tốc góc đo được

  double foot_x_force_p_gain_;       // Hệ số P cho lực dọc trên trục x đo được từ cảm biến lực
  double foot_x_force_d_gain_;       // Hệ số D cho lực dọc trên trục x đo được từ cảm biến lực
  double foot_y_force_p_gain_;       // Hệ số P cho lực dọc trên trục y đo được từ cảm biến lực
  double foot_y_force_d_gain_;       // Hệ số D cho lực dọc trên trục y đo được từ cảm biến lực
  double foot_z_force_p_gain_;       // Hệ số P cho lực dọc trên trục z đo được từ cảm biến lực
  double foot_z_force_d_gain_;       // Hệ số D cho lực dọc trên trục z đo được từ cảm biến lực


  double foot_roll_torque_p_gain_;    // Hệ số P cho mô-men xoay roll dựa trên mô-men xoay đo được từ cảm biến lực
  double foot_roll_torque_d_gain_;    // Hệ số D cho mô-men xoay roll dựa trên giảm tốc mô-men xoay đo được từ cảm biến lực
  double foot_pitch_torque_p_gain_;   // Hệ số P cho mô-men xoay pitch dựa trên mô-men xoay đo được từ cảm biến lực
  double foot_pitch_torque_d_gain_;   // Hệ số D cho mô-men xoay pitch dựa trên giảm tốc mô-men xoay đo được từ cảm biến lực

  double roll_gyro_cut_off_frequency_;    // Tần số cắt cho độ lệch góc roll dựa trên giảm tốc góc đo được
  double pitch_gyro_cut_off_frequency_;   // Tần số cắt cho độ lệch góc pitch dựa trên giảm tốc góc đo được

  double roll_angle_cut_off_frequency_;   // Tần số cắt cho góc roll dựa trên giảm tốc góc đo được
  double pitch_angle_cut_off_frequency_;  // Tần số cắt cho góc pitch dựa trên giảm tốc góc đo được


  double foot_x_force_cut_off_frequency_;           // Tần số cắt cho lực tác động theo trục X đo được từ cảm biến lực
  double foot_y_force_cut_off_frequency_;           // Tần số cắt cho lực tác động theo trục Y đo được từ cảm biến lực
  double foot_z_force_cut_off_frequency_;           // Tần số cắt cho lực tác động theo trục Z đo được từ cảm biến lực

  double foot_roll_torque_cut_off_frequency_;       // Tần số cắt cho mô-men xoay roll đo được từ cảm biến lực
  double foot_pitch_torque_cut_off_frequency_;      // Tần số cắt cho mô-men xoay pitch đo được từ cảm biến lực

  double balance_hip_roll_gain_;                    // Hệ số điều khiển PD cho cân bằng roll của cơ thể
  double balance_knee_gain_;                        // Hệ số điều khiển PD cho cân bằng knee của cơ thể
  double balance_ankle_roll_gain_;                  // Hệ số điều khiển PD cho cân bằng roll của mắt cá chân
  double balance_ankle_pitch_gain_;                 // Hệ số điều khiển PD cho cân bằng pitch của mắt cá chân




  // Balance Control : Desired Force
  double balance_l_foot_force_x_;     // Lực tác động theo trục X đo được từ cảm biến lực ở chân trái
  double balance_l_foot_force_y_;     // Lực tác động theo trục Y đo được từ cảm biến lực ở chân trái
  double balance_l_foot_force_z_;     // Lực tác động theo trục Z đo được từ cảm biến lực ở chân trái
  double balance_l_foot_torque_x_;    // Mô-men xoay theo trục X đo được từ cảm biến lực ở chân trái
  double balance_l_foot_torque_y_;    // Mô-men xoay theo trục Y đo được từ cảm biến lực ở chân trái
  double balance_l_foot_torque_z_;    // Mô-men xoay theo trục Z đo được từ cảm biến lực ở chân trái


  double balance_r_foot_force_x_;     // Lực tác động theo trục X đo được từ cảm biến lực ở chân phải
  double balance_r_foot_force_y_;     // Lực tác động theo trục Y đo được từ cảm biến lực ở chân phải
  double balance_r_foot_force_z_;     // Lực tác động theo trục Z đo được từ cảm biến lực ở chân phải
  double balance_r_foot_torque_x_;    // Mô-men xoay theo trục X đo được từ cảm biến lực ở chân phải
  double balance_r_foot_torque_y_;    // Mô-men xoay theo trục Y đo được từ cảm biến lực ở chân phải
  double balance_r_foot_torque_z_;    // Mô-men xoay theo trục Z đo được từ cảm biến lực ở chân phải


Eigen::MatrixXd g_to_r_leg_;  // Ma trận chuyển đổi từ không gian toàn cầu (g) sang chân phải (r_leg)
Eigen::MatrixXd g_to_l_leg_;  // Ma trận chuyển đổi từ không gian toàn cầu (g) sang chân trái (l_leg)




  // Sensor msgs
// Sensor messages
sensor_msgs::Imu imu_data_msg_;               // Dữ liệu IMU (Inertial Measurement Unit)
geometry_msgs::Wrench l_foot_ft_data_msg_;    // Dữ liệu lực và mô-men chân trái
geometry_msgs::Wrench r_foot_ft_data_msg_;    // Dữ liệu lực và mô-men chân phải

double total_mass_;                            // Tổng khối lượng của robot (có thể là tổng khối lượng các phần của robot)

};


}

#endif

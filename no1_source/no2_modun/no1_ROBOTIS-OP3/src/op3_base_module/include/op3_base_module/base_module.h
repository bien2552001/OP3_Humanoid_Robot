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

/* Authors: SCH, Kayman */

#ifndef BASEMODULE_H_
#define BASEMODULE_H_

#include <map>
#include <boost/thread.hpp>
#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>

#include "robotis_framework_common/motion_module.h"
#include "robotis_controller_msgs/JointCtrlModule.h"
#include "robotis_controller_msgs/SetModule.h"
#include "robotis_controller_msgs/StatusMsg.h"
#include "robotis_math/robotis_math.h"
#include "op3_kinematics_dynamics/op3_kinematics_dynamics.h"

#include "base_module_state.h"

namespace robotis_op
{

// Định nghĩa một lớp chứa dữ liệu về một khớp cơ bản
class BaseJointData
{
 public:
  double position_;  // Vị trí của khớp
  double velocity_;  // Vận tốc của khớp
  double effort_;    // Nỗ lực (hoặc công suất) của khớp

  int p_gain_;       // Hệ số P trong PID
  int i_gain_;       // Hệ số I trong PID
  int d_gain_;       // Hệ số D trong PID
};

// Định nghĩa lớp chứa trạng thái của các khớp cơ bản (hiện tại, mục tiêu, giả mạo)
class BaseJointState
{
 public:
  BaseJointData curr_joint_state_[MAX_JOINT_ID + 1];  // Trạng thái hiện tại của các khớp
  BaseJointData goal_joint_state_[MAX_JOINT_ID + 1];  // Trạng thái mục tiêu của các khớp
  BaseJointData fake_joint_state_[MAX_JOINT_ID + 1];  // Trạng thái giả mạo của các khớp
};

// Lớp chính của module cơ bản, kế thừa từ MotionModule và Singleton
class BaseModule : public robotis_framework::MotionModule, public robotis_framework::Singleton<BaseModule>
{
 public:
  BaseModule();      // Constructor
  virtual ~BaseModule(); // Destructor

  /* ROS Framework Functions */
  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);  // Hàm khởi tạo
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors); // Hàm xử lý chính

  void stop();       // Dừng module
  bool isRunning();  // Kiểm tra xem module có đang chạy không

  void onModuleEnable();  // Xử lý khi module được bật
  void onModuleDisable(); // Xử lý khi module được tắt

  /* ROS Topic Callback Functions */
  void initPoseMsgCallback(const std_msgs::String::ConstPtr& msg); // Hàm xử lý khi có sự kiện callback từ topic

  /* ROS Calculation Functions */
  void initPoseTrajGenerateProc();  // Hàm tạo quỹ đạo cho vị trí khởi tạo

  void poseGenerateProc(Eigen::MatrixXd joint_angle_pose); // Hàm tạo quỹ đạo vị trí
  void poseGenerateProc(std::map<std::string, double>& joint_angle_pose); // Hàm tạo quỹ đạo vị trí

  /* Parameter */
  BaseModuleState *base_module_state_;  // Trạng thái của module cơ bản
  BaseJointState *joint_state_;         // Trạng thái của các khớp

 private:
  void queueThread();  // Hàm xử lý trong thread
  void setCtrlModule(std::string module);  // Hàm đặt module điều khiển
  void callServiceSettingModule(const std::string &module_name);  // Gọi dịch vụ để đặt module
  void parseInitPoseData(const std::string &path);  // Phân tích dữ liệu về vị trí khởi tạo từ đường dẫn
  void publishStatusMsg(unsigned int type, std::string msg);  // Xuất thông điệp trạng thái

  int control_cycle_msec_; // Chu kỳ điều khiển
  boost::thread queue_thread_;  // Thread xử lý
  boost::thread tra_gene_tread_; // Thread tạo quỹ đạo

  ros::Publisher status_msg_pub_;     // Publisher cho thông điệp trạng thái
  ros::Publisher set_ctrl_module_pub_;  // Publisher để đặt module điều khiển

  ros::ServiceClient set_module_client_;  // Client để gọi dịch vụ để đặt module

  std::map<std::string, int> joint_name_to_id_;  // Ánh xạ tên khớp sang ID

  bool has_goal_joints_;   // Biến kiểm tra xem có mục tiêu cho các khớp không
  bool ini_pose_only_;     // Biến kiểm tra xem chỉ có vị trí khởi tạo không

  std::string	init_pose_file_path_;  // Đường dẫn đến tệp chứa dữ liệu về vị trí khởi tạo
};


}

#endif /* BASEMODULE_H_ */

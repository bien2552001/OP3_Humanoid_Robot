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

/* Authors: Kayman, Jay Song */

#ifndef ACTION_MOTION_MODULE_H_
#define ACTION_MOTION_MODULE_H_

#define _USE_MATH_DEFINES

#include <cmath>
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/callback_queue.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <boost/thread.hpp>

#include "robotis_controller_msgs/StatusMsg.h"
#include "op3_action_module_msgs/IsRunning.h"
#include "op3_action_module_msgs/StartAction.h"
#include "robotis_framework_common/motion_module.h"
#include "action_file_define.h"

namespace robotis_op
{

class ActionModule : public robotis_framework::MotionModule, public robotis_framework::Singleton<ActionModule>
{
 public:
  // Constructor (Hàm tạo)
  ActionModule();
  // Destructor (Hàm hủy)
  virtual ~ActionModule();

  // Phương thức khởi tạo module
  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  // Phương thức xử lý chính của module
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);

  // Dừng module
  void stop();
  // Kiểm tra xem module có đang chạy hay không
  bool isRunning();

  // Load dữ liệu từ file
  bool loadFile(std::string file_name);
  // Tạo mới file
  bool createFile(std::string file_name);

  // Bắt đầu chạy hành động theo số trang
  bool start(int page_number);
  // Bắt đầu chạy hành động theo tên trang
  bool start(std::string page_name);
  // Bắt đầu chạy hành động theo số trang và trang cụ thể
  bool start(int page_number, action_file_define::Page* page);

  // Xử lý khi module được kích hoạt
  void onModuleEnable();
  // Xử lý khi module bị vô hiệu hóa
  void onModuleDisable();

  // Phương thức phanh
  void brake();
  // Kiểm tra xem hành động có đang chạy hay không
  bool isRunning(int* playing_page_num, int* playing_step_num);
  // Load trang từ file
  bool loadPage(int page_number, action_file_define::Page* page);
  // Lưu trang vào file
  bool savePage(int page_number, action_file_define::Page* page);
  // Đặt lại trang
  void resetPage(action_file_define::Page* page);

  // Kích hoạt tất cả các khớp
  void enableAllJoints();
  // Xử lý chính của quá trình phát hành động
  void actionPlayProcess(std::map<std::string, robotis_framework::Dynamixel *> dxls);

 private:
  // Các hằng số cho các phần của hành động
  const int PRE_SECTION;
  const int MAIN_SECTION;
  const int POST_SECTION;
  const int PAUSE_SECTION;
  const int ZERO_FINISH;
  const int NONE_ZERO_FINISH;
  const bool DEBUG_PRINT;

  // Phương thức chạy trong luồng
  void queueThread();

  // Xác nhận checksum của trang
  bool verifyChecksum( action_file_define::Page* page );
  // Đặt checksum cho trang
  void setChecksum( action_file_define::Page* page );

  // Xuất thông điệp trạng thái
  void publishStatusMsg(unsigned int type, std::string msg);
  // Xuất thông điệp khi hoàn thành hành động
  void publishDoneMsg(std::string msg);

  // Callback cho dịch vụ kiểm tra xem hành động có đang chạy hay không
  bool isRunningServiceCallback(op3_action_module_msgs::IsRunning::Request  &req,
                                op3_action_module_msgs::IsRunning::Response &res);

  // Callback khi số trang được nhận
  void pageNumberCallback(const std_msgs::Int32::ConstPtr& msg);
  // Callback khi yêu cầu bắt đầu hành động được nhận
  void startActionCallback(const op3_action_module_msgs::StartAction::ConstPtr& msg);

  // Chuyển đổi từ radian sang w4095
  int convertRadTow4095(double rad);
  // Chuyển đổi từ w4095 sang radian
  double convertw4095ToRad(int w4095);
  // Chuyển đổi số nguyên sang chuỗi
  std::string convertIntToString(int n);

  // Dùng để lưu trạng thái của các khớp trong hành động
  std::map<std::string, bool> action_joints_enable_;
  // Dùng để lưu trạng thái của các động cơ
  std::map<std::string, robotis_framework::DynamixelState *> action_result_;
  // Chu kỳ kiểm soát
  int             control_cycle_msec_;
  // Luồng cho queue
  boost::thread   queue_thread_;

  /* Các publisher & subscriber mẫu */
  ros::Publisher status_msg_pub_;
  ros::Publisher  done_msg_pub_;
  /////////////////////////////////////////////////////////////////////////
  // Dùng để ánh xạ tên và ID của các khớp
  std::map<std::string, int> joint_name_to_id_;
  std::map<int, std::string> joint_id_to_name_;
  // Tệp hành động
  FILE* action_file_;
  // Trang đang chạy
  action_file_define::Page play_page_;
  // Trang kế tiếp cần chạy
  action_file_define::Page next_play_page_;
  // Bước hiện tại của hành động
  action_file_define::Step current_step_;

  // Chỉ số của trang đang chạy
  int play_page_idx_;
  // Biến kiểm tra để đảm bảo rằng việc khởi đầu lần đầu chỉ xảy ra một lần
  bool first_driving_start_;
  // Số bước trong trang đang chạy
  int page_step_count_;

  // Biến kiểm tra xem hành động có đang chạy hay không
  bool playing_;
  // Biến kiểm tra để dừng chạy hành động
  bool stop_playing_;
  // Biến kiểm tra để kiểm tra xem hành động đã hoàn thành hay chưa
  bool playing_finished_;

  // Biến kiểm tra để kiểm tra xem module có được kích hoạt hay không
  bool action_module_enabled_;
  // Biến kiểm tra để lưu trạng thái chạy trước và sau khi module được kích hoạt
  bool previous_running_;
  bool present_running_;
};


}

#endif /* OP3_ACTION_MOTION_MODULE_H_ */

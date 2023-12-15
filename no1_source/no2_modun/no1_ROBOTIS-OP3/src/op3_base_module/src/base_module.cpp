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

/* Author: Kayman, SCH */

#include <stdio.h>
#include "op3_base_module/base_module.h"

namespace robotis_op
{

  // Constructor của lớp BaseModule
  BaseModule::BaseModule()
      : control_cycle_msec_(0),  // Khởi tạo chu kỳ điều khiển là 0 mili giây
        has_goal_joints_(false), // Không có mục tiêu cho các khớp khi mới khởi tạo
        ini_pose_only_(false),   // Chưa chỉ có vị trí khởi tạo
        init_pose_file_path_("") // Đường dẫn đến tệp vị trí khởi tạo trống
  {
    enable_ = false;                                    // Module chưa được bật khi mới khởi tạo
    module_name_ = "base_module";                       // Tên của module là "base_module"
    control_mode_ = robotis_framework::PositionControl; // Chế độ điều khiển là PositionControl

    base_module_state_ = new BaseModuleState(); // Khởi tạo trạng thái của module
    joint_state_ = new BaseJointState();        // Khởi tạo trạng thái của các khớp
  }

  // Destructor của lớp BaseModule
  BaseModule::~BaseModule()
  {
    queue_thread_.join(); // Chờ thread kết thúc trước khi hủy đối tượng
  }

  // Hàm khởi tạo của lớp BaseModule
  void BaseModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
  {
    control_cycle_msec_ = control_cycle_msec; // Gán giá trị của chu kỳ điều khiển

    // Tạo một thread để xử lý hàng đợi
    queue_thread_ = boost::thread(boost::bind(&BaseModule::queueThread, this));

    // Khởi tạo kết quả và bảng ánh xạ ID của khớp
    for (std::map<std::string, robotis_framework::Dynamixel *>::iterator it = robot->dxls_.begin();
         it != robot->dxls_.end(); it++)
    {
      std::string joint_name = it->first;
      robotis_framework::Dynamixel *dxl_info = it->second;

      joint_name_to_id_[joint_name] = dxl_info->id_;
      result_[joint_name] = new robotis_framework::DynamixelState();
      result_[joint_name]->goal_position_ = dxl_info->dxl_state_->goal_position_;
    }

    ros::NodeHandle ros_node;

    /* Load ROS Parameter */
    ros_node.param<std::string>("init_pose_file_path", init_pose_file_path_, ros::package::getPath("op3_base_module") + "/data/ini_pose.yaml");

    /* publish topics */
    // Publisher cho thông điệp trạng thái
    status_msg_pub_ = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 1);
    // Publisher để bật/tắt module
    set_ctrl_module_pub_ = ros_node.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 1);
  }

  // Hàm phân tích dữ liệu vị trí khởi tạo từ tệp YAML
  void BaseModule::parseInitPoseData(const std::string &path)
  {
    YAML::Node doc;
    try
    {
      // Nạp tệp YAML
      doc = YAML::LoadFile(path.c_str());
    }
    catch (const std::exception &e)
    {
      ROS_ERROR("Fail to load yaml file."); // Ghi log lỗi nếu không thể nạp tệp YAML
      return;
    }

    // Phân tích thời gian di chuyển từ tệp YAML
    double mov_time;
    mov_time = doc["mov_time"].as<double>();
    base_module_state_->mov_time_ = mov_time;

    // Phân tích số điểm thông qua từ tệp YAML
    int via_num;
    via_num = doc["via_num"].as<int>();
    base_module_state_->via_num_ = via_num;

    // Phân tích thời gian của điểm thông qua từ tệp YAML
    std::vector<double> via_time;
    via_time = doc["via_time"].as<std::vector<double>>();
    base_module_state_->via_time_.resize(via_num, 1);
    for (int num = 0; num < via_num; num++)
      base_module_state_->via_time_.coeffRef(num, 0) = via_time[num];

    // Phân tích vị trí của điểm thông qua từ tệp YAML
    base_module_state_->joint_via_pose_.resize(via_num, MAX_JOINT_ID + 1);
    base_module_state_->joint_via_dpose_.resize(via_num, MAX_JOINT_ID + 1);
    base_module_state_->joint_via_ddpose_.resize(via_num, MAX_JOINT_ID + 1);

    base_module_state_->joint_via_pose_.fill(0.0);
    base_module_state_->joint_via_dpose_.fill(0.0);
    base_module_state_->joint_via_ddpose_.fill(0.0);

    YAML::Node via_pose_node = doc["via_pose"];
    for (YAML::iterator yaml_it = via_pose_node.begin(); yaml_it != via_pose_node.end(); ++yaml_it)
    {
      int id;
      std::vector<double> value;

      id = yaml_it->first.as<int>();
      value = yaml_it->second.as<std::vector<double>>();

      for (int num = 0; num < via_num; num++)
        base_module_state_->joint_via_pose_.coeffRef(num, id) = value[num] * DEGREE2RADIAN;
    }

    // Phân tích vị trí mục tiêu từ tệp YAML
    YAML::Node tar_pose_node = doc["tar_pose"];
    for (YAML::iterator yaml_it = tar_pose_node.begin(); yaml_it != tar_pose_node.end(); ++yaml_it)
    {
      int id;
      double value;

      id = yaml_it->first.as<int>();
      value = yaml_it->second.as<double>();

      base_module_state_->joint_ini_pose_.coeffRef(id, 0) = value * DEGREE2RADIAN;
    }

    // Tính toán số bước thời gian dựa trên thời gian di chuyển và thời gian lấy mẫu
    base_module_state_->all_time_steps_ = int(base_module_state_->mov_time_ / base_module_state_->smp_time_) + 1;
    // Điều chỉnh kích thước của ma trận quỹ đạo khớp tính toán
    base_module_state_->calc_joint_tra_.resize(base_module_state_->all_time_steps_, MAX_JOINT_ID + 1);
  }

  // Hàm chạy trong một thread riêng để xử lý hàng đợi ROS
  void BaseModule::queueThread()
  {
    ros::NodeHandle ros_node;
    ros::CallbackQueue callback_queue;

    ros_node.setCallbackQueue(&callback_queue);

    /* Đăng ký các chủ đề để lắng nghe */
    ros::Subscriber ini_pose_msg_sub = ros_node.subscribe("/robotis/base/ini_pose", 5, &BaseModule::initPoseMsgCallback, this);
    set_module_client_ = ros_node.serviceClient<robotis_controller_msgs::SetModule>("/robotis/set_present_ctrl_modules");

    ros::WallDuration duration(control_cycle_msec_ / 1000.0);

    // Vòng lặp chờ và xử lý các sự kiện trong hàng đợi ROS
    while (ros_node.ok())
      callback_queue.callAvailable(duration);
  }

  // Callback được gọi khi nhận được thông điệp vị trí khởi tạo
  void BaseModule::initPoseMsgCallback(const std_msgs::String::ConstPtr &msg)
  {
    // Kiểm tra xem module có đang thực hiện một nhiệm vụ không
    if (base_module_state_->is_moving_ == false)
    {
      // Nếu thông điệp yêu cầu khởi tạo vị trí
      if (msg->data == "ini_pose")
      {
        // Đặt module của tất cả các khớp thành module hiện tại (base_module)
        callServiceSettingModule(module_name_);

        // Chờ cho việc chuyển module thành công và nhận được mục tiêu vị trí
        while (enable_ == false || has_goal_joints_ == false)
          usleep(8 * 1000); // Chờ 8 milliseconds

        // Phân tích dữ liệu vị trí khởi tạo từ tệp YAML
        parseInitPoseData(init_pose_file_path_);

        // Tạo quỹ đạo cho vị trí khởi tạo
        tra_gene_tread_ = boost::thread(boost::bind(&BaseModule::initPoseTrajGenerateProc, this));
      }
    }
    else
      ROS_INFO("previous task is alive"); // Thông báo nếu nhiệm vụ trước đó vẫn đang chạy

    return;
  }

  // Hàm tạo quỹ đạo cho vị trí khởi tạo
  void BaseModule::initPoseTrajGenerateProc()
  {
    // Duyệt qua tất cả các khớp
    for (int id = 1; id <= MAX_JOINT_ID; id++)
    {
      // Lấy giá trị vị trí hiện tại của khớp
      double ini_value = joint_state_->goal_joint_state_[id].position_;
      // Lấy giá trị vị trí mục tiêu từ dữ liệu vị trí khởi tạo
      double tar_value = base_module_state_->joint_ini_pose_.coeff(id, 0);

      Eigen::MatrixXd tra; // Ma trận quỹ đạo

      // Nếu không có điểm thông qua
      if (base_module_state_->via_num_ == 0)
      {
        // Tính toán quỹ đạo bằng phương pháp Minimum Jerk
        tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0, tar_value, 0.0, 0.0,
                                                    base_module_state_->smp_time_, base_module_state_->mov_time_);
      }
      else // Nếu có điểm thông qua
      {
        // Lấy giá trị điểm thông qua, vận tốc và gia tốc
        Eigen::MatrixXd via_value = base_module_state_->joint_via_pose_.col(id);
        Eigen::MatrixXd d_via_value = base_module_state_->joint_via_dpose_.col(id);
        Eigen::MatrixXd dd_via_value = base_module_state_->joint_via_ddpose_.col(id);

        // Tính toán quỹ đạo bằng phương pháp Minimum Jerk với điểm thông qua
        tra = robotis_framework::calcMinimumJerkTraWithViaPoints(base_module_state_->via_num_, ini_value, 0.0, 0.0,
                                                                 via_value, d_via_value, dd_via_value, tar_value, 0.0,
                                                                 0.0, base_module_state_->smp_time_,
                                                                 base_module_state_->via_time_,
                                                                 base_module_state_->mov_time_);
      }

      // Lưu quỹ đạo vào ma trận trạng thái của module
      base_module_state_->calc_joint_tra_.block(0, id, base_module_state_->all_time_steps_, 1) = tra;
    }

    // Thiết lập trạng thái di chuyển và thiết lập bộ đếm về 0
    base_module_state_->is_moving_ = true;
    base_module_state_->cnt_ = 0;
    ROS_INFO("[start] send trajectory");
  }

  // Hàm tạo quỹ đạo di chuyển đến một vị trí cụ thể
  void BaseModule::poseGenerateProc(Eigen::MatrixXd joint_angle_pose)
  {
    // Đặt module của tất cả các khớp thành module hiện tại (base_module)
    callServiceSettingModule(module_name_);

    // Chờ cho việc chuyển module thành công và nhận được mục tiêu vị trí
    while (enable_ == false || has_goal_joints_ == false)
      usleep(8 * 1000); // Chờ 8 milliseconds

    // Thiết lập thời gian di chuyển và tính toán số bước thời gian
    base_module_state_->mov_time_ = 5.0;
    base_module_state_->all_time_steps_ = int(base_module_state_->mov_time_ / base_module_state_->smp_time_) + 1;

    // Điều chỉnh kích thước của ma trận quỹ đạo khớp tính toán
    base_module_state_->calc_joint_tra_.resize(base_module_state_->all_time_steps_, MAX_JOINT_ID + 1);

    // Gán giá trị vị trí mục tiêu
    base_module_state_->joint_pose_ = joint_angle_pose;

    // Duyệt qua tất cả các khớp
    for (int id = 1; id <= MAX_JOINT_ID; id++)
    {
      // Lấy giá trị vị trí hiện tại của khớp
      double ini_value = joint_state_->goal_joint_state_[id].position_;
      // Lấy giá trị vị trí mục tiêu từ ma trận vị trí mục tiêu
      double tar_value = base_module_state_->joint_pose_.coeff(id, 0);

      // Ghi log thông tin vị trí khởi tạo và vị trí mục tiêu
      ROS_INFO_STREAM("[ID : " << id << "] ini_value : " << ini_value << "  tar_value : " << tar_value);

      // Tính toán quỹ đạo bằng phương pháp Minimum Jerk
      Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0, tar_value, 0.0, 0.0,
                                                                  base_module_state_->smp_time_,
                                                                  base_module_state_->mov_time_);

      // Lưu quỹ đạo vào ma trận trạng thái của module
      base_module_state_->calc_joint_tra_.block(0, id, base_module_state_->all_time_steps_, 1) = tra;
    }

    // Thiết lập trạng thái di chuyển và thiết lập bộ đếm về 0
    base_module_state_->is_moving_ = true;
    base_module_state_->cnt_ = 0;
    ini_pose_only_ = true;
    ROS_INFO("[start] send trajectory");
  }

  // Hàm tạo quỹ đạo di chuyển đến một vị trí cụ thể dựa trên map của các góc khớp
  void BaseModule::poseGenerateProc(std::map<std::string, double> &joint_angle_pose)
  {
    // Đặt module của tất cả các khớp thành module hiện tại (base_module)
    callServiceSettingModule(module_name_);

    // Chờ cho việc chuyển module thành công và nhận được mục tiêu vị trí
    while (enable_ == false || has_goal_joints_ == false)
      usleep(8 * 1000); // Chờ 8 milliseconds

    // Khởi tạo ma trận vị trí mục tiêu với giá trị mặc định là 0
    Eigen::MatrixXd target_pose = Eigen::MatrixXd::Zero(MAX_JOINT_ID + 1, 1);

    // Duyệt qua tất cả các cặp khóa-giá trị trong map góc khớp
    for (std::map<std::string, double>::iterator joint_angle_it = joint_angle_pose.begin();
         joint_angle_it != joint_angle_pose.end(); joint_angle_it++)
    {
      std::string joint_name = joint_angle_it->first;
      double joint_angle_rad = joint_angle_it->second;

      // Tìm id của khớp trong map joint_name_to_id_
      std::map<std::string, int>::iterator joint_name_to_id_it = joint_name_to_id_.find(joint_name);
      if (joint_name_to_id_it != joint_name_to_id_.end())
      {
        // Gán giá trị góc khớp vào ma trận vị trí mục tiêu
        target_pose.coeffRef(joint_name_to_id_it->second, 0) = joint_angle_rad;
      }
    }

    // Gán ma trận vị trí mục tiêu cho trạng thái của module
    base_module_state_->joint_pose_ = target_pose;

    // Thiết lập thời gian di chuyển và tính toán số bước thời gian
    base_module_state_->mov_time_ = 5.0;
    base_module_state_->all_time_steps_ = int(base_module_state_->mov_time_ / base_module_state_->smp_time_) + 1;

    // Điều chỉnh kích thước của ma trận quỹ đạo khớp tính toán
    base_module_state_->calc_joint_tra_.resize(base_module_state_->all_time_steps_, MAX_JOINT_ID + 1);

    // Duyệt qua tất cả các khớp
    for (int id = 1; id <= MAX_JOINT_ID; id++)
    {
      // Lấy giá trị vị trí hiện tại của khớp
      double ini_value = joint_state_->goal_joint_state_[id].position_;
      // Lấy giá trị vị trí mục tiêu từ ma trận vị trí mục tiêu
      double tar_value = base_module_state_->joint_pose_.coeff(id, 0);

      // Ghi log thông tin vị trí khởi tạo và vị trí mục tiêu
      ROS_INFO_STREAM("[ID : " << id << "] ini_value : " << ini_value << "  tar_value : " << tar_value);

      // Tính toán quỹ đạo bằng phương pháp Minimum Jerk
      Eigen::MatrixXd tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0, tar_value, 0.0, 0.0,
                                                                  base_module_state_->smp_time_,
                                                                  base_module_state_->mov_time_);

      // Lưu quỹ đạo vào ma trận trạng thái của module
      base_module_state_->calc_joint_tra_.block(0, id, base_module_state_->all_time_steps_, 1) = tra;
    }

    // Thiết lập trạng thái di chuyển và thiết lập bộ đếm về 0
    base_module_state_->is_moving_ = true;
    base_module_state_->cnt_ = 0;
    ini_pose_only_ = true;
    ROS_INFO("[start] send trajectory");
  }

  // Kiểm tra xem module có đang di chuyển không
  bool BaseModule::isRunning()
  {
    return base_module_state_->is_moving_;
  }

  // Hàm xử lý các bước trong chu kỳ điều khiển
  void BaseModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                           std::map<std::string, double> sensors)
  {
    // Nếu module không được kích hoạt, thoát khỏi hàm
    if (enable_ == false)
      return;

    /*----- Ghi vị trí hiện tại -----*/
    for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
         state_iter != result_.end(); state_iter++)
    {
      std::string joint_name = state_iter->first;

      // Lấy thông tin về khớp từ map dxls
      robotis_framework::Dynamixel *dxl = NULL;
      std::map<std::string, robotis_framework::Dynamixel *>::iterator dxl_it = dxls.find(joint_name);
      if (dxl_it != dxls.end())
        dxl = dxl_it->second;
      else
        continue;

      // Lấy vị trí hiện tại và vị trí mục tiêu của khớp
      double joint_curr_position = dxl->dxl_state_->present_position_;
      double joint_goal_position = dxl->dxl_state_->goal_position_;

      // Cập nhật vị trí hiện tại và vị trí mục tiêu trong trạng thái của module
      joint_state_->curr_joint_state_[joint_name_to_id_[joint_name]].position_ = joint_curr_position;
      joint_state_->goal_joint_state_[joint_name_to_id_[joint_name]].position_ = joint_goal_position;
    }

    // Đã có mục tiêu vị trí
    has_goal_joints_ = true;

    /* ----- Gửi quỹ đạo ----- */
    if (base_module_state_->is_moving_ == true)
    {
      // Hiển thị thông báo khi bắt đầu di chuyển đến vị trí khởi tạo
      if (base_module_state_->cnt_ == 1)
        publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start Init Pose");

      // Cập nhật vị trí mục tiêu của từng khớp dựa trên quỹ đạo tính toán
      for (int id = 1; id <= MAX_JOINT_ID; id++)
        joint_state_->goal_joint_state_[id].position_ = base_module_state_->calc_joint_tra_(base_module_state_->cnt_, id);

      // Tăng bộ đếm
      base_module_state_->cnt_++;
    }

    /*----- Thiết lập dữ liệu của khớp -----*/
    for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
         state_iter != result_.end(); state_iter++)
    {
      std::string joint_name = state_iter->first;

      // Cập nhật giá trị vị trí mục tiêu của khớp trong result_
      result_[joint_name]->goal_position_ = joint_state_->goal_joint_state_[joint_name_to_id_[joint_name]].position_;
    }

    /*---------- Khởi tạo số đếm ----------*/
    // Nếu đã hoàn thành việc gửi quỹ đạo và module đang di chuyển
    if ((base_module_state_->cnt_ >= base_module_state_->all_time_steps_) && (base_module_state_->is_moving_ == true))
    {
      // Ghi log và gửi thông báo khi hoàn thành việc gửi quỹ đạo
      ROS_INFO("[end] send trajectory");
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Finish Init Pose");

      // Đặt trạng thái di chuyển về false và đặt bộ đếm về 0
      base_module_state_->is_moving_ = false;
      base_module_state_->cnt_ = 0;

      // Nếu chỉ là khởi tạo vị trí, đặt tất cả các khớp về module none
      if (ini_pose_only_ == true)
      {
        setCtrlModule("none");
        ini_pose_only_ = false;
      }
    }
  }

  // Tạm dừng module (không có thực hiện công việc cụ thể)
  void BaseModule::stop()
  {
    return;
  }

  // Xử lý khi module được kích hoạt
  void BaseModule::onModuleEnable()
  {
    ROS_INFO("Base Module is enabled");
  }

  // Xử lý khi module bị vô hiệu hóa
  void BaseModule::onModuleDisable()
  {
    // Đặt cờ hiệu cho biết không có mục tiêu vị trí khớp nào
    has_goal_joints_ = false;
  }

  // Thiết lập module điều khiển cho tất cả các khớp
  void BaseModule::setCtrlModule(std::string module)
  {
    // Gửi yêu cầu đặt module điều khiển
    std_msgs::String control_msg;
    control_msg.data = module_name_;
    set_ctrl_module_pub_.publish(control_msg);
  }

  // Gọi dịch vụ để thiết lập module điều khiển
  void BaseModule::callServiceSettingModule(const std::string &module_name)
  {
    // Gọi dịch vụ để đặt module điều khiển
    robotis_controller_msgs::SetModule set_module_srv;
    set_module_srv.request.module_name = module_name;

    if (set_module_client_.call(set_module_srv) == false)
    {
      ROS_ERROR("Failed to set module");
      return;
    }

    return;
  }

  // Xuất thông báo trạng thái của module
  void BaseModule::publishStatusMsg(unsigned int type, std::string msg)
  {
    // Tạo và xuất thông báo trạng thái
    robotis_controller_msgs::StatusMsg status_msg;
    status_msg.header.stamp = ros::Time::now();
    status_msg.type = type;
    status_msg.module_name = "Base";
    status_msg.status_msg = msg;

    status_msg_pub_.publish(status_msg);
  }

}

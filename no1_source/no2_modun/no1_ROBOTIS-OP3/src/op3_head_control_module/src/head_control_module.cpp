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

#include <stdio.h>
#include "op3_head_control_module/head_control_module.h"

namespace robotis_op
{

  // hàm khởi tạo
  HeadControlModule::HeadControlModule()
      // Biến lưu chu kỳ kiểm soát (milliseconds)
      : control_cycle_msec_(0),
        // Biến kiểm soát dừng quá trình
        stop_process_(false),
        // Biến kiểm tra trạng thái chuyển động
        is_moving_(false),
        // Cho phép kiểm soát trực tiếp hay không
        is_direct_control_(true),
        // Biến đếm và kích thước của quỹ đạo
        tra_count_(0),
        tra_size_(0),
        // Thời gian di chuyển mặc định
        moving_time_(3.0),
        // Trạng thái quét đầu mặc định là không quét
        scan_state_(NoScan),
        // Kiểm tra xem có mục tiêu vị trí không
        has_goal_position_(false),
        // Góc đơn vị khi quét
        angle_unit_(35),
        // Chế độ debug
        DEBUG(false)
  {
    // Khởi tạo các biến và thuộc tính

    // Module không được kích hoạt khi mới khởi tạo
    enable_ = false;

    // Tên của module
    module_name_ = "head_control_module";

    // Chế độ kiểm soát mặc định là PositionControl
    control_mode_ = robotis_framework::PositionControl;

    // Khởi tạo thông tin cho khớp đầu "head_pan" và "head_tilt"
    result_["head_pan"] = new robotis_framework::DynamixelState();
    result_["head_tilt"] = new robotis_framework::DynamixelState();

    // Xác định các khớp đầu được sử dụng và chỉ số tương ứng của chúng
    using_joint_name_["head_pan"] = 0;
    using_joint_name_["head_tilt"] = 1;

    // Đặt giới hạn góc cho các khớp đầu
    max_angle_[using_joint_name_["head_pan"]] = 85 * DEGREE2RADIAN;
    min_angle_[using_joint_name_["head_pan"]] = -85 * DEGREE2RADIAN;
    max_angle_[using_joint_name_["head_tilt"]] = 30 * DEGREE2RADIAN;
    min_angle_[using_joint_name_["head_tilt"]] = -75 * DEGREE2RADIAN;

    // Khởi tạo ma trận theo dõi vị trí, vận tốc và gia tốc
    target_position_ = Eigen::MatrixXd::Zero(1, result_.size());
    current_position_ = Eigen::MatrixXd::Zero(1, result_.size());
    goal_position_ = Eigen::MatrixXd::Zero(1, result_.size());
    goal_velocity_ = Eigen::MatrixXd::Zero(1, result_.size());
    goal_acceleration_ = Eigen::MatrixXd::Zero(1, result_.size());

    // Thời điểm cuối cùng nhận tin nhắn
    last_msg_time_ = ros::Time::now();
  }

  // Hủy bỏ (destructor)
  HeadControlModule::~HeadControlModule()
  {
    // Kết thúc luồng
    queue_thread_.join();
  }

  // Hàm khởi tạo
  void HeadControlModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
  {
    // Thiết lập các tham số từ các tham số ROS
    ros::NodeHandle param_nh("~");
    angle_unit_ = param_nh.param("angle_unit", 35.0);

    ROS_WARN_STREAM("Head control - angle unit : " << angle_unit_);

    // Tạo và chạy một luồng để xử lý các sự kiện từ ROS
    queue_thread_ = boost::thread(boost::bind(&HeadControlModule::queueThread, this));

    // Lưu giữ chu kỳ kiểm soát
    control_cycle_msec_ = control_cycle_msec;

    ros::NodeHandle ros_node;

    /* publish topics */
    // Xuất dữ liệu trạng thái tới một topic ROS
    status_msg_pub_ = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 0);
  }

  // Hàm xử lý sự kiện từ ROS
  void HeadControlModule::queueThread()
  {
    ros::NodeHandle ros_node;
    ros::CallbackQueue callback_queue;

    ros_node.setCallbackQueue(&callback_queue);

    /* subscribe topics */
    // Đăng ký các topic để lắng nghe sự kiện từ ROS
    ros::Subscriber set_head_joint_sub = ros_node.subscribe("/robotis/head_control/set_joint_states", 1,
                                                            &HeadControlModule::setHeadJointCallback, this);
    ros::Subscriber set_head_joint_offset_sub = ros_node.subscribe("/robotis/head_control/set_joint_states_offset", 1,
                                                                   &HeadControlModule::setHeadJointOffsetCallback, this);
    ros::Subscriber set_head_scan_sub = ros_node.subscribe("/robotis/head_control/scan_command", 1,
                                                           &HeadControlModule::setHeadScanCallback, this);

    ros::WallDuration duration(control_cycle_msec_ / 1000.0);
    while (ros_node.ok())
      callback_queue.callAvailable(duration);
  }

  // Hàm xử lý sự kiện khi nhận được thông tin về trạng thái đầu từ ROS
  void HeadControlModule::setHeadJointCallback(const sensor_msgs::JointState::ConstPtr &msg)
  {
    setHeadJoint(msg, false);
  }

  // Hàm xử lý sự kiện khi nhận được thông tin về trạng thái đầu với offset từ ROS
  void HeadControlModule::setHeadJointOffsetCallback(const sensor_msgs::JointState::ConstPtr &msg)
  {
    setHeadJoint(msg, true);
  }

  // Hàm xử lý sự kiện đặt trạng thái đầu dựa trên thông tin từ ROS
  void HeadControlModule::setHeadJoint(const sensor_msgs::JointState::ConstPtr &msg, bool is_offset)
  {
    // Kiểm tra xem module đã được kích hoạt chưa
    if (enable_ == false)
    {
      ROS_INFO_THROTTLE(1, "Head module is not enable.");
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, "Not Enable");
      return;
    }

    // Chờ đến khi nhận được thông tin về trạng thái đầu
    while (has_goal_position_ == false)
    {
      std::cout << "wait for receiving current position" << std::endl;
      usleep(80 * 1000);
    }

    // Thời gian di chuyển mặc định
    moving_time_ = is_offset ? 0.1 : 1.0; // mặc định: 1 giây

    // Thiết lập góc đầu mục tiêu
    target_position_ = goal_position_; // mặc định

    // Duyệt qua các cấu trúc dữ liệu JointState từ ROS để cập nhật trạng thái đầu
    for (int ix = 0; ix < msg->name.size(); ix++)
    {
      std::string joint_name = msg->name[ix];
      std::map<std::string, int>::iterator joint_it = using_joint_name_.find(joint_name);

      if (joint_it != using_joint_name_.end())
      {
        double target_position = 0.0;
        int joint_index = joint_it->second;

        // Thiết lập vị trí mục tiêu
        if (is_offset == true)
          target_position = goal_position_.coeff(0, joint_index) + msg->position[ix];
        else
          target_position = msg->position[ix];

        // Kiểm tra giới hạn góc
        bool is_checked = checkAngleLimit(joint_index, target_position);
        if (is_checked == false)
        {
          ROS_ERROR_STREAM("Failed to find limit angle \n    id : " << joint_index << ", value : " << (target_position_ * 180 / M_PI));
        }

        // Áp dụng vị trí mục tiêu
        target_position_.coeffRef(0, joint_index) = target_position;

        // Thiết lập thời gian
        double angle_unit = is_offset ? (angle_unit_ * M_PI / 180 * 1.5) : (angle_unit_ * M_PI / 180);
        double calc_moving_time = fabs(goal_position_.coeff(0, joint_index) - target_position_.coeff(0, joint_index)) / angle_unit;
        if (calc_moving_time > moving_time_)
          moving_time_ = calc_moving_time;

        if (DEBUG)
          std::cout << " - joint : " << joint_name << ", Index : " << joint_index << "\n     Target Angle : " << target_position_.coeffRef(0, joint_index) << ", Curr Goal : " << goal_position_.coeff(0, joint_index)
                    << ", Time : " << moving_time_ << ", msg : " << msg->position[ix] << std::endl;
      }
    }

    // Thiết lập chế độ
    is_direct_control_ = true;
    scan_state_ = NoScan;

    // Tạo đường đi chuyển động
    tra_gene_thread_ = new boost::thread(boost::bind(&HeadControlModule::jointTraGeneThread, this));
    delete tra_gene_thread_;
  }

  // Hàm xử lý sự kiện quét đầu dựa trên thông tin từ ROS
  void HeadControlModule::setHeadScanCallback(const std_msgs::String::ConstPtr &msg)
  {
    // Kiểm tra xem module đã được kích hoạt chưa
    if (enable_ == false)
    {
      ROS_ERROR_THROTTLE(1, "Head control module is not enabled, scan command is canceled.");
      return;
    }
    else
      ROS_INFO_THROTTLE(1, "Scan command is accepted. [%d]", scan_state_);

    // Kiểm tra lệnh quét và trạng thái quét hiện tại
    if (msg->data == "scan" && scan_state_ == NoScan)
    {
      std::srand(std::time(NULL));       // Sử dụng thời gian hiện tại làm hạt giống cho bộ tạo số ngẫu nhiên
      scan_state_ = std::rand() % 4 + 1; // Chọn một hướng quét ngẫu nhiên

      is_direct_control_ = false;

      // Tạo đường đi quét
      generateScanTra(scan_state_);
    }
    else if (msg->data == "stop")
    {
      scan_state_ = NoScan; // Dừng quét
    }
  }

  // Hàm kiểm tra giới hạn góc của khớp
  bool HeadControlModule::checkAngleLimit(const int joint_index, double &goal_position)
  {
    // Tìm giới hạn góc tối thiểu cho khớp
    std::map<int, double>::iterator angle_it = min_angle_.find(joint_index);
    if (angle_it == min_angle_.end())
      return false;
    double min_angle = angle_it->second;

    // Tìm giới hạn góc tối đa cho khớp
    angle_it = max_angle_.find(joint_index);
    if (angle_it == max_angle_.end())
      return false;
    double max_angle = angle_it->second;

    // Kiểm tra và điều chỉnh góc mục tiêu nếu vượt quá giới hạn
    if (goal_position < min_angle)
      goal_position = min_angle;
    if (goal_position > max_angle)
      goal_position = max_angle;

    return true;
  }

  // Hàm xử lý module
  void HeadControlModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                                  std::map<std::string, double> sensors)
  {
    // Kiểm tra trạng thái enable
    if (enable_ == false)
      return;

    // Khóa mutex để tránh xung đột
    tra_lock_.lock();

    // Lấy dữ liệu vị trí của các khớp
    for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_it = result_.begin();
         state_it != result_.end(); state_it++)
    {
      std::string joint_name = state_it->first;
      int index = using_joint_name_[joint_name];

      robotis_framework::Dynamixel *_dxl = NULL;
      std::map<std::string, robotis_framework::Dynamixel *>::iterator dxl_it = dxls.find(joint_name);
      if (dxl_it != dxls.end())
        _dxl = dxl_it->second;
      else
        continue;

      current_position_.coeffRef(0, index) = _dxl->dxl_state_->present_position_;
      goal_position_.coeffRef(0, index) = _dxl->dxl_state_->goal_position_;
    }

    has_goal_position_ = true;

    // Kiểm tra để dừng
    if (stop_process_ == true)
    {
      stopMoving();
    }
    else
    {
      // Xử lý
      if (tra_size_ != 0)
      {
        // Bắt đầu các bước
        if (tra_count_ == 0)
        {
          startMoving();
        }

        // Kết thúc các bước
        if (tra_count_ >= tra_size_)
        {
          finishMoving();
        }
        else
        {
          // Cập nhật vị trí mục tiêu
          goal_position_ = calc_joint_tra_.block(tra_count_, 0, 1, result_.size());
          goal_velocity_ = calc_joint_vel_tra_.block(tra_count_, 0, 1, result_.size());
          goal_acceleration_ = calc_joint_accel_tra_.block(tra_count_, 0, 1, result_.size());

          tra_count_ += 1;
        }
      }
    }

    // Mở khóa mutex
    tra_lock_.unlock();

    // Thiết lập dữ liệu vị trí của các khớp
    for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_it = result_.begin();
         state_it != result_.end(); state_it++)
    {
      std::string joint_name = state_it->first;
      int index = using_joint_name_[joint_name];
      double goal_position = goal_position_.coeff(0, index);
      checkAngleLimit(index, goal_position);

      result_[joint_name]->goal_position_ = goal_position;
    }
  }

  // Dừng xử lý
  void HeadControlModule::stop()
  {
    // Khóa mutex để tránh xung đột
    tra_lock_.lock();

    // Kiểm tra nếu đang trong quá trình di chuyển
    if (is_moving_ == true)
      stop_process_ = true;

    // Mở khóa mutex
    tra_lock_.unlock();

    return;
  }

  // Kiểm tra trạng thái đang chạy
  bool HeadControlModule::isRunning()
  {
    return is_moving_;
  }

  // Khi module được bật
  void HeadControlModule::onModuleEnable()
  {
    scan_state_ = NoScan;
  }

  // Khi module bị tắt
  void HeadControlModule::onModuleDisable()
  {
    // Khởi tạo giá trị
    calc_joint_tra_ = goal_position_;
    tra_size_ = 0;
    tra_count_ = 0;
    is_direct_control_ = true;
    is_moving_ = false;
    has_goal_position_ = false;

    goal_velocity_ = Eigen::MatrixXd::Zero(1, result_.size());
    goal_acceleration_ = Eigen::MatrixXd::Zero(1, result_.size());

    scan_state_ = NoScan;

    std::cout << "head_control_module : disable";
  }

  // Bắt đầu quá trình di chuyển
  void HeadControlModule::startMoving()
  {
    is_moving_ = true;

    // Bắt đầu quy trình
  }

  // Kết thúc quá trình di chuyển
  void HeadControlModule::finishMoving()
  {
    // Khởi tạo giá trị
    calc_joint_tra_ = goal_position_;
    tra_size_ = 0;
    tra_count_ = 0;
    is_direct_control_ = true;
    is_moving_ = false;

    goal_velocity_ = Eigen::MatrixXd::Zero(1, result_.size());
    goal_acceleration_ = Eigen::MatrixXd::Zero(1, result_.size());

    // Ghi log
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Head movement is finished.");

    if (DEBUG)
      std::cout << "Trajectory End" << std::endl;

    // Quét đầu
    switch (scan_state_)
    {
    case TopLeft:
      scan_state_ = BottomRight;
      break;

    case BottomRight:
      scan_state_ = BottomLeft;
      break;

    case BottomLeft:
      scan_state_ = TopRight;
      break;

    case TopRight:
      scan_state_ = TopLeft;
      break;

    // Không quét
    default:
      return;
    }

    generateScanTra(scan_state_);
  }

  // Dừng chuyển động của module, khôi phục giá trị và trạng thái ban đầu
  void HeadControlModule::stopMoving()
  {
    // Khởi tạo giá trị của đường đi cần tới
    calc_joint_tra_ = goal_position_;

    // Đặt lại kích thước và chỉ số của đường đi
    tra_size_ = 0;
    tra_count_ = 0;

    // Tắt trạng thái chuyển động
    is_moving_ = false;
    is_direct_control_ = true;

    // Không còn quá trình dừng
    stop_process_ = false;

    // Đặt trạng thái quét về trạng thái ban đầu
    scan_state_ = NoScan;

    // Đặt tốc độ và gia tốc về giá trị ban đầu
    goal_velocity_ = Eigen::MatrixXd::Zero(1, result_.size());
    goal_acceleration_ = Eigen::MatrixXd::Zero(1, result_.size());

    // Ghi log cảnh báo
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_WARN, "Stop Module.");
  }

  // Hàm tạo đường đi quét cho đầu dựa trên hướng chỉ định
  void HeadControlModule::generateScanTra(const int head_direction)
  {
    switch (head_direction)
    {
    case TopLeft:
    {
      // Đặt vị trí đầu và tilt cho hướng quét TopLeft
      target_position_.coeffRef(0, using_joint_name_["head_pan"]) = min_angle_[using_joint_name_["head_pan"]] * 0.6;
      target_position_.coeffRef(0, using_joint_name_["head_tilt"]) = min_angle_[using_joint_name_["head_tilt"]] * 0.25;
      break;
    }

    case TopRight:
    {
      // Đặt vị trí đầu và tilt cho hướng quét TopRight
      target_position_.coeffRef(0, using_joint_name_["head_pan"]) = max_angle_[using_joint_name_["head_pan"]] * 0.6;
      target_position_.coeffRef(0, using_joint_name_["head_tilt"]) = min_angle_[using_joint_name_["head_tilt"]] * 0.25;
      break;
    }

    case BottomLeft:
    {
      // Đặt vị trí đầu và tilt cho hướng quét BottomLeft
      target_position_.coeffRef(0, using_joint_name_["head_pan"]) = min_angle_[using_joint_name_["head_pan"]] * 0.45;
      target_position_.coeffRef(0, using_joint_name_["head_tilt"]) = min_angle_[using_joint_name_["head_tilt"]] * 0.8;
      break;
    }

    case BottomRight:
    {
      // Đặt vị trí đầu và tilt cho hướng quét BottomRight
      target_position_.coeffRef(0, using_joint_name_["head_pan"]) = max_angle_[using_joint_name_["head_pan"]] * 0.45;
      target_position_.coeffRef(0, using_joint_name_["head_tilt"]) = min_angle_[using_joint_name_["head_tilt"]] * 0.8;
      break;
    }

    default:
      return;
    }

    // Đặt thời gian chuyển động mặc định
    moving_time_ = 0.5; // Giá trị mặc định: 0.5 giây

    // Tính toán thời gian chuyển động dựa trên vận tốc góc
    for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_it = result_.begin();
         state_it != result_.end(); state_it++)
    {
      std::string joint_name = state_it->first;
      int index = using_joint_name_[joint_name];

      // Tính toán thời gian
      double calc_moving_time = fabs(goal_position_.coeff(0, index) - target_position_.coeff(0, index)) / (60.0 * M_PI / 180.0);
      if (calc_moving_time > moving_time_)
        moving_time_ = calc_moving_time;
    }

    // Tạo đường đi
    tra_gene_thread_ = new boost::thread(boost::bind(&HeadControlModule::jointTraGeneThread, this));
    delete tra_gene_thread_;
  }

  /*
    Hàm tạo đường đi tối thiểu jerk đơn giản

    pos_start : vị trí ở trạng thái ban đầu
    vel_start : vận tốc ở trạng thái ban đầu
    accel_start : gia tốc ở trạng thái ban đầu

    pos_end : vị trí ở trạng thái cuối
    vel_end : vận tốc ở trạng thái cuối
    accel_end : gia tốc ở trạng thái cuối

    smp_time : thời gian lấy mẫu

    mov_time : thời gian chuyển động
  */
  Eigen::MatrixXd HeadControlModule::calcMinimumJerkTraPVA(double pos_start, double vel_start, double accel_start,
                                                           double pos_end, double vel_end, double accel_end,
                                                           double smp_time, double mov_time)
  {
    // Ma trận và vector đa thức tối thiểu jerk
    Eigen::MatrixXd poly_matrix(3, 3);
    Eigen::MatrixXd poly_vector(3, 1);

    // Xây dựng ma trận đa thức
    poly_matrix << robotis_framework::powDI(mov_time, 3), robotis_framework::powDI(mov_time, 4), robotis_framework::powDI(mov_time, 5),
        3 * robotis_framework::powDI(mov_time, 2), 4 * robotis_framework::powDI(mov_time, 3), 5 * robotis_framework::powDI(mov_time, 4),
        6 * mov_time, 12 * robotis_framework::powDI(mov_time, 2), 20 * robotis_framework::powDI(mov_time, 3);

    // Xây dựng vector đa thức
    poly_vector << pos_end - pos_start - vel_start * mov_time - accel_start * pow(mov_time, 2) / 2,
        vel_end - vel_start - accel_start * mov_time,
        accel_end - accel_start;

    // Tính toán hệ số đa thức
    Eigen::MatrixXd poly_coeff = poly_matrix.inverse() * poly_vector;

    // Số bước thời gian
    int all_time_steps = round(mov_time / smp_time + 1);

    // Ma trận thời gian và đa thức tối thiểu jerk
    Eigen::MatrixXd time = Eigen::MatrixXd::Zero(all_time_steps, 1);
    Eigen::MatrixXd minimum_jer_tra = Eigen::MatrixXd::Zero(all_time_steps, 3);

    // Tính toán đa thức tối thiểu jerk
    for (int step = 0; step < all_time_steps; step++)
      time.coeffRef(step, 0) = step * smp_time;

    for (int step = 0; step < all_time_steps; step++)
    {
      // Vị trí
      minimum_jer_tra.coeffRef(step, 0) = pos_start + vel_start * time.coeff(step, 0) + 0.5 * accel_start * robotis_framework::powDI(time.coeff(step, 0), 2) +
                                          poly_coeff.coeff(0, 0) * robotis_framework::powDI(time.coeff(step, 0), 3) + poly_coeff.coeff(1, 0) * robotis_framework::powDI(time.coeff(step, 0), 4) +
                                          poly_coeff.coeff(2, 0) * robotis_framework::powDI(time.coeff(step, 0), 5);

      // Vận tốc
      minimum_jer_tra.coeffRef(step, 1) = vel_start + accel_start * time.coeff(step, 0) + 3 * poly_coeff.coeff(0, 0) * robotis_framework::powDI(time.coeff(step, 0), 2) +
                                          4 * poly_coeff.coeff(1, 0) * robotis_framework::powDI(time.coeff(step, 0), 3) + 5 * poly_coeff.coeff(2, 0) * robotis_framework::powDI(time.coeff(step, 0), 4);

      // Gia tốc
      minimum_jer_tra.coeffRef(step, 2) = accel_start + 6 * poly_coeff.coeff(0, 0) * time.coeff(step, 0) +
                                          12 * poly_coeff.coeff(1, 0) * robotis_framework::powDI(time.coeff(step, 0), 2) + 20 * poly_coeff.coeff(2, 0) * robotis_framework::powDI(time.coeff(step, 0), 3);
    }

    return minimum_jer_tra;
  }

  // Hàm tạo đường đi cho đầu trong một luồng riêng
  void HeadControlModule::jointTraGeneThread()
  {
    tra_lock_.lock();

    // Chuyển đổi thời gian lấy mẫu từ millisecond sang giây
    double smp_time = control_cycle_msec_ * 0.001; // ms -> s
    // Tính số bước thời gian cần thiết dựa trên thời gian chuyển động và thời gian lấy mẫu
    int all_time_steps = int(moving_time_ / smp_time) + 1;

    // Resize ma trận để lưu trữ các giá trị
    try
    {
      calc_joint_tra_.resize(all_time_steps, result_.size());
      calc_joint_vel_tra_.resize(all_time_steps, result_.size());
      calc_joint_accel_tra_.resize(all_time_steps, result_.size());
    }
    catch (std::exception &e)
    {
      // In ra thông báo lỗi nếu có vấn đề với việc cấp phát bộ nhớ
      std::cout << "All step tile : " << all_time_steps << std::endl;
      std::cout << e.what() << std::endl;
      throw;
    }

    // Lặp qua các động cơ để tạo đường đi
    for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_it = result_.begin();
         state_it != result_.end(); state_it++)
    {
      std::string joint_name = state_it->first;
      int index = using_joint_name_[joint_name];

      // Lấy giá trị và vận tốc ban đầu, gia tốc khởi tạo
      double ini_value = goal_position_.coeff(0, index);
      double ini_vel = goal_velocity_.coeff(0, index);
      double ini_accel = goal_acceleration_.coeff(0, index);

      // Lấy giá trị mục tiêu
      double tar_value = target_position_.coeff(0, index);

      // Tính toán đường đi sử dụng hàm tối thiểu jerk
      Eigen::MatrixXd tra = calcMinimumJerkTraPVA(ini_value, ini_vel, ini_accel, tar_value, 0.0, 0.0, smp_time, moving_time_);

      // Lưu trữ kết quả vào các ma trận tương ứng

      // Gán giá trị từ ma trận tra cho cột thứ 'index' của ma trận calc_joint_tra_
      calc_joint_tra_.block(0, index, all_time_steps, 1) = tra.block(0, 0, all_time_steps, 1);

      // Gán giá trị từ ma trận tra cho cột thứ 'index' của ma trận calc_joint_vel_tra_
      calc_joint_vel_tra_.block(0, index, all_time_steps, 1) = tra.block(0, 1, all_time_steps, 1);

      // Gán giá trị từ ma trận tra cho cột thứ 'index' của ma trận calc_joint_accel_tra_
      calc_joint_accel_tra_.block(0, index, all_time_steps, 1) = tra.block(0, 2, all_time_steps, 1);

    }

    // Cập nhật kích thước và bước đếm cho đường đi
    tra_size_ = calc_joint_tra_.rows();
    tra_count_ = 0;

    // Hiển thị thông báo nếu đang ở chế độ DEBUG
    if (DEBUG)
      ROS_INFO("[ready] make trajectory : %d, %d", tra_size_, tra_count_);

    tra_lock_.unlock();
  }

  // Hàm xuất thông báo trạng thái
  void HeadControlModule::publishStatusMsg(unsigned int type, std::string msg)
  {
    // Lấy thời gian hiện tại
    ros::Time now = ros::Time::now();

    // Kiểm tra xem thông báo có giống với thông báo trước đó không
    if (msg.compare(last_msg_) == 0)
    {
      // Nếu giống, kiểm tra thời gian kể từ lần xuất thông báo trước đó
      ros::Duration dur = now - last_msg_time_;
      // Nếu thời gian nhỏ hơn 1 giây, không xuất thông báo để tránh làm phiền
      if (dur.sec < 1)
        return;
    }

    // Tạo đối tượng StatusMsg và điền thông tin

    // Khai báo đối tượng StatusMsg
    robotis_controller_msgs::StatusMsg status_msg;

    // Gán thời điểm hiện tại cho dấu thời gian của thông điệp (status_msg.header.stamp)
    status_msg.header.stamp = now;

    // Gán loại thông điệp cho đối tượng StatusMsg (status_msg.type)
    status_msg.type = type;

    // Gán tên module cho đối tượng StatusMsg (status_msg.module_name)
    status_msg.module_name = "Head Control";

    // Gán thông điệp trạng thái cho đối tượng StatusMsg (status_msg.status_msg)
    status_msg.status_msg = msg;

    // Xuất thông báo
    status_msg_pub_.publish(status_msg);

    // Cập nhật thông báo và thời gian
    last_msg_ = msg;
    last_msg_time_ = now;
  }

}

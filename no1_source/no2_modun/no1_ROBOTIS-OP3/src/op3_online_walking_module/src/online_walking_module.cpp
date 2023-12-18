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

#include "op3_online_walking_module/online_walking_module.h"

using namespace robotis_op;

// Constructor của lớp OnlineWalkingModule
OnlineWalkingModule::OnlineWalkingModule()
    // Chu kỳ kiểm soát (thời gian giữa các lần kiểm soát liên tiếp)
    : control_cycle_sec_(0.008),

      // Biến đánh dấu trạng thái di chuyển của robot
      is_moving_(false),

      // Biến đánh dấu trạng thái cân bằng của robot
      is_balancing_(false),

      // Biến đánh dấu trạng thái cập nhật offset của robot
      is_offset_updating_(false),

      // Biến đánh dấu trạng thái khởi tạo mục tiêu của robot
      goal_initialize_(false),

      // Biến đánh dấu trạng thái khởi tạo kiểm soát cân bằng
      balance_control_initialize_(false),

      // Biến đánh dấu trạng thái khởi tạo offset của cơ thể
      body_offset_initialize_(false),

      // Biến đánh dấu trạng thái khởi tạo kiểm soát khớp
      joint_control_initialize_(false),

      // Biến đánh dấu trạng thái khởi tạo kiểm soát toàn bộ cơ thể
      wholebody_initialize_(false),

      // Biến đánh dấu trạng thái khởi tạo kiểm soát đi bộ
      walking_initialize_(false),

      // Biến đánh dấu trạng thái sử dụng bước chân 2D hay không
      is_foot_step_2d_(false),

      // Giai đoạn đi bộ hiện tại (DSP hoặc SSP)
      walking_phase_(DSP),

      // Tổng khối lượng của robot
      total_mass_(3.5),

      // Khoảng cách giữa hai chân của robot
      foot_distance_(0.07)

{
  // Thân hàm constructor, có thể thêm các xử lý khởi tạo khác nếu cần

  enable_ = false;
  module_name_ = "online_walking_module";
  control_mode_ = robotis_framework::PositionControl;
  control_type_ = NONE;
  balance_type_ = OFF;

  op3_kdl_ = new OP3Kinematics();

  /* leg */
  result_["r_hip_yaw"] = new robotis_framework::DynamixelState();
  result_["r_hip_roll"] = new robotis_framework::DynamixelState();
  result_["r_hip_pitch"] = new robotis_framework::DynamixelState();
  result_["r_knee"] = new robotis_framework::DynamixelState();
  result_["r_ank_pitch"] = new robotis_framework::DynamixelState();
  result_["r_ank_roll"] = new robotis_framework::DynamixelState();
  result_["l_hip_yaw"] = new robotis_framework::DynamixelState();
  result_["l_hip_roll"] = new robotis_framework::DynamixelState();
  result_["l_hip_pitch"] = new robotis_framework::DynamixelState();
  result_["l_knee"] = new robotis_framework::DynamixelState();
  result_["l_ank_pitch"] = new robotis_framework::DynamixelState();
  result_["l_ank_roll"] = new robotis_framework::DynamixelState();

  /* leg */
  joint_name_to_id_["r_hip_yaw"] = 1;
  joint_name_to_id_["l_hip_yaw"] = 2;
  joint_name_to_id_["r_hip_roll"] = 3;
  joint_name_to_id_["l_hip_roll"] = 4;
  joint_name_to_id_["r_hip_pitch"] = 5;
  joint_name_to_id_["l_hip_pitch"] = 6;
  joint_name_to_id_["r_knee"] = 7;
  joint_name_to_id_["l_knee"] = 8;
  joint_name_to_id_["r_ank_pitch"] = 9;
  joint_name_to_id_["l_ank_pitch"] = 10;
  joint_name_to_id_["r_ank_roll"] = 11;
  joint_name_to_id_["l_ank_roll"] = 12;

  /* parameter */
  /* Khai báo và khởi tạo thông số cho các khớp trong hệ thống robot hoặc cơ cấu cơ khí */
  number_of_joints_ = 12;

  // Các thông số hiện tại của các khớp
  curr_joint_accel_.resize(number_of_joints_, 0.0);
  curr_joint_vel_.resize(number_of_joints_, 0.0);
  curr_joint_pos_.resize(number_of_joints_, 0.0);

  // Các thông số mục tiêu mong muốn của các khớp
  des_joint_accel_.resize(number_of_joints_, 0.0);
  des_joint_vel_.resize(number_of_joints_, 0.0);
  des_joint_pos_.resize(number_of_joints_, 0.0);

  // Các thông số mục tiêu cuối cùng mong muốn của các khớp
  goal_joint_accel_.resize(number_of_joints_, 0.0);
  goal_joint_vel_.resize(number_of_joints_, 0.0);
  goal_joint_pos_.resize(number_of_joints_, 0.0);

  // Các thông số phản hồi và điều khiển thêm cho các khớp
  des_joint_feedback_.resize(number_of_joints_, 0.0);
  des_joint_feedforward_.resize(number_of_joints_, 0.0);
  des_joint_pos_to_robot_.resize(number_of_joints_, 0.0);

  // Hệ số điều khiển thêm cho mỗi khớp
  joint_feedforward_gain_.resize(number_of_joints_, 0.0);

  // Vị trí mặc định của cơ thể
  des_body_pos_.resize(3, 0.0);   // Vị trí
  des_body_vel_.resize(3, 0.0);   // Vận tốc
  des_body_accel_.resize(3, 0.0); // Gia tốc
  des_body_Q_.resize(4, 0.0);     // Quaternion

  // Vị trí mặc định của chân trái
  des_l_leg_pos_.resize(3, 0.0);   // Vị trí
  des_l_leg_vel_.resize(3, 0.0);   // Vận tốc
  des_l_leg_accel_.resize(3, 0.0); // Gia tốc
  des_l_leg_Q_.resize(4, 0.0);     // Quaternion

  // Vị trí mặc định của chân phải
  des_r_leg_pos_.resize(3, 0.0);   // Vị trí
  des_r_leg_vel_.resize(3, 0.0);   // Vận tốc
  des_r_leg_accel_.resize(3, 0.0); // Gia tốc
  des_r_leg_Q_.resize(4, 0.0);     // Quaternion

  // Vị trí của LIPM (Linear Inverted Pendulum Model)
  x_lipm_.resize(3, 0.0);
  y_lipm_.resize(3, 0.0);

  // Thiết lập lại tư thế cơ thể mặc định
  resetBodyPose();

  // walking parameter default
  // Tham số mặc định cho việc đi bộ
  walking_param_.dsp_ratio = 0.2;
  walking_param_.lipm_height = 0.12;
  walking_param_.foot_height_max = 0.05;
  walking_param_.zmp_offset_x = 0.0; // không áp dụng
  walking_param_.zmp_offset_y = 0.0;

  // Tỷ lệ cân bằng mong muốn
  des_balance_gain_ratio_.resize(1, 0.0);
  goal_balance_gain_ratio_.resize(1, 0.0);

  // Khởi tạo điều khiển cân bằng
  balance_control_.initialize(control_cycle_sec_ * 1000.0);
  balance_control_.setGyroBalanceEnable(false);        // Gyro
  balance_control_.setOrientationBalanceEnable(false); // IMU
  balance_control_.setForceTorqueBalanceEnable(false); // FT

  // Cân bằng lực và mô-men chân trái
  balance_l_foot_force_x_ = 0.0;
  balance_l_foot_force_y_ = 0.0;
  balance_l_foot_force_z_ = 0.0;
  balance_l_foot_torque_x_ = 0.0;
  balance_l_foot_torque_y_ = 0.0;
  balance_l_foot_torque_z_ = 0.0;

  // Cân bằng lực và mô-men chân phải
  balance_r_foot_force_x_ = 0.0;
  balance_r_foot_force_y_ = 0.0;
  balance_r_foot_force_z_ = 0.0;
  balance_r_foot_torque_x_ = 0.0;
  balance_r_foot_torque_y_ = 0.0;
  balance_r_foot_torque_z_ = 0.0;

  // Body Offset
  // Độ lệch cơ thể
  des_body_offset_.resize(3, 0.0);
  goal_body_offset_.resize(3, 0.0);

  // Đường dẫn cho các thông số cân bằng
  std::string balance_gain_path = ros::package::getPath("op3_online_walking_module") + "/config/balance_gain.yaml";
  parseBalanceGainData(balance_gain_path);

  // Đường dẫn cho các thông số phản hồi khớp
  std::string joint_feedback_gain_path = ros::package::getPath("op3_online_walking_module") + "/config/joint_feedback_gain.yaml";
  parseJointFeedbackGainData(joint_feedback_gain_path);

  // Đường dẫn cho các thông số điều khiển thêm khớp
  std::string joint_feedforward_gain_path = ros::package::getPath("op3_online_walking_module") + "/config/joint_feedforward_gain.yaml";
  parseJointFeedforwardGainData(joint_feedforward_gain_path);
}

// Hàm hủy của OnlineWalkingModule
OnlineWalkingModule::~OnlineWalkingModule()
{
  // Đợi kết thúc luồng hàng đợi
  queue_thread_.join();
}

// Hàm khởi tạo của OnlineWalkingModule
void OnlineWalkingModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  // Thiết lập chu kỳ điều khiển và bắt đầu luồng hàng đợi
  control_cycle_sec_ = control_cycle_msec * 0.001;
  queue_thread_ = boost::thread(boost::bind(&OnlineWalkingModule::queueThread, this));

  // Tạo một NodeHandle ROS
  ros::NodeHandle ros_node;

  // Publisher
  status_msg_pub_ = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 1);
  movement_done_pub_ = ros_node.advertise<std_msgs::String>("/robotis/movement_done", 1);
  goal_joint_state_pub_ = ros_node.advertise<sensor_msgs::JointState>("/robotis/online_walking/goal_joint_states", 1);
  pelvis_pose_pub_ = ros_node.advertise<geometry_msgs::PoseStamped>("/robotis/pelvis_pose", 1);

  // Service (chưa được sử dụng trong đoạn mã)
  // get_preview_matrix_client_ = ros_node.serviceClient<op3_online_walking_module_msgs::GetPreviewMatrix>("/robotis/online_walking/get_preview_matrix", 0);
}

// Hàm xử lý hàng đợi của OnlineWalkingModule
void OnlineWalkingModule::queueThread()
{
  // Tạo NodeHandle và CallbackQueue mới
  ros::NodeHandle ros_node;
  ros::CallbackQueue callback_queue;
  ros_node.setCallbackQueue(&callback_queue);

  // Đăng ký các Subscriber cho các topic ROS cần lắng nghe
  ros::Subscriber reset_body_sub_ = ros_node.subscribe("/robotis/online_walking/reset_body", 5,
                                                       &OnlineWalkingModule::setResetBodyCallback, this);
  ros::Subscriber joint_pose_sub_ = ros_node.subscribe("/robotis/online_walking/goal_joint_pose", 5,
                                                       &OnlineWalkingModule::goalJointPoseCallback, this);
  ros::Subscriber kinematics_pose_sub_ = ros_node.subscribe("/robotis/online_walking/goal_kinematics_pose", 5,
                                                            &OnlineWalkingModule::goalKinematicsPoseCallback, this);
  ros::Subscriber foot_step_command_sub_ = ros_node.subscribe("/robotis/online_walking/foot_step_command", 5,
                                                              &OnlineWalkingModule::footStepCommandCallback, this);
  ros::Subscriber walking_param_sub_ = ros_node.subscribe("/robotis/online_walking/walking_param", 5,
                                                          &OnlineWalkingModule::walkingParamCallback, this);
  ros::Subscriber wholebody_balance_msg_sub = ros_node.subscribe("/robotis/online_walking/wholebody_balance_msg", 5,
                                                                 &OnlineWalkingModule::setWholebodyBalanceMsgCallback, this);
  ros::Subscriber body_offset_msg_sub = ros_node.subscribe("/robotis/online_walking/body_offset", 5,
                                                           &OnlineWalkingModule::setBodyOffsetCallback, this);
  ros::Subscriber foot_distance_msg_sub = ros_node.subscribe("/robotis/online_walking/foot_distance", 5,
                                                             &OnlineWalkingModule::setFootDistanceCallback, this);
  ros::Subscriber footsteps_sub = ros_node.subscribe("/robotis/online_walking/footsteps_2d", 5,
                                                     &OnlineWalkingModule::footStep2DCallback, this);

  // Các Subscriber và Service (chưa được sử dụng trong đoạn mã)
  //  ros::Subscriber imu_data_sub = ros_node.subscribe("/robotis/sensor/imu/imu", 5,
  //                                                    &OnlineWalkingModule::imuDataCallback, this);
  //  ros::Subscriber l_foot_ft_sub = ros_node.subscribe("/robotis/sensor/l_foot_ft", 3,
  //                                                     &OnlineWalkingModule::leftFootForceTorqueOutputCallback, this);
  //  ros::Subscriber r_foot_ft_sub = ros_node.subscribe("/robotis/sensor/r_foot_ft", 3,
  //                                                     &OnlineWalkingModule::rightFootForceTorqueOutputCallback, this);

  // Service
  ros::ServiceServer get_joint_pose_server = ros_node.advertiseService("/robotis/online_walking/get_joint_pose",
                                                                       &OnlineWalkingModule::getJointPoseCallback, this);
  ros::ServiceServer get_kinematics_pose_server = ros_node.advertiseService("/robotis/online_walking/get_kinematics_pose",
                                                                            &OnlineWalkingModule::getKinematicsPoseCallback, this);

  // Đăng ký dịch vụ cho các Subscriber đã được kích hoạt
  ros::WallDuration duration(control_cycle_sec_);
  while (ros_node.ok())
    callback_queue.callAvailable(duration);
}

// Phương thức đặt lại tư thế cơ thể (resetBodyPose) của lớp OnlineWalkingModule
void OnlineWalkingModule::resetBodyPose()
{
  // Đặt lại vị trí và quay của cơ thể
  des_body_pos_[0] = 0.0;
  des_body_pos_[1] = 0.0;
  des_body_pos_[2] = 0.3402256;

  des_body_Q_[0] = 0.0;
  des_body_Q_[1] = 0.0;
  des_body_Q_[2] = 0.0;
  des_body_Q_[3] = 1.0;

  // Đặt lại vị trí và quay của chân phải
  des_r_leg_pos_[0] = 0.0;
  des_r_leg_pos_[1] = -0.5 * foot_distance_; // -0.045; // -0.035;
  des_r_leg_pos_[2] = 0.0;

  des_r_leg_Q_[0] = 0.0;
  des_r_leg_Q_[1] = 0.0;
  des_r_leg_Q_[2] = 0.0;
  des_r_leg_Q_[3] = 1.0;

  // Đặt lại vị trí và quay của chân trái
  des_l_leg_pos_[0] = 0.0;
  des_l_leg_pos_[1] = 0.5 * foot_distance_; // 0.045; // 0.035;
  des_l_leg_pos_[2] = 0.0;

  des_l_leg_Q_[0] = 0.0;
  des_l_leg_Q_[1] = 0.0;
  des_l_leg_Q_[2] = 0.0;
  des_l_leg_Q_[3] = 1.0;

  // Đặt lại vị trí của LIPM (Linear Inverted Pendulum Model)
  x_lipm_[0] = des_body_pos_[0];
  x_lipm_[1] = 0.0;
  x_lipm_[2] = 0.0;

  y_lipm_[0] = des_body_pos_[1];
  y_lipm_[1] = 0.0;
  y_lipm_[2] = 0.0;

  // Đặt lại giá trị của tham số zmp_offset_x trong tham số đi bộ
  walking_param_.zmp_offset_x = des_body_pos_[0];
}

// Phương thức phân tích dữ liệu Balance Gain từ file YAML
void OnlineWalkingModule::parseBalanceGainData(const std::string &path)
{
  YAML::Node doc;
  try
  {
    // Load dữ liệu YAML từ file
    doc = YAML::LoadFile(path.c_str());
  }
  catch (const std::exception &e)
  {
    // Ghi log lỗi nếu không thể load file YAML
    ROS_ERROR("Fail to load YAML file at path: %s. Error: %s", path.c_str(), e.what());
    return;
  }

  // Phân tích dữ liệu từ YAML và gán giá trị cho các biến thành viên

  // Gain cho giảm chấn của gyro và angle (proportional và derivative)
  foot_roll_gyro_p_gain_ = doc["foot_roll_gyro_p_gain"].as<double>();
  foot_roll_gyro_d_gain_ = doc["foot_roll_gyro_d_gain"].as<double>();
  foot_pitch_gyro_p_gain_ = doc["foot_pitch_gyro_p_gain"].as<double>();
  foot_pitch_gyro_d_gain_ = doc["foot_pitch_gyro_d_gain"].as<double>();

  foot_roll_angle_p_gain_ = doc["foot_roll_angle_p_gain"].as<double>();
  foot_roll_angle_d_gain_ = doc["foot_roll_angle_d_gain"].as<double>();
  foot_pitch_angle_p_gain_ = doc["foot_pitch_angle_p_gain"].as<double>();
  foot_pitch_angle_d_gain_ = doc["foot_pitch_angle_d_gain"].as<double>();

  // Gain cho lực và moment tác động lên chân (proportional và derivative)
  foot_x_force_p_gain_ = doc["foot_x_force_p_gain"].as<double>();
  foot_x_force_d_gain_ = doc["foot_x_force_d_gain"].as<double>();
  foot_y_force_p_gain_ = doc["foot_y_force_p_gain"].as<double>();
  foot_y_force_d_gain_ = doc["foot_y_force_d_gain"].as<double>();
  foot_z_force_p_gain_ = doc["foot_z_force_p_gain"].as<double>();
  foot_z_force_d_gain_ = doc["foot_z_force_d_gain"].as<double>();

  foot_roll_torque_p_gain_ = doc["foot_roll_torque_p_gain"].as<double>();
  foot_roll_torque_d_gain_ = doc["foot_roll_torque_d_gain"].as<double>();
  foot_pitch_torque_p_gain_ = doc["foot_pitch_torque_p_gain"].as<double>();
  foot_pitch_torque_d_gain_ = doc["foot_pitch_torque_d_gain"].as<double>();

  // Các thông số cắt bớt tần số cho các sensor
  roll_gyro_cut_off_frequency_ = doc["roll_gyro_cut_off_frequency"].as<double>();
  pitch_gyro_cut_off_frequency_ = doc["pitch_gyro_cut_off_frequency"].as<double>();

  roll_angle_cut_off_frequency_ = doc["roll_angle_cut_off_frequency"].as<double>();
  pitch_angle_cut_off_frequency_ = doc["pitch_angle_cut_off_frequency"].as<double>();

  foot_x_force_cut_off_frequency_ = doc["foot_x_force_cut_off_frequency"].as<double>();
  foot_y_force_cut_off_frequency_ = doc["foot_y_force_cut_off_frequency"].as<double>();
  foot_z_force_cut_off_frequency_ = doc["foot_z_force_cut_off_frequency"].as<double>();

  foot_roll_torque_cut_off_frequency_ = doc["foot_roll_torque_cut_off_frequency"].as<double>();
  foot_pitch_torque_cut_off_frequency_ = doc["foot_pitch_torque_cut_off_frequency"].as<double>();

  // Các hệ số gain cho cân bằng của các khớp
  balance_hip_roll_gain_ = doc["balance_hip_roll_gain"].as<double>();
  balance_knee_gain_ = doc["balance_knee_gain"].as<double>();
  balance_ankle_roll_gain_ = doc["balance_ankle_roll_gain"].as<double>();
  balance_ankle_pitch_gain_ = doc["balance_ankle_pitch_gain"].as<double>();
}

// Phương thức phân tích dữ liệu Joint Feedback Gain từ file YAML
void OnlineWalkingModule::parseJointFeedbackGainData(const std::string &path)
{
  YAML::Node doc;
  try
  {
    // Load dữ liệu YAML từ file
    doc = YAML::LoadFile(path.c_str());
  }
  catch (const std::exception &e)
  {
    // Ghi log lỗi nếu không thể load file YAML
    ROS_ERROR("Fail to load YAML file at path: %s. Error: %s", path.c_str(), e.what());
    return;
  }

  // Gán giá trị cho các biến thành viên trong mảng joint_feedback_

  // Joint feedback gain cho các khớp của chân phải
  joint_feedback_[joint_name_to_id_["r_hip_yaw"] - 1].p_gain_ = doc["r_hip_yaw_p_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["r_hip_yaw"] - 1].d_gain_ = doc["r_hip_yaw_d_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["r_hip_roll"] - 1].p_gain_ = doc["r_hip_roll_p_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["r_hip_roll"] - 1].d_gain_ = doc["r_hip_roll_d_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["r_hip_pitch"] - 1].p_gain_ = doc["r_hip_pitch_p_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["r_hip_pitch"] - 1].d_gain_ = doc["r_hip_pitch_d_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["r_knee"] - 1].p_gain_ = doc["r_knee_p_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["r_knee"] - 1].d_gain_ = doc["r_knee_d_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["r_ank_pitch"] - 1].p_gain_ = doc["r_ank_pitch_p_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["r_ank_pitch"] - 1].d_gain_ = doc["r_ank_pitch_d_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["r_ank_roll"] - 1].p_gain_ = doc["r_ank_roll_p_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["r_ank_roll"] - 1].d_gain_ = doc["r_ank_roll_d_gain"].as<double>();

  // Joint feedback gain cho các khớp của chân trái
  joint_feedback_[joint_name_to_id_["l_hip_yaw"] - 1].p_gain_ = doc["l_hip_yaw_p_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["l_hip_yaw"] - 1].d_gain_ = doc["l_hip_yaw_d_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["l_hip_roll"] - 1].p_gain_ = doc["l_hip_roll_p_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["l_hip_roll"] - 1].d_gain_ = doc["l_hip_roll_d_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["l_hip_pitch"] - 1].p_gain_ = doc["l_hip_pitch_p_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["l_hip_pitch"] - 1].d_gain_ = doc["l_hip_pitch_d_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["l_knee"] - 1].p_gain_ = doc["l_knee_p_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["l_knee"] - 1].d_gain_ = doc["l_knee_d_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["l_ank_pitch"] - 1].p_gain_ = doc["l_ank_pitch_p_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["l_ank_pitch"] - 1].d_gain_ = doc["l_ank_pitch_d_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["l_ank_roll"] - 1].p_gain_ = doc["l_ank_roll_p_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["l_ank_roll"] - 1].d_gain_ = doc["l_ank_roll_d_gain"].as<double>();
}

// Phương thức phân tích dữ liệu Joint Feedforward Gain từ file YAML
void OnlineWalkingModule::parseJointFeedforwardGainData(const std::string &path)
{
  YAML::Node doc;
  try
  {
    // Load dữ liệu YAML từ file
    doc = YAML::LoadFile(path.c_str());
  }
  catch (const std::exception &e)
  {
    // Ghi log lỗi nếu không thể load file YAML
    ROS_ERROR("Fail to load YAML file at path: %s. Error: %s", path.c_str(), e.what());
    return;
  }

  // Gán giá trị cho các biến thành viên trong mảng joint_feedforward_gain_

  // Joint feedforward gain cho các khớp của chân phải
  joint_feedforward_gain_[joint_name_to_id_["r_hip_yaw"] - 1] = doc["r_hip_yaw_gain"].as<double>();
  joint_feedforward_gain_[joint_name_to_id_["r_hip_roll"] - 1] = doc["r_hip_roll_gain"].as<double>();
  joint_feedforward_gain_[joint_name_to_id_["r_hip_pitch"] - 1] = doc["r_hip_pitch_gain"].as<double>();
  joint_feedforward_gain_[joint_name_to_id_["r_knee"] - 1] = doc["r_knee_gain"].as<double>();
  joint_feedforward_gain_[joint_name_to_id_["r_ank_pitch"] - 1] = doc["r_ank_pitch_gain"].as<double>();
  joint_feedforward_gain_[joint_name_to_id_["r_ank_roll"] - 1] = doc["r_ank_roll_gain"].as<double>();

  // Joint feedforward gain cho các khớp của chân trái
  joint_feedforward_gain_[joint_name_to_id_["l_hip_yaw"] - 1] = doc["l_hip_yaw_gain"].as<double>();
  joint_feedforward_gain_[joint_name_to_id_["l_hip_roll"] - 1] = doc["l_hip_roll_gain"].as<double>();
  joint_feedforward_gain_[joint_name_to_id_["l_hip_pitch"] - 1] = doc["l_hip_pitch_gain"].as<double>();
  joint_feedforward_gain_[joint_name_to_id_["l_knee"] - 1] = doc["l_knee_gain"].as<double>();
  joint_feedforward_gain_[joint_name_to_id_["l_ank_pitch"] - 1] = doc["l_ank_pitch_gain"].as<double>();
  joint_feedforward_gain_[joint_name_to_id_["l_ank_roll"] - 1] = doc["l_ank_roll_gain"].as<double>();
}

// Phương thức callback xử lý thông điệp cân bằng toàn bộ cơ thể
void OnlineWalkingModule::setWholebodyBalanceMsgCallback(const std_msgs::String::ConstPtr &msg)
{
  // Kiểm tra trạng thái hoạt động của module
  if (enable_ == false)
    return;

  // Lấy đường dẫn đến file cấu hình cân bằng
  std::string balance_gain_path = ros::package::getPath("op3_online_walking_module") + "/config/balance_gain.yaml";
  // Phân tích và cập nhật dữ liệu cân bằng từ file YAML
  parseBalanceGainData(balance_gain_path);

  // Lấy đường dẫn đến file cấu hình thông số phản hồi khớp
  std::string joint_feedback_gain_path = ros::package::getPath("op3_online_walking_module") + "/config/joint_feedback_gain.yaml";
  // Phân tích và cập nhật dữ liệu phản hồi khớp từ file YAML
  parseJointFeedbackGainData(joint_feedback_gain_path);

  // Lấy đường dẫn đến file cấu hình thông số feedforward khớp
  std::string joint_feedforward_gain_path = ros::package::getPath("op3_online_walking_module") + "/config/joint_feedforward_gain.yaml";
  // Phân tích và cập nhật dữ liệu feedforward khớp từ file YAML
  parseJointFeedforwardGainData(joint_feedforward_gain_path);

  // Xử lý theo nội dung thông điệp
  if (msg->data == "balance_on")
    goal_balance_gain_ratio_[0] = 1.0;
  else if (msg->data == "balance_off")
    goal_balance_gain_ratio_[0] = 0.0;

  // Đặt lại trạng thái cân bằng và giai đoạn đi bộ
  balance_control_initialize_ = false;
  balance_type_ = ON;
  walking_phase_ = DSP;
}

// Phương thức khởi tạo điều khiển cân bằng
void OnlineWalkingModule::initBalanceControl()
{
  // Nếu điều khiển cân bằng đã được khởi tạo, không thực hiện gì
  if (balance_control_initialize_ == true)
    return;

  // Đánh dấu rằng điều khiển cân bằng đã được khởi tạo
  balance_control_initialize_ = true;

  // Thời gian khởi tạo và thời gian chuyển động
  double ini_time = 0.0;
  double mov_time = 1.0;

  // Đặt lại bước và kích thước cân bằng
  balance_step_ = 0;
  balance_size_ = static_cast<int>((mov_time / control_cycle_sec_) + 1);

  // Chuẩn bị vector chứa giá trị zero
  std::vector<double_t> balance_zero;
  balance_zero.resize(1, 0.0);

  // Tạo đối tượng MinimumJerk để tạo chuỗi chuyển động cân bằng
  balance_tra_ =
      new robotis_framework::MinimumJerk(ini_time, mov_time,
                                         des_balance_gain_ratio_, balance_zero, balance_zero,
                                         goal_balance_gain_ratio_, balance_zero, balance_zero);

  // Nếu đang trong quá trình cân bằng, ghi log cập nhật. Ngược lại, ghi log bắt đầu cân bằng.
  if (is_balancing_ == true)
    ROS_INFO("[UPDATE] Balance Gain");
  else
  {
    is_balancing_ = true;
    ROS_INFO("[START] Balance Gain");
  }
}

// Phương thức tính toán điều khiển cân bằng
void OnlineWalkingModule::calcBalanceControl()
{
  // Nếu đang trong quá trình cân bằng
  if (is_balancing_ == true)
  {
    // Lấy thời gian hiện tại dựa trên bước và chu kỳ kiểm soát
    double cur_time = static_cast<double>(balance_step_) * control_cycle_sec_;
    // Cập nhật tỷ lệ cân bằng dựa trên vị trí tại thời điểm hiện tại
    des_balance_gain_ratio_ = balance_tra_->getPosition(cur_time);

    // Nếu đã đến cuối chuỗi cân bằng
    if (balance_step_ == balance_size_ - 1)
    {
      // Đặt lại bước và đánh dấu kết thúc quá trình cân bằng
      balance_step_ = 0;
      is_balancing_ = false;
      // Giải phóng bộ nhớ cho đối tượng chuỗi chuyển động cân bằng
      delete balance_tra_;

      // Nếu tỷ lệ cân bằng cuối cùng là 0.0, đặt lại loại điều khiển và trạng thái cân bằng
      if (des_balance_gain_ratio_[0] == 0.0)
      {
        control_type_ = NONE;
        balance_type_ = OFF;
      }

      ROS_INFO("[END] Balance Gain");
    }
    else
      // Nếu chưa đến cuối chuỗi cân bằng, tăng bước lên
      balance_step_++;
  }
}

// Callback khi nhận dữ liệu từ cảm biến IMU
void OnlineWalkingModule::imuDataCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
  // Khóa mutex để đảm bảo an toàn khi truy cập dữ liệu IMU
  imu_data_mutex_lock_.lock();

  // Lưu trữ dữ liệu IMU nhận được
  imu_data_msg_ = *msg;

  // Đảo chiều dữ liệu về tốc độ góc theo trục x và y để phù hợp với hệ tọa độ của robot
  imu_data_msg_.angular_velocity.x *= -1.0;
  imu_data_msg_.angular_velocity.y *= -1.0;

  // Mở khóa mutex
  imu_data_mutex_lock_.unlock();
}

// Callback khi nhận dữ liệu lực và mô-men xoắn từ cảm biến lực chân trái
void OnlineWalkingModule::leftFootForceTorqueOutputCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
  // Khởi tạo ma trận lực và mô-men xoắn từ thông điệp nhận được
  Eigen::MatrixXd force = Eigen::MatrixXd::Zero(3, 1);
  force.coeffRef(0, 0) = msg->wrench.force.x;
  force.coeffRef(1, 0) = msg->wrench.force.y;
  force.coeffRef(2, 0) = msg->wrench.force.z;

  Eigen::MatrixXd torque = Eigen::MatrixXd::Zero(3, 1);
  torque.coeffRef(0, 0) = msg->wrench.torque.x;
  torque.coeffRef(1, 0) = msg->wrench.torque.y;
  torque.coeffRef(2, 0) = msg->wrench.torque.z;

  // Chuyển đổi hệ tọa độ của lực và mô-men xoắn
  Eigen::MatrixXd force_new = robotis_framework::getRotationX(M_PI) * robotis_framework::getRotationZ(-0.5 * M_PI) * force;
  Eigen::MatrixXd torque_new = robotis_framework::getRotationX(M_PI) * robotis_framework::getRotationZ(-0.5 * M_PI) * torque;

  // Giới hạn giá trị lực và mô-men xoắn
  double l_foot_fx_N = robotis_framework::sign(force_new.coeff(0, 0)) * fmin(fabs(force_new.coeff(0, 0)), 2000.0);
  double l_foot_fy_N = robotis_framework::sign(force_new.coeff(1, 0)) * fmin(fabs(force_new.coeff(1, 0)), 2000.0);
  double l_foot_fz_N = robotis_framework::sign(force_new.coeff(2, 0)) * fmin(fabs(force_new.coeff(2, 0)), 2000.0);
  double l_foot_Tx_Nm = robotis_framework::sign(torque_new.coeff(0, 0)) * fmin(fabs(torque_new.coeff(0, 0)), 300.0);
  double l_foot_Ty_Nm = robotis_framework::sign(torque_new.coeff(1, 0)) * fmin(fabs(torque_new.coeff(1, 0)), 300.0);
  double l_foot_Tz_Nm = robotis_framework::sign(torque_new.coeff(2, 0)) * fmin(fabs(torque_new.coeff(2, 0)), 300.0);

  // Lưu trữ dữ liệu lực và mô-men xoắn sau khi xử lý
  l_foot_ft_data_msg_.force.x = l_foot_fx_N;
  l_foot_ft_data_msg_.force.y = l_foot_fy_N;
  l_foot_ft_data_msg_.force.z = l_foot_fz_N;
  l_foot_ft_data_msg_.torque.x = l_foot_Tx_Nm;
  l_foot_ft_data_msg_.torque.y = l_foot_Ty_Nm;
  l_foot_ft_data_msg_.torque.z = l_foot_Tz_Nm;
}

// Callback khi nhận dữ liệu lực và mô-men xoắn từ cảm biến lực chân phải
void OnlineWalkingModule::rightFootForceTorqueOutputCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
  // Khởi tạo ma trận lực và mô-men xoắn từ thông điệp nhận được
  Eigen::MatrixXd force = Eigen::MatrixXd::Zero(3, 1);
  force.coeffRef(0, 0) = msg->wrench.force.x;
  force.coeffRef(1, 0) = msg->wrench.force.y;
  force.coeffRef(2, 0) = msg->wrench.force.z;

  Eigen::MatrixXd torque = Eigen::MatrixXd::Zero(3, 1);
  torque.coeffRef(0, 0) = msg->wrench.torque.x;
  torque.coeffRef(1, 0) = msg->wrench.torque.y;
  torque.coeffRef(2, 0) = msg->wrench.torque.z;

  // Chuyển đổi hệ tọa độ của lực và mô-men xoắn
  Eigen::MatrixXd force_new = robotis_framework::getRotationX(M_PI) * robotis_framework::getRotationZ(-0.5 * M_PI) * force;
  Eigen::MatrixXd torque_new = robotis_framework::getRotationX(M_PI) * robotis_framework::getRotationZ(-0.5 * M_PI) * torque;

  // Giới hạn giá trị lực và mô-men xoắn
  double r_foot_fx_N = robotis_framework::sign(force_new.coeff(0, 0)) * fmin(fabs(force_new.coeff(0, 0)), 2000.0);
  double r_foot_fy_N = robotis_framework::sign(force_new.coeff(1, 0)) * fmin(fabs(force_new.coeff(1, 0)), 2000.0);
  double r_foot_fz_N = robotis_framework::sign(force_new.coeff(2, 0)) * fmin(fabs(force_new.coeff(2, 0)), 2000.0);
  double r_foot_Tx_Nm = robotis_framework::sign(torque_new.coeff(0, 0)) * fmin(fabs(torque_new.coeff(0, 0)), 300.0);
  double r_foot_Ty_Nm = robotis_framework::sign(torque_new.coeff(1, 0)) * fmin(fabs(torque_new.coeff(1, 0)), 300.0);
  double r_foot_Tz_Nm = robotis_framework::sign(torque_new.coeff(2, 0)) * fmin(fabs(torque_new.coeff(2, 0)), 300.0);

  // Lưu trữ dữ liệu lực và mô-men xoắn sau khi xử lý
  r_foot_ft_data_msg_.force.x = r_foot_fx_N;
  r_foot_ft_data_msg_.force.y = r_foot_fy_N;
  r_foot_ft_data_msg_.force.z = r_foot_fz_N;
  r_foot_ft_data_msg_.torque.x = r_foot_Tx_Nm;
  r_foot_ft_data_msg_.torque.y = r_foot_Ty_Nm;
  r_foot_ft_data_msg_.torque.z = r_foot_Tz_Nm;
}

// Callback khi nhận yêu cầu reset vị trí cơ thể
void OnlineWalkingModule::setResetBodyCallback(const std_msgs::Bool::ConstPtr &msg)
{
  if (msg->data == true)
  {
    // Đặt vị trí offset của cơ thể về giá trị mặc định
    des_body_offset_[0] = 0.0;
    des_body_offset_[1] = 0.0;
    des_body_offset_[2] = 0.0;

    // Gọi phương thức để đặt lại vị trí cơ thể
    resetBodyPose();
  }
}

// Callback khi nhận thông số của quá trình đi bộ
void OnlineWalkingModule::walkingParamCallback(const op3_online_walking_module_msgs::WalkingParam &msg)
{
  // Lưu trữ thông số quá trình đi bộ
  walking_param_ = msg;
}

// Callback khi nhận thông tin về vị trí mục tiêu của các khớp cần kiểm soát
void OnlineWalkingModule::goalJointPoseCallback(const op3_online_walking_module_msgs::JointPose &msg)
{
  if (enable_ == false)
    return;

  // Số lượng khớp cần kiểm soát
  size_t joint_size = msg.pose.name.size();

  // Kiểm tra kiểu kiểm soát và khởi tạo nếu kiểu kiểm soát là JOINT_CONTROL hoặc kiểu kiểm soát là NONE
  if (control_type_ == NONE || control_type_ == JOINT_CONTROL)
  {
    // Lưu thời gian di chuyển của quá trình kiểm soát khớp
    mov_time_ = msg.mov_time;

    // Lưu trữ vị trí mục tiêu của các khớp
    for (size_t i = 0; i < joint_size; i++)
    {
      std::string joint_name = msg.pose.name[i];
      goal_joint_pos_[joint_name_to_id_[joint_name] - 1] = msg.pose.position[i];
    }

    // Đặt cờ để báo hiệu rằng kiểm soát khớp đã được khởi tạo
    joint_control_initialize_ = false;

    // Đặt kiểu kiểm soát và kiểu cân bằng tương ứng
    control_type_ = JOINT_CONTROL;
    balance_type_ = OFF;

    // Đặt tỉ lệ cân bằng về 0
    des_balance_gain_ratio_[0] = 0.0;
  }
  else
    ROS_WARN("[WARN] Control type is different!");
}

// Phương thức này khởi tạo quá trình kiểm soát khớp.
void OnlineWalkingModule::initJointControl()
{
  // Kiểm tra nếu kiểm soát khớp đã được khởi tạo
  if (joint_control_initialize_ == true)
    return;

  // Đặt cờ để báo hiệu rằng kiểm soát khớp đã được khởi tạo
  joint_control_initialize_ = true;

  // Thời gian ban đầu và thời gian di chuyển của quá trình kiểm soát khớp
  double ini_time = 0.0;
  double mov_time = mov_time_;

  // Khởi tạo bước và kích thước của quá trình kiểm soát khớp
  mov_step_ = 0;
  mov_size_ = (int)(mov_time / control_cycle_sec_) + 1;

  // Tạo đối tượng kiểm soát khớp với mô hình chuyển động tuyến tính (Minimum Jerk)
  joint_tra_ = new robotis_framework::MinimumJerk(ini_time, mov_time,
                                                  des_joint_pos_, des_joint_vel_, des_joint_accel_,
                                                  goal_joint_pos_, goal_joint_vel_, goal_joint_accel_);

  // Kiểm tra và in thông báo tương ứng
  if (is_moving_ == true)
    ROS_INFO("[UPDATE] Joint Control");
  else
  {
    is_moving_ = true;
    ROS_INFO("[START] Joint Control");
  }
}

// Phương thức này được gọi để tính toán kiểm soát các khớp
void OnlineWalkingModule::calcJointControl()
{
  // Kiểm tra xem quá trình kiểm soát khớp có đang diễn ra không
  if (is_moving_ == true)
  {
    // Tính thời gian hiện tại dựa trên bước di chuyển và chu kỳ kiểm soát
    double cur_time = (double)mov_step_ * control_cycle_sec_;

    // Bảo vệ dữ liệu trong hàng đợi sử dụng mutex
    queue_mutex_.lock();

    // Lấy giá trị mong muốn của khớp, vận tốc và gia tốc từ đối tượng Minimum Jerk
    des_joint_pos_ = joint_tra_->getPosition(cur_time);
    des_joint_vel_ = joint_tra_->getVelocity(cur_time);
    des_joint_accel_ = joint_tra_->getAcceleration(cur_time);

    // Giải phóng mutex sau khi sử dụng dữ liệu
    queue_mutex_.unlock();

    // Kiểm tra xem đã hoàn thành quá trình kiểm soát khớp chưa
    if (mov_step_ == mov_size_ - 1)
    {
      // Reset bước di chuyển nếu đã hoàn thành
      mov_step_ = 0;
      // Đặt cờ hiệu kết thúc quá trình kiểm soát khớp
      is_moving_ = false;
      // Giải phóng bộ nhớ đã cấp phát cho đối tượng Minimum Jerk
      delete joint_tra_;

      // Đặt loại kiểm soát hiện tại thành không có
      control_type_ = NONE;

      ROS_INFO("[END] Joint Control");
    }
    else
      mov_step_++;
  }
}

// Phương thức này được gọi khi có tin nhắn đầu vào về thay đổi vị trí cơ thể (offset).
void OnlineWalkingModule::setBodyOffsetCallback(const geometry_msgs::Pose::ConstPtr &msg)
{
  // Kiểm tra xem chế độ kiểm soát có được kích hoạt hay không
  if (enable_ == false)
    return;

  // Kiểm tra xem chế độ cân bằng có được kích hoạt hay không
  if (balance_type_ == OFF)
  {
    ROS_WARN("[WARN] Chế độ cân bằng đã tắt!");
    return;
  }

  // Kiểm tra xem kiểu kiểm soát hiện tại là kiểm soát không hoặc kiểm soát offset
  if (control_type_ == NONE || control_type_ == OFFSET_CONTROL)
  {
    // Đặt giá trị mong muốn của offset cơ thể từ dữ liệu đầu vào
    goal_body_offset_[0] = msg->position.x;
    goal_body_offset_[1] = msg->position.y;
    goal_body_offset_[2] = msg->position.z;

    // Đặt cờ hiệu khởi tạo lại offset cơ thể
    body_offset_initialize_ = false;
    // Đặt kiểu kiểm soát hiện tại thành kiểm soát offset
    control_type_ = OFFSET_CONTROL;
  }
  else
    ROS_WARN("[WARN] Kiểu kiểm soát khác nhau!");
}

// Phương thức này được gọi khi có tin nhắn đầu vào về thay đổi khoảng cách giữa hai chân.
void OnlineWalkingModule::setFootDistanceCallback(const std_msgs::Float64::ConstPtr &msg)
{
  // Kiểm tra xem chế độ kiểm soát có được kích hoạt hay không
  if (enable_ == false)
    return;

  // Đặt giá trị khoảng cách giữa hai chân từ dữ liệu đầu vào
  foot_distance_ = msg->data;

  // Reset lại tư thế cơ thể
  resetBodyPose();
}

// Phương thức này khởi tạo quá trình kiểm soát offset của cơ thể.
void OnlineWalkingModule::initOffsetControl()
{
  // Kiểm tra nếu kiểm soát offset cơ thể đã được khởi tạo
  if (body_offset_initialize_ == true)
    return;

  // Đặt cờ để báo hiệu rằng kiểm soát offset cơ thể đã được khởi tạo
  body_offset_initialize_ = true;

  // Thời gian ban đầu và thời gian di chuyển của quá trình kiểm soát offset cơ thể
  double ini_time = 0.0;
  double mov_time = 1.0;

  // Khởi tạo bước và kích thước của quá trình kiểm soát offset cơ thể
  body_offset_step_ = 0;
  body_offset_size_ = (int)(mov_time / control_cycle_sec_) + 1;

  // Tạo đối tượng kiểm soát offset cơ thể với mô hình chuyển động tuyến tính (Minimum Jerk)
  std::vector<double_t> offset_zero;
  offset_zero.resize(3, 0.0);

  body_offset_tra_ =
      new robotis_framework::MinimumJerk(ini_time, mov_time,
                                         des_body_offset_, offset_zero, offset_zero,
                                         goal_body_offset_, offset_zero, offset_zero);

  // Kiểm tra và in thông báo tương ứng
  if (is_moving_ == true)
    ROS_INFO("[UPDATE] Kiểm soát Offset Cơ thể");
  else
  {
    is_moving_ = true;
    ROS_INFO("[START] Kiểm soát Offset Cơ thể");
  }
}

// Phương thức này tính toán quá trình kiểm soát offset của cơ thể.
void OnlineWalkingModule::calcOffsetControl()
{
  // Kiểm tra xem quá trình kiểm soát offset của cơ thể có đang diễn ra không
  if (is_moving_ == true)
  {
    // Tính thời gian hiện tại dựa trên bước di chuyển và chu kỳ kiểm soát
    double cur_time = (double)body_offset_step_ * control_cycle_sec_;

    // Bảo vệ dữ liệu trong hàng đợi sử dụng mutex
    queue_mutex_.lock();

    // Lấy giá trị mong muốn của offset cơ thể từ đối tượng Minimum Jerk
    des_body_offset_ = body_offset_tra_->getPosition(cur_time);

    // Giải phóng mutex sau khi sử dụng dữ liệu
    queue_mutex_.unlock();

    // Kiểm tra xem đã hoàn thành quá trình kiểm soát offset cơ thể chưa
    if (body_offset_step_ == body_offset_size_ - 1)
    {
      // Reset bước di chuyển nếu đã hoàn thành
      body_offset_step_ = 0;
      // Đặt cờ hiệu kết thúc quá trình kiểm soát offset cơ thể
      is_moving_ = false;
      // Giải phóng bộ nhớ đã cấp phát cho đối tượng Minimum Jerk
      delete body_offset_tra_;

      // Đặt loại kiểm soát hiện tại thành không có
      control_type_ = NONE;

      ROS_INFO("[END] Kiểm soát Offset Cơ thể");
    }
    else
      body_offset_step_++;
  }
}

// Phương thức này xử lý callback khi nhận được thông điệp mục tiêu về vị trí học kiến thức toàn bộ cơ thể.
void OnlineWalkingModule::goalKinematicsPoseCallback(const op3_online_walking_module_msgs::KinematicsPose &msg)
{
  // Kiểm tra xem module có được kích hoạt không
  if (enable_ == false)
    return;

  // Kiểm tra xem chế độ cân bằng có đang tắt không
  if (balance_type_ == OFF)
  {
    ROS_WARN("[WARN] Balance is off!");
    return;
  }

  // Kiểm tra loại kiểm soát hiện tại
  if (control_type_ == NONE || control_type_ == WHOLEBODY_CONTROL)
  {
    // Kiểm tra xem quá trình kiểm soát cơ thể đang diễn ra không
    if (is_moving_ == true)
    {
      // Kiểm tra xem nhóm kiểm soát cơ thể có giống không
      if (wholegbody_control_group_ != msg.name)
      {
        ROS_WARN("[WARN] Control group is different!");
        return;
      }
    }

    // Đặt thời gian di chuyển và nhóm kiểm soát cơ thể
    mov_time_ = msg.mov_time;
    wholegbody_control_group_ = msg.name;
    wholebody_goal_msg_ = msg.pose;

    // Đặt cờ hiệu khởi tạo kiểm soát cơ thể thành false
    wholebody_initialize_ = false;
    // Đặt loại kiểm soát hiện tại thành WHOLEBODY_CONTROL
    control_type_ = WHOLEBODY_CONTROL;
  }
  else
    ROS_WARN("[WARN] Control type is different!");
}

// Phương thức này khởi tạo quá trình kiểm soát toàn bộ cơ thể.
void OnlineWalkingModule::initWholebodyControl()
{
  // Kiểm tra xem kiểm soát toàn bộ cơ thể đã được khởi tạo chưa
  if (wholebody_initialize_ == true)
    return;

  // Đặt cờ hiệu khởi tạo kiểm soát toàn bộ cơ thể thành true
  wholebody_initialize_ = true;

  // Thời gian ban đầu và thời gian di chuyển của quá trình kiểm soát toàn bộ cơ thể
  double ini_time = 0.0;
  double mov_time = mov_time_;

  // Khởi tạo bước và kích thước của quá trình kiểm soát toàn bộ cơ thể
  mov_step_ = 0;
  mov_size_ = (int)(mov_time / control_cycle_sec_) + 1;

  // Tạo đối tượng kiểm soát toàn bộ cơ thể
  wholebody_control_ =
      new WholebodyControl(wholegbody_control_group_,
                           ini_time, mov_time,
                           wholebody_goal_msg_);

  // Kiểm tra và in thông báo tương ứng nếu quá trình kiểm soát cơ thể đang diễn ra
  if (is_moving_ == true)
  {
    // TODO: Xử lý khi quá trình đang diễn ra
  }
  else
  {
    // In thông báo bắt đầu quá trình kiểm soát toàn bộ cơ thể
    ROS_INFO("[START] Wholebody Control");

    // Khởi tạo giá trị mong muốn cho cơ thể, chân phải, chân trái
    wholebody_control_->initialize(des_body_pos_, des_body_Q_,
                                   des_r_leg_pos_, des_r_leg_Q_,
                                   des_l_leg_pos_, des_l_leg_Q_);

    // Đặt cờ hiệu quá trình di chuyển thành true
    is_moving_ = true;
  }
}

// Phương thức này tính toán quá trình kiểm soát toàn bộ cơ thể.
void OnlineWalkingModule::calcWholebodyControl()
{
  // Kiểm tra xem quá trình kiểm soát toàn bộ cơ thể đang diễn ra không
  if (is_moving_ == true)
  {
    // Tính thời gian hiện tại dựa trên bước di chuyển và chu kỳ kiểm soát
    double cur_time = (double)mov_step_ * control_cycle_sec_;

    // Thiết lập thời điểm hiện tại cho kiểm soát toàn bộ cơ thể
    wholebody_control_->set(cur_time);

    // Lấy giá trị mong muốn của vị trí và hướng của chân trái, chân phải, và cơ thể
    wholebody_control_->getTaskPosition(des_l_leg_pos_,
                                        des_r_leg_pos_,
                                        des_body_pos_);
    wholebody_control_->getTaskOrientation(des_l_leg_Q_,
                                           des_r_leg_Q_,
                                           des_body_Q_);

    // Kiểm tra xem quá trình đã hoàn thành hay chưa
    if (mov_step_ == mov_size_ - 1)
    {
      // Reset bước di chuyển nếu đã hoàn thành
      mov_step_ = 0;
      // Đặt cờ hiệu kết thúc quá trình kiểm soát toàn bộ cơ thể
      is_moving_ = false;
      // Kết thúc quá trình kiểm soát toàn bộ cơ thể
      wholebody_control_->finalize();

      // Đặt loại kiểm soát hiện tại thành không có
      control_type_ = NONE;

      // In thông báo kết thúc quá trình kiểm soát toàn bộ cơ thể
      ROS_INFO("[END] Wholebody Control");
    }
    else
      mov_step_++;
  }
}

// Phương thức xử lý callback cho dữ liệu bước chân 2D
void OnlineWalkingModule::footStep2DCallback(const op3_online_walking_module_msgs::Step2DArray &msg)
{
  // Kiểm tra xem module có được bật hay không
  if (enable_ == false)
    return;

  // Kiểm tra xem kiểm soát cân bằng có đang tắt hay không
  if (balance_type_ == OFF)
  {
    ROS_WARN("[WARN] Balance is off!");
    return;
  }

  // Chuyển đổi giá trị quaternions của cơ thể thành ma trận quay
  Eigen::Quaterniond body_Q(des_body_Q_[3], des_body_Q_[0], des_body_Q_[1], des_body_Q_[2]);
  Eigen::MatrixXd body_R = robotis_framework::convertQuaternionToRotation(body_Q);
  Eigen::MatrixXd body_rpy = robotis_framework::convertQuaternionToRPY(body_Q);
  Eigen::MatrixXd body_T = Eigen::MatrixXd::Identity(4, 4);
  body_T.block(0, 0, 3, 3) = body_R;
  body_T.coeffRef(0, 3) = des_body_pos_[0];
  body_T.coeffRef(1, 3) = des_body_pos_[1];

  // Tạo thông điệp cho bước chân mới
  op3_online_walking_module_msgs::Step2DArray foot_step_msg;

  // Lấy số lượng bước chân đã có
  int old_size = msg.footsteps_2d.size();
  // Tính số lượng mới sau khi thêm 3 bước (1 chuyển động và 2 tĩnh)
  int new_size = old_size + 3;

  // Tạo thông điệp cho 2 bước chân đầu tiên
  op3_online_walking_module_msgs::Step2D first_msg;
  op3_online_walking_module_msgs::Step2D second_msg;

  // Xác định chân đang di chuyển và chân còn lại
  first_msg.moving_foot = msg.footsteps_2d[0].moving_foot - 1;
  second_msg.moving_foot = first_msg.moving_foot + 1;

  // Thiết lập vị trí và hướng của chân đầu tiên
  if (first_msg.moving_foot == LEFT_LEG)
  {
    first_msg.step2d.x = des_l_leg_pos_[0];
    first_msg.step2d.y = des_l_leg_pos_[1];
    first_msg.step2d.theta = body_rpy.coeff(2, 0);
  }
  else if (first_msg.moving_foot == RIGHT_LEG)
  {
    first_msg.step2d.x = des_r_leg_pos_[0];
    first_msg.step2d.y = des_r_leg_pos_[1];
    first_msg.step2d.theta = body_rpy.coeff(2, 0);
  }

  // Thiết lập vị trí và hướng của chân thứ hai
  if (second_msg.moving_foot == LEFT_LEG)
  {
    second_msg.step2d.x = des_l_leg_pos_[0];
    second_msg.step2d.y = des_l_leg_pos_[1];
    second_msg.step2d.theta = body_rpy.coeff(2, 0);
  }
  else if (second_msg.moving_foot == RIGHT_LEG)
  {
    second_msg.step2d.x = des_r_leg_pos_[0];
    second_msg.step2d.y = des_r_leg_pos_[1];
    second_msg.step2d.theta = body_rpy.coeff(2, 0);
  }

  // Thêm thông điệp của 2 bước chân đầu tiên vào danh sách
  foot_step_msg.footsteps_2d.push_back(first_msg);
  foot_step_msg.footsteps_2d.push_back(second_msg);

  // Biến lưu giữ giá trị cuối cùng của góc quay của bước chân
  double step_final_theta;

  // Xử lý thông điệp bước chân từ callback
  if (control_type_ == NONE || control_type_ == WALKING_CONTROL)
  {
    for (int i = 0; i < old_size; i++)
    {
      // Lấy thông điệp của bước chân từ danh sách đã có
      op3_online_walking_module_msgs::Step2D step_msg = msg.footsteps_2d[i];
      step_msg.moving_foot -= 1;

      // Chuyển đổi vị trí và hướng của bước chân từ hệ toạ độ cơ thể về hệ toạ độ thế giới
      Eigen::MatrixXd step_R = robotis_framework::convertRPYToRotation(0.0, 0.0, step_msg.step2d.theta);
      Eigen::MatrixXd step_T = Eigen::MatrixXd::Identity(4, 4);
      step_T.block(0, 0, 3, 3) = step_R;
      step_T.coeffRef(0, 3) = step_msg.step2d.x;
      step_T.coeffRef(1, 3) = step_msg.step2d.y;

      Eigen::MatrixXd step_T_new = body_T * step_T;
      Eigen::MatrixXd step_R_new = step_T_new.block(0, 0, 3, 3);

      double step_new_x = step_T_new.coeff(0, 3);
      double step_new_y = step_T_new.coeff(1, 3);
      Eigen::MatrixXd step_new_rpy = robotis_framework::convertRotationToRPY(step_R_new);
      double step_new_theta = step_new_rpy.coeff(2, 0);

      step_msg.step2d.x = step_new_x;
      step_msg.step2d.y = step_new_y;
      step_msg.step2d.theta = step_new_theta;

      // Lưu giá trị cuối cùng của góc quay
      if (i == old_size - 1)
        step_final_theta = step_new_theta;

      // Thêm thông điệp đã xử lý vào danh sách mới
      foot_step_msg.footsteps_2d.push_back(step_msg);
    }

    // Tạo thông điệp cho bước chân cuối cùng
    op3_online_walking_module_msgs::Step2D step_msg = msg.footsteps_2d[old_size - 1];

    // Xác định chân đang di chuyển
    if (step_msg.moving_foot - 1 == LEFT_LEG)
      first_msg.moving_foot = RIGHT_LEG;
    else
      first_msg.moving_foot = LEFT_LEG;

    // Thiết lập vị trí và hướng của chân cuối cùng
    first_msg.step2d.x = 0.0;
    first_msg.step2d.y = 0.0;
    first_msg.step2d.theta = step_final_theta;

    // Thêm thông điệp của bước chân cuối cùng vào danh sách
    foot_step_msg.footsteps_2d.push_back(first_msg);

    // Cập nhật thông điệp bước chân và thời gian di chuyển
    foot_step_2d_ = foot_step_msg;
    foot_step_2d_.step_time = msg.step_time;

    // Cập nhật kích thước của quá trình di chuyển
    walking_size_ = new_size;
    mov_time_ = msg.step_time;
    is_foot_step_2d_ = true;
    control_type_ = WALKING_CONTROL;

    // Kiểm tra và khởi tạo quá trình kiểm soát di chuyển
    if (is_moving_ == false)
      initWalkingControl();
    else
      ROS_WARN("[WARN] Previous task is alive!");
  }
  else
    ROS_WARN("[WARN] Control type is different!");
}

// Phương thức xử lý callback cho dữ liệu lệnh bước chân
void OnlineWalkingModule::footStepCommandCallback(const op3_online_walking_module_msgs::FootStepCommand &msg)
{
  // Kiểm tra xem module có được bật hay không
  if (enable_ == false)
    return;

  // Kiểm tra xem kiểm soát cân bằng có đang tắt hay không
  if (balance_type_ == OFF)
  {
    ROS_WARN("[WARN] Balance is off!");
    return;
  }

  // Tắt kiểm soát bước chân 2D (nếu có)
  is_foot_step_2d_ = false;

  // Xử lý dữ liệu lệnh bước chân
  if (control_type_ == NONE || control_type_ == WALKING_CONTROL)
  {
    // Cập nhật kích thước của quá trình di chuyển và thời gian di chuyển
    walking_size_ = msg.step_num + 3;
    mov_time_ = msg.step_time;

    // Lưu trữ thông điệp lệnh bước chân
    foot_step_command_ = msg;
    foot_step_command_.step_num = walking_size_;

    // Đặt loại kiểm soát thành WALKING_CONTROL
    control_type_ = WALKING_CONTROL;

    // Kiểm tra và khởi tạo quá trình kiểm soát di chuyển
    if (is_moving_ == false)
      initWalkingControl();
    else
      ROS_WARN("[WARN] Previous task is alive!");
  }
  else
    ROS_WARN("[WARN] Control type is different!");
}

// Phương thức khởi tạo quá trình kiểm soát di chuyển
void OnlineWalkingModule::initWalkingControl()
{
  // Lấy thời gian di chuyển từ biến cấu hình
  double mov_time = mov_time_;

  // Tính toán số bước di chuyển và kích thước của mỗi bước
  mov_step_ = 0;
  mov_size_ = (int)(mov_time / control_cycle_sec_) + 1;

  // Thiết lập các biến liên quan đến quá trình di chuyển
  walking_step_ = 0;

  // Tạo đối tượng kiểm soát di chuyển
  walking_control_ = new WalkingControl(control_cycle_sec_,
                                        walking_param_.dsp_ratio, walking_param_.lipm_height, walking_param_.foot_height_max,
                                        walking_param_.zmp_offset_x, walking_param_.zmp_offset_y,
                                        x_lipm_, y_lipm_,
                                        foot_distance_);

  // Lấy chiều cao cơ bản của mô hình LIPM
  double lipm_height = walking_control_->getLipmHeight();

  // Đặt thông số cho yêu cầu mô phỏng xem trước
  preview_request_.lipm_height = lipm_height;
  preview_request_.control_cycle = control_cycle_sec_;

  // Định nghĩa ma trận xem trước
  bool get_preview_matrix = false;
  get_preview_matrix = definePreviewMatrix();

  // Kiểm tra xem có thành công lấy ma trận xem trước không
  if (get_preview_matrix == true)
  {
    // Xử lý dữ liệu nếu module đang trong trạng thái di chuyển
    if (is_moving_ == true)
    {
      // TODO
    }
    else
    {
      // Khởi tạo kiểm soát di chuyển dựa trên dữ liệu bước chân 2D hoặc lệnh bước chân
      if (is_foot_step_2d_ == true)
      {
        walking_control_->initialize(foot_step_2d_,
                                     des_body_pos_, des_body_Q_,
                                     des_r_leg_pos_, des_r_leg_Q_,
                                     des_l_leg_pos_, des_l_leg_Q_);
      }
      else
      {
        walking_control_->initialize(foot_step_command_,
                                     des_body_pos_, des_body_Q_,
                                     des_r_leg_pos_, des_r_leg_Q_,
                                     des_l_leg_pos_, des_l_leg_Q_);
      }

      // Tính toán thông số mô phỏng xem trước
      walking_control_->calcPreviewParam(preview_response_K_, preview_response_K_row_, preview_response_K_col_,
                                         preview_response_P_, preview_response_P_row_, preview_response_P_row_);

      // Đặt cờ hiệu di chuyển thành đúng
      is_moving_ = true;

      // Khởi tạo kiểm soát feedforward
      initFeedforwardControl();

      ROS_INFO("[START] Walking Control (%d/%d)", walking_step_ + 1, walking_size_);
    }

    // Đặt cờ hiệu khởi tạo di chuyển thành đúng
    walking_initialize_ = true;
  }
  else
    ROS_WARN("[FAIL] Cannot get preview matrix");
}

// Phương thức tính toán kiểm soát di chuyển
void OnlineWalkingModule::calcWalkingControl()
{
  // Kiểm tra xem module có đang trong trạng thái di chuyển không
  if (is_moving_ == true)
  {
    // Tính thời gian hiện tại dựa trên bước di chuyển và chu kỳ kiểm soát
    double cur_time = (double)mov_step_ * control_cycle_sec_;

    // Cập nhật thông tin về trạng thái và bước di chuyển trong kiểm soát di chuyển
    walking_control_->set(cur_time, walking_step_, is_foot_step_2d_);

    // Lấy vị trí mong muốn của bước chân và cơ thể
    walking_control_->getWalkingPosition(des_l_leg_pos_,
                                         des_r_leg_pos_,
                                         des_body_pos_);

    // Lấy hướng mong muốn của bước chân và cơ thể
    walking_control_->getWalkingOrientation(des_l_leg_Q_,
                                            des_r_leg_Q_,
                                            des_body_Q_);

    // Lấy thông số LIPM hiện tại
    walking_control_->getLIPM(x_lipm_, y_lipm_);

    // Lấy trạng thái di chuyển hiện tại
    walking_control_->getWalkingState(walking_leg_, walking_phase_);

    // Kiểm tra xem đã hoàn thành bước di chuyển hiện tại chưa
    if (mov_step_ == mov_size_ - 1)
    {
      // In thông báo khi hoàn thành bước di chuyển
      ROS_INFO("[END] Walking Control (%d/%d)", walking_step_ + 1, walking_size_);

      // Reset bước di chuyển
      mov_step_ = 0;
      // Chuyển sang bước di chuyển tiếp theo trong kiểm soát di chuyển
      walking_control_->next();

      // Kiểm tra xem đã hoàn thành tất cả các bước di chuyển chưa
      if (walking_step_ == walking_size_ - 1)
      {
        // Đặt cờ hiệu di chuyển về giảm
        is_moving_ = false;
        is_foot_step_2d_ = false;
        // Kết thúc quá trình kiểm soát di chuyển
        walking_control_->finalize();

        // Đặt loại kiểm soát hiện tại về không có kiểm soát
        control_type_ = NONE;
        // Đặt trạng thái di chuyển về DSP (Double Support Phase)
        walking_phase_ = DSP;
      }
      else
      {
        // Chuyển sang bước di chuyển tiếp theo
        walking_step_++;
        // In thông báo khi bắt đầu bước di chuyển mới
        ROS_INFO("[START] Walking Control (%d/%d)", walking_step_ + 1, walking_size_);
      }
    }
    else
      mov_step_++;
  }
}

// Phương thức khởi tạo kiểm soát Feedforward
void OnlineWalkingModule::initFeedforwardControl()
{
  // Feedforward trajectory
  std::vector<double_t> zero_vector;
  zero_vector.resize(1, 0.0);

  std::vector<double_t> via_pos;
  via_pos.resize(3, 0.0);
  via_pos[0] = 1.0 * DEGREE2RADIAN;

  double init_time = 0.0;
  double fin_time = mov_time_;
  double via_time = 0.5 * (init_time + fin_time);
  double dsp_ratio = walking_param_.dsp_ratio;

  feed_forward_tra_ =
      new robotis_framework::MinimumJerkViaPoint(init_time, fin_time, via_time, dsp_ratio,
                                                 zero_vector, zero_vector, zero_vector,
                                                 zero_vector, zero_vector, zero_vector,
                                                 via_pos, zero_vector, zero_vector);
}

// Phương thức tính toán vị trí của robot
void OnlineWalkingModule::calcRobotPose()
{
  // Chuyển vị trí mong muốn của cơ thể thành ma trận Eigen
  Eigen::MatrixXd des_body_pos = Eigen::MatrixXd::Zero(3, 1);
  des_body_pos.coeffRef(0, 0) = des_body_pos_[0];
  des_body_pos.coeffRef(1, 0) = des_body_pos_[1];
  des_body_pos.coeffRef(2, 0) = des_body_pos_[2];

  // Chuyển hướng mong muốn của cơ thể thành dạng Quaternion và ma trận quay
  Eigen::Quaterniond des_body_Q(des_body_Q_[3], des_body_Q_[0], des_body_Q_[1], des_body_Q_[2]);
  Eigen::MatrixXd des_body_rot = robotis_framework::convertQuaternionToRotation(des_body_Q);

  // Tính toán Forward Kinematics
  op3_kdl_->initialize(des_body_pos, des_body_rot);

  // Lấy vị trí của các khớp chân phải và chân trái
  Eigen::VectorXd r_leg_joint_pos, l_leg_joint_pos;
  r_leg_joint_pos.resize(6);
  l_leg_joint_pos.resize(6);

  // Gán giá trị vị trí của các khớp chân phải và chân trái từ danh sách vị trí mong muốn
  r_leg_joint_pos(0) = des_joint_pos_[joint_name_to_id_["r_hip_yaw"] - 1];
  r_leg_joint_pos(1) = des_joint_pos_[joint_name_to_id_["r_hip_roll"] - 1];
  r_leg_joint_pos(2) = des_joint_pos_[joint_name_to_id_["r_hip_pitch"] - 1];
  r_leg_joint_pos(3) = des_joint_pos_[joint_name_to_id_["r_knee"] - 1];
  r_leg_joint_pos(4) = des_joint_pos_[joint_name_to_id_["r_ank_pitch"] - 1];
  r_leg_joint_pos(5) = des_joint_pos_[joint_name_to_id_["r_ank_roll"] - 1];

  l_leg_joint_pos(0) = des_joint_pos_[joint_name_to_id_["l_hip_yaw"] - 1];
  l_leg_joint_pos(1) = des_joint_pos_[joint_name_to_id_["l_hip_roll"] - 1];
  l_leg_joint_pos(2) = des_joint_pos_[joint_name_to_id_["l_hip_pitch"] - 1];
  l_leg_joint_pos(3) = des_joint_pos_[joint_name_to_id_["l_knee"] - 1];
  l_leg_joint_pos(4) = des_joint_pos_[joint_name_to_id_["l_ank_pitch"] - 1];
  l_leg_joint_pos(5) = des_joint_pos_[joint_name_to_id_["l_ank_roll"] - 1];

  // Thiết lập vị trí của các khớp chân phải và chân trái
  op3_kdl_->setJointPosition(r_leg_joint_pos, l_leg_joint_pos);

  std::vector<double_t> r_leg_pos, r_leg_Q;
  r_leg_pos.resize(3, 0.0);
  r_leg_Q.resize(4, 0.0);

  std::vector<double_t> l_leg_pos, l_leg_Q;
  l_leg_pos.resize(3, 0.0);
  l_leg_Q.resize(4, 0.0);

  // Giải quyết vấn đề Forward Kinematics để lấy vị trí và hướng của chân phải và chân trái
  op3_kdl_->solveForwardKinematics(r_leg_pos, r_leg_Q, l_leg_pos, l_leg_Q);

  // Chuyển định dạng Quaternion và Rotation sang Eigen
  Eigen::Quaterniond curr_r_leg_Q(r_leg_Q[3], r_leg_Q[0], r_leg_Q[1], r_leg_Q[2]);
  Eigen::MatrixXd curr_r_leg_rot = robotis_framework::convertQuaternionToRotation(curr_r_leg_Q);

  Eigen::MatrixXd g_to_r_leg = Eigen::MatrixXd::Identity(4, 4);
  g_to_r_leg.block(0, 0, 3, 3) = curr_r_leg_rot;
  g_to_r_leg.coeffRef(0, 3) = r_leg_pos[0];
  g_to_r_leg.coeffRef(1, 3) = r_leg_pos[1];
  g_to_r_leg.coeffRef(2, 3) = r_leg_pos[2];

  Eigen::Quaterniond curr_l_leg_Q(l_leg_Q[3], l_leg_Q[0], l_leg_Q[1], l_leg_Q[2]);
  Eigen::MatrixXd curr_l_leg_rot = robotis_framework::convertQuaternionToRotation(curr_l_leg_Q);

  Eigen::MatrixXd g_to_l_leg = Eigen::MatrixXd::Identity(4, 4);
  g_to_l_leg.block(0, 0, 3, 3) = curr_l_leg_rot;
  g_to_l_leg.coeffRef(0, 3) = l_leg_pos[0];
  g_to_l_leg.coeffRef(1, 3) = l_leg_pos[1];
  g_to_l_leg.coeffRef(2, 3) = l_leg_pos[2];

  // Kết thúc quá trình Forward Kinematics
  op3_kdl_->finalize();
}

// Phương thức thiết lập lực và mô-men xoắn mục tiêu
void OnlineWalkingModule::setTargetForceTorque()
{
  // Nếu đang ở giai đoạn DSP (Double Support Phase)
  if (walking_phase_ == DSP)
  {
    // Thiết lập lực và mô-men xoắn cần thiết cho chân phải và chân trái
    balance_r_foot_force_x_ = -0.5 * total_mass_ * x_lipm_[2];
    balance_r_foot_force_y_ = -0.5 * total_mass_ * y_lipm_[2];
    balance_r_foot_force_z_ = -0.5 * total_mass_ * 9.81;

    balance_l_foot_force_x_ = -0.5 * total_mass_ * x_lipm_[2];
    balance_l_foot_force_y_ = -0.5 * total_mass_ * y_lipm_[2];
    balance_l_foot_force_z_ = -0.5 * total_mass_ * 9.81;
  }
  // Nếu đang ở giai đoạn SSP (Single Support Phase)
  else if (walking_phase_ == SSP)
  {
    // Nếu chân đang nâng lên là chân trái
    if (walking_leg_ == LEFT_LEG)
    {
      // Thiết lập lực và mô-men xoắn cần thiết cho chân phải và chân trái
      balance_r_foot_force_x_ = -1.0 * total_mass_ * x_lipm_[2];
      balance_r_foot_force_y_ = -1.0 * total_mass_ * y_lipm_[2];
      balance_r_foot_force_z_ = -1.0 * total_mass_ * 9.81;

      balance_l_foot_force_x_ = 0.0;
      balance_l_foot_force_y_ = 0.0;
      balance_l_foot_force_z_ = 0.0;
    }
    // Nếu chân đang nâng lên là chân phải
    else if (walking_leg_ == RIGHT_LEG)
    {
      // Thiết lập lực và mô-men xoắn cần thiết cho chân phải và chân trái
      balance_r_foot_force_x_ = 0.0;
      balance_r_foot_force_y_ = 0.0;
      balance_r_foot_force_z_ = 0.0;

      balance_l_foot_force_x_ = -1.0 * total_mass_ * x_lipm_[2];
      balance_l_foot_force_y_ = -1.0 * total_mass_ * y_lipm_[2];
      balance_l_foot_force_z_ = -1.0 * total_mass_ * 9.81;
    }
  }
}

void OnlineWalkingModule::setBalanceControlGain()
{
  //// Thiết lập các thông số kiểm soát (gains)

  // Gains cho giảm chấn từ cảm biến gyro
  balance_control_.foot_roll_gyro_ctrl_.p_gain_ = foot_roll_gyro_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.foot_roll_gyro_ctrl_.d_gain_ = foot_roll_gyro_d_gain_ * des_balance_gain_ratio_[0];
  balance_control_.foot_pitch_gyro_ctrl_.p_gain_ = foot_pitch_gyro_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.foot_pitch_gyro_ctrl_.d_gain_ = foot_pitch_gyro_d_gain_ * des_balance_gain_ratio_[0];

  // Gains cho giảm chấn từ cảm biến orientation
  balance_control_.foot_roll_angle_ctrl_.p_gain_ = foot_roll_angle_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.foot_roll_angle_ctrl_.d_gain_ = foot_roll_angle_d_gain_ * des_balance_gain_ratio_[0];
  balance_control_.foot_pitch_angle_ctrl_.p_gain_ = foot_pitch_angle_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.foot_pitch_angle_ctrl_.d_gain_ = foot_pitch_angle_d_gain_ * des_balance_gain_ratio_[0];

  // Gains cho giảm chấn từ lực và mô-men lực
  balance_control_.right_foot_force_x_ctrl_.p_gain_ = foot_x_force_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.right_foot_force_y_ctrl_.p_gain_ = foot_y_force_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.right_foot_force_z_ctrl_.p_gain_ = foot_z_force_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.right_foot_torque_roll_ctrl_.p_gain_ = foot_roll_torque_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.right_foot_torque_pitch_ctrl_.p_gain_ = foot_roll_torque_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.right_foot_force_x_ctrl_.d_gain_ = foot_x_force_d_gain_ * des_balance_gain_ratio_[0];
  balance_control_.right_foot_force_y_ctrl_.d_gain_ = foot_y_force_d_gain_ * des_balance_gain_ratio_[0];
  balance_control_.right_foot_force_z_ctrl_.d_gain_ = foot_z_force_d_gain_ * des_balance_gain_ratio_[0];
  balance_control_.right_foot_torque_roll_ctrl_.d_gain_ = foot_roll_torque_d_gain_ * des_balance_gain_ratio_[0];
  balance_control_.right_foot_torque_pitch_ctrl_.d_gain_ = foot_roll_torque_d_gain_ * des_balance_gain_ratio_[0];

  // Tương tự cho chân trái
  balance_control_.left_foot_force_x_ctrl_.p_gain_ = foot_x_force_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.left_foot_force_y_ctrl_.p_gain_ = foot_y_force_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.left_foot_force_z_ctrl_.p_gain_ = foot_z_force_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.left_foot_torque_roll_ctrl_.p_gain_ = foot_roll_torque_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.left_foot_torque_pitch_ctrl_.p_gain_ = foot_roll_torque_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.left_foot_force_x_ctrl_.d_gain_ = foot_x_force_d_gain_ * des_balance_gain_ratio_[0];
  balance_control_.left_foot_force_y_ctrl_.d_gain_ = foot_y_force_d_gain_ * des_balance_gain_ratio_[0];
  balance_control_.left_foot_force_z_ctrl_.d_gain_ = foot_z_force_d_gain_ * des_balance_gain_ratio_[0];
  balance_control_.left_foot_torque_roll_ctrl_.d_gain_ = foot_roll_torque_d_gain_ * des_balance_gain_ratio_[0];
  balance_control_.left_foot_torque_pitch_ctrl_.d_gain_ = foot_roll_torque_d_gain_ * des_balance_gain_ratio_[0];

  //// Thiết lập tần số cắt cho bộ lọc (LPF)

  // Tần số cắt cho giảm chấn từ gyro và orientation
  balance_control_.roll_gyro_lpf_.setCutOffFrequency(roll_gyro_cut_off_frequency_);
  balance_control_.pitch_gyro_lpf_.setCutOffFrequency(pitch_gyro_cut_off_frequency_);
  balance_control_.roll_angle_lpf_.setCutOffFrequency(roll_angle_cut_off_frequency_);
  balance_control_.pitch_angle_lpf_.setCutOffFrequency(pitch_angle_cut_off_frequency_);

  // Tần số cắt cho giảm chấn từ lực và mô-men lực
  balance_control_.right_foot_force_x_lpf_.setCutOffFrequency(foot_x_force_cut_off_frequency_);
  balance_control_.right_foot_force_y_lpf_.setCutOffFrequency(foot_y_force_cut_off_frequency_);
  balance_control_.right_foot_force_z_lpf_.setCutOffFrequency(foot_z_force_cut_off_frequency_);
  balance_control_.right_foot_torque_roll_lpf_.setCutOffFrequency(foot_roll_torque_cut_off_frequency_);
  balance_control_.right_foot_torque_pitch_lpf_.setCutOffFrequency(foot_pitch_torque_cut_off_frequency_);

  // Tương tự cho chân trái
  balance_control_.left_foot_force_x_lpf_.setCutOffFrequency(foot_x_force_cut_off_frequency_);
  balance_control_.left_foot_force_y_lpf_.setCutOffFrequency(foot_y_force_cut_off_frequency_);
  balance_control_.left_foot_force_z_lpf_.setCutOffFrequency(foot_z_force_cut_off_frequency_);
  balance_control_.left_foot_torque_roll_lpf_.setCutOffFrequency(foot_roll_torque_cut_off_frequency_);
  balance_control_.left_foot_torque_pitch_lpf_.setCutOffFrequency(foot_pitch_torque_cut_off_frequency_);
}

bool OnlineWalkingModule::setBalanceControl()
{
  // Bật kiểm soát cân bằng
  balance_control_.setGyroBalanceEnable(true);        // Bật cân bằng từ dữ liệu giảm chấn góc quay
  balance_control_.setOrientationBalanceEnable(true); // Bật cân bằng dựa trên hướng của robot
  balance_control_.setForceTorqueBalanceEnable(true); // Bật cân bằng từ dữ liệu lực và moment

  // Điều chỉnh vị trí trọng tâm cơ bản của robot
  balance_control_.setCOBManualAdjustment(des_body_offset_[0], des_body_offset_[1], des_body_offset_[2]);

  // Thiết lập hệ số kiểm soát cân bằng và mục tiêu lực và moment
  setBalanceControlGain();
  setTargetForceTorque();

  // Biến kiểm tra thành công của bước ngược hạn chế
  bool ik_success = true;

  // Body Pose (Vị trí và góc quay của cơ bản)
  Eigen::MatrixXd des_body_pos = Eigen::MatrixXd::Zero(3, 1);
  des_body_pos.coeffRef(0, 0) = des_body_pos_[0];
  des_body_pos.coeffRef(1, 0) = des_body_pos_[1];
  des_body_pos.coeffRef(2, 0) = des_body_pos_[2];

  Eigen::Quaterniond des_body_Q(des_body_Q_[3], des_body_Q_[0], des_body_Q_[1], des_body_Q_[2]);
  Eigen::MatrixXd des_body_rot = robotis_framework::convertQuaternionToRotation(des_body_Q);
  Eigen::MatrixXd des_body_rpy = robotis_framework::convertQuaternionToRPY(des_body_Q);

  // Right Leg Pose (Vị trí và góc quay của chân phải)
  Eigen::MatrixXd des_r_foot_pos = Eigen::MatrixXd::Zero(3, 1);
  des_r_foot_pos.coeffRef(0, 0) = des_r_leg_pos_[0];
  des_r_foot_pos.coeffRef(1, 0) = des_r_leg_pos_[1];
  des_r_foot_pos.coeffRef(2, 0) = des_r_leg_pos_[2];

  // Tạo đối tượng Quaternion từ mảng des_r_leg_Q_
  Eigen::Quaterniond des_r_foot_Q(des_r_leg_Q_[3], des_r_leg_Q_[0], des_r_leg_Q_[1], des_r_leg_Q_[2]);

  // Chuyển đổi Quaternion thành ma trận xoay
  Eigen::MatrixXd des_r_foot_rot = robotis_framework::convertQuaternionToRotation(des_r_foot_Q);

  // Left Leg Pose (Vị trí và góc quay của chân trái)
  Eigen::MatrixXd des_l_foot_pos = Eigen::MatrixXd::Zero(3, 1);
  des_l_foot_pos.coeffRef(0, 0) = des_l_leg_pos_[0];
  des_l_foot_pos.coeffRef(1, 0) = des_l_leg_pos_[1];
  des_l_foot_pos.coeffRef(2, 0) = des_l_leg_pos_[2];

  // Tạo đối tượng Quaternion từ mảng des_l_leg_Q_
  Eigen::Quaterniond des_l_foot_Q(des_l_leg_Q_[3], des_l_leg_Q_[0], des_l_leg_Q_[1], des_l_leg_Q_[2]);

  // Chuyển đổi Quaternion thành ma trận xoay
  Eigen::MatrixXd des_l_foot_rot = robotis_framework::convertQuaternionToRotation(des_l_foot_Q);

  // Set Desired Value for Balance Control
  // Set Desired Value for Balance Control
  Eigen::MatrixXd body_pose = Eigen::MatrixXd::Identity(4, 4);
  body_pose.block<3, 3>(0, 0) = des_body_rot; // Góc quay của cơ bản
  body_pose.block<3, 1>(0, 3) = des_body_pos; // Vị trí của cơ bản

  Eigen::MatrixXd l_foot_pose = Eigen::MatrixXd::Identity(4, 4);
  l_foot_pose.block<3, 3>(0, 0) = des_l_foot_rot; // Góc quay của chân trái
  l_foot_pose.block<3, 1>(0, 3) = des_l_foot_pos; // Vị trí của chân trái

  Eigen::MatrixXd r_foot_pose = Eigen::MatrixXd::Identity(4, 4);
  r_foot_pose.block<3, 3>(0, 0) = des_r_foot_rot; // Góc quay của chân phải
  r_foot_pose.block<3, 1>(0, 3) = des_r_foot_pos; // Vị trí của chân phải

  // ===== Transformation =====
  Eigen::MatrixXd robot_to_body = Eigen::MatrixXd::Identity(4, 4);     // Ma trận đồng nhất (identity matrix)
  Eigen::MatrixXd robot_to_l_foot = body_pose.inverse() * l_foot_pose; // Biến đổi từ hệ tọa độ của cơ bản đến chân trái
  Eigen::MatrixXd robot_to_r_foot = body_pose.inverse() * r_foot_pose; // Biến đổi từ hệ tọa độ của cơ bản đến chân phải
  // =====

  // Set IMU
  imu_data_mutex_lock_.lock(); // Khóa mutex để đảm bảo an toàn khi truy cập dữ liệu từ cảm biến IMU

  // Thiết lập dữ liệu góc quay từ cảm biến giảm chấn (gyro)
  balance_control_.setCurrentGyroSensorOutput(imu_data_msg_.angular_velocity.x, imu_data_msg_.angular_velocity.y);

  // Tạo đối tượng Quaternion từ dữ liệu góc quay của cảm biến hướng (orientation)
  Eigen::Quaterniond imu_quaternion(imu_data_msg_.orientation.w,
                                    imu_data_msg_.orientation.x,
                                    imu_data_msg_.orientation.y,
                                    imu_data_msg_.orientation.z);

  // Chuyển đổi Quaternion thành Roll-Pitch-Yaw (RPY)
  Eigen::MatrixXd imu_rpy =
      robotis_framework::convertRotationToRPY(robotis_framework::getRotationX(M_PI) * imu_quaternion.toRotationMatrix() * robotis_framework::getRotationZ(M_PI));

  imu_data_mutex_lock_.unlock(); // Mở khóa mutex sau khi đã xử lý xong dữ liệu từ cảm biến IMU

  // Set FT
  // Thiết lập thông tin từ cảm biến lực/moment (FT) cho chân phải
  Eigen::MatrixXd robot_to_r_foot_force =
      robot_to_r_foot.block(0, 0, 3, 3) * robotis_framework::getRotationX(M_PI) *
      robotis_framework::getTransitionXYZ(r_foot_ft_data_msg_.force.x, r_foot_ft_data_msg_.force.y, r_foot_ft_data_msg_.force.z);

  Eigen::MatrixXd robot_to_r_foot_torque =
      robot_to_r_foot.block(0, 0, 3, 3) * robotis_framework::getRotationX(M_PI) *
      robotis_framework::getTransitionXYZ(r_foot_ft_data_msg_.torque.x, r_foot_ft_data_msg_.torque.y, r_foot_ft_data_msg_.torque.z);

  // Thiết lập thông tin từ cảm biến lực/moment (FT) cho chân trái
  Eigen::MatrixXd robot_to_l_foot_force =
      robot_to_l_foot.block(0, 0, 3, 3) * robotis_framework::getRotationX(M_PI) *
      robotis_framework::getTransitionXYZ(l_foot_ft_data_msg_.force.x, l_foot_ft_data_msg_.force.y, l_foot_ft_data_msg_.force.z);

  Eigen::MatrixXd robot_to_l_foot_torque =
      robot_to_l_foot.block(0, 0, 3, 3) * robotis_framework::getRotationX(M_PI) *
      robotis_framework::getTransitionXYZ(l_foot_ft_data_msg_.torque.x, l_foot_ft_data_msg_.torque.y, l_foot_ft_data_msg_.torque.z);

  // Cập nhật góc Roll-Pitch-Yaw (RPY) từ cảm biến IMU
  balance_control_.setCurrentOrientationSensorOutput(imu_rpy.coeff(0, 0), imu_rpy.coeff(1, 0));

  // Cập nhật đầu ra cảm biến lực/moment (FT) và cảm biến giảm chấn góc quay cho cả hai chân của robot
  balance_control_.setCurrentFootForceTorqueSensorOutput(robot_to_r_foot_force.coeff(0, 0), robot_to_r_foot_force.coeff(1, 0), robot_to_r_foot_force.coeff(2, 0),
                                                         robot_to_r_foot_torque.coeff(0, 0), robot_to_r_foot_torque.coeff(1, 0), robot_to_r_foot_torque.coeff(2, 0),
                                                         robot_to_l_foot_force.coeff(0, 0), robot_to_l_foot_force.coeff(1, 0), robot_to_l_foot_force.coeff(2, 0),
                                                         robot_to_l_foot_torque.coeff(0, 0), robot_to_l_foot_torque.coeff(1, 0), robot_to_l_foot_torque.coeff(2, 0));

  // Thiết lập giá trị mong muốn cho kiểm soát cân bằng
  balance_control_.setDesiredCOBGyro(0.0, 0.0);
  balance_control_.setDesiredCOBOrientation(des_body_rpy.coeff(0, 0), des_body_rpy.coeff(1, 0));

  balance_control_.setDesiredFootForceTorque(balance_r_foot_force_x_, balance_r_foot_force_y_, balance_r_foot_force_z_,
                                             balance_r_foot_torque_x_, balance_r_foot_torque_y_, balance_r_foot_torque_z_,
                                             balance_l_foot_force_x_, balance_l_foot_force_y_, balance_l_foot_force_z_,
                                             balance_l_foot_torque_x_, balance_l_foot_torque_y_, balance_l_foot_torque_z_);

  // Thiết lập giá trị mong muốn cho vị trí và góc quay của cơ bản và chân
  balance_control_.setDesiredPose(robot_to_body, robot_to_r_foot, robot_to_l_foot);

  // Xử lý kiểm soát
  int error;
  Eigen::MatrixXd robot_to_body_mod, robot_to_r_foot_mod, robot_to_l_foot_mod;
  balance_control_.process(&error, &robot_to_body_mod, &robot_to_r_foot_mod, &robot_to_l_foot_mod);

  // Áp dụng các ma trận biến đổi mới để cập nhật vị trí và góc quay của cơ bản và chân
  // ===== Transformation =====
  Eigen::MatrixXd body_pose_mod = body_pose * robot_to_body_mod;
  Eigen::MatrixXd r_foot_pose_mod = body_pose * robot_to_r_foot_mod;
  Eigen::MatrixXd l_foot_pose_mod = body_pose * robot_to_l_foot_mod;

  // =====
  // Lấy thông tin vị trí và góc quay mới của cơ bản và chân sau khi kiểm soát
  Eigen::MatrixXd des_body_rot_mod = body_pose_mod.block<3, 3>(0, 0);
  Eigen::MatrixXd des_body_pos_mod = body_pose_mod.block<3, 1>(0, 3);

  Eigen::MatrixXd des_r_foot_rot_mod = r_foot_pose_mod.block<3, 3>(0, 0);
  Eigen::MatrixXd des_r_foot_pos_mod = r_foot_pose_mod.block<3, 1>(0, 3);
  Eigen::MatrixXd des_l_foot_rot_mod = l_foot_pose_mod.block<3, 3>(0, 0);
  Eigen::MatrixXd des_l_foot_pos_mod = l_foot_pose_mod.block<3, 1>(0, 3);

  // Khởi tạo solver giải phương trình nghịch đảo
  op3_kdl_->initialize(des_body_pos_mod, des_body_rot_mod);

  // Lấy giá trị mong muốn của các khớp chân từ thông tin vị trí mong muốn
  Eigen::VectorXd r_leg_joint_pos, l_leg_joint_pos;

  r_leg_joint_pos.resize(6);
  r_leg_joint_pos(0) = des_joint_pos_[joint_name_to_id_["r_hip_yaw"] - 1];
  r_leg_joint_pos(1) = des_joint_pos_[joint_name_to_id_["r_hip_roll"] - 1];
  r_leg_joint_pos(2) = des_joint_pos_[joint_name_to_id_["r_hip_pitch"] - 1];
  r_leg_joint_pos(3) = des_joint_pos_[joint_name_to_id_["r_knee"] - 1];
  r_leg_joint_pos(4) = des_joint_pos_[joint_name_to_id_["r_ank_pitch"] - 1];
  r_leg_joint_pos(5) = des_joint_pos_[joint_name_to_id_["r_ank_roll"] - 1];

  l_leg_joint_pos.resize(6);
  l_leg_joint_pos(0) = des_joint_pos_[joint_name_to_id_["l_hip_yaw"] - 1];
  l_leg_joint_pos(1) = des_joint_pos_[joint_name_to_id_["l_hip_roll"] - 1];
  l_leg_joint_pos(2) = des_joint_pos_[joint_name_to_id_["l_hip_pitch"] - 1];
  l_leg_joint_pos(3) = des_joint_pos_[joint_name_to_id_["l_knee"] - 1];
  l_leg_joint_pos(4) = des_joint_pos_[joint_name_to_id_["l_ank_pitch"] - 1];
  l_leg_joint_pos(5) = des_joint_pos_[joint_name_to_id_["l_ank_roll"] - 1];

  // Thiết lập giá trị của các khớp chân cho solver
  op3_kdl_->setJointPosition(r_leg_joint_pos, l_leg_joint_pos);

  // Khởi tạo các biến lưu trữ đầu ra của solver
  std::vector<double_t> r_leg_output, l_leg_output;

  // Chuyển đổi góc quay mong muốn của chân thành đối tượng Quaternion
  Eigen::Quaterniond des_r_foot_Q_mod = robotis_framework::convertRotationToQuaternion(des_r_foot_rot_mod);
  Eigen::Quaterniond des_l_foot_Q_mod = robotis_framework::convertRotationToQuaternion(des_l_foot_rot_mod);

  // Giải nghịch đảo để tính toán giá trị của các khớp chân
  ik_success = op3_kdl_->solveInverseKinematics(r_leg_output,
                                                des_r_foot_pos_mod, des_r_foot_Q_mod,
                                                l_leg_output,
                                                des_l_foot_pos_mod, des_l_foot_Q_mod);

  // Kết thúc quá trình giải nghịch đảo
  op3_kdl_->finalize();

  // Kiểm tra xem giải nghịch đảo có thành công không
  if (ik_success == true)
  {
    // Nếu thành công, cập nhật giá trị mong muốn của các khớp chân
    des_joint_pos_[joint_name_to_id_["r_hip_yaw"] - 1] = r_leg_output[0];
    des_joint_pos_[joint_name_to_id_["r_hip_roll"] - 1] = r_leg_output[1];
    des_joint_pos_[joint_name_to_id_["r_hip_pitch"] - 1] = r_leg_output[2];
    des_joint_pos_[joint_name_to_id_["r_knee"] - 1] = r_leg_output[3];
    des_joint_pos_[joint_name_to_id_["r_ank_pitch"] - 1] = r_leg_output[4];
    des_joint_pos_[joint_name_to_id_["r_ank_roll"] - 1] = r_leg_output[5];

    des_joint_pos_[joint_name_to_id_["l_hip_yaw"] - 1] = l_leg_output[0];
    des_joint_pos_[joint_name_to_id_["l_hip_roll"] - 1] = l_leg_output[1];
    des_joint_pos_[joint_name_to_id_["l_hip_pitch"] - 1] = l_leg_output[2];
    des_joint_pos_[joint_name_to_id_["l_knee"] - 1] = l_leg_output[3];
    des_joint_pos_[joint_name_to_id_["l_ank_pitch"] - 1] = l_leg_output[4];
    des_joint_pos_[joint_name_to_id_["l_ank_roll"] - 1] = l_leg_output[5];
  }

  // Trả về kết quả của quá trình giải nghịch đảo (true nếu thành công, false nếu không)
  return ik_success;
}

void OnlineWalkingModule::setFeedbackControl()
{
  // Duyệt qua tất cả các khớp của robot
  for (int i = 0; i < number_of_joints_; i++)
  {
    // Tính toán giá trị mong muốn của khớp với phần feedforward
    des_joint_pos_to_robot_[i] = des_joint_pos_[i] + des_joint_feedforward_[i];

    // Thiết lập giá trị mong muốn cho điều khiển phản hồi
    joint_feedback_[i].desired_ = des_joint_pos_[i];

    // Lấy thông tin phản hồi từ cảm biến về vị trí hiện tại của khớp
    des_joint_feedback_[i] = joint_feedback_[i].getFeedBack(curr_joint_pos_[i]);

    // Cộng thêm thông tin phản hồi vào giá trị mong muốn của khớp
    des_joint_pos_to_robot_[i] += des_joint_feedback_[i];
  }
}

void OnlineWalkingModule::setFeedforwardControl()
{
  // Tính thời gian hiện tại dựa trên bước chuyển động và chu kỳ điều khiển
  double cur_time = (double)mov_step_ * control_cycle_sec_;

  // Lấy giá trị feedforward tại thời điểm hiện tại
  std::vector<double_t> feed_forward_value = feed_forward_tra_->getPosition(cur_time);

  // Nếu đang ở giai đoạn Double Support (DSP), giảm giá trị feedforward về 0
  if (walking_phase_ == DSP)
    feed_forward_value[0] = 0.0;

  // Khởi tạo vector hệ số gain cho các khớp chân hỗ trợ
  std::vector<double_t> support_leg_gain;
  support_leg_gain.resize(number_of_joints_, 0.0);

  // Thiết lập hệ số gain cho các khớp chân hỗ trợ dựa trên chân đang di chuyển
  if (walking_leg_ == LEFT_LEG)
  {
    // Nếu chân trái đang di chuyển, hệ số gain cho các khớp chân phải là 0 và cho chân trái là 1
    support_leg_gain[joint_name_to_id_["r_hip_yaw"] - 1] = 1.0;
    support_leg_gain[joint_name_to_id_["r_hip_roll"] - 1] = 1.0;
    support_leg_gain[joint_name_to_id_["r_hip_pitch"] - 1] = 1.0;
    support_leg_gain[joint_name_to_id_["r_knee"] - 1] = 1.0;
    support_leg_gain[joint_name_to_id_["r_ank_pitch"] - 1] = 1.0;
    support_leg_gain[joint_name_to_id_["r_ank_roll"] - 1] = 1.0;

    support_leg_gain[joint_name_to_id_["l_hip_yaw"] - 1] = 0.0;
    support_leg_gain[joint_name_to_id_["l_hip_roll"] - 1] = 0.0;
    support_leg_gain[joint_name_to_id_["l_hip_pitch"] - 1] = 0.0;
    support_leg_gain[joint_name_to_id_["l_knee"] - 1] = 0.0;
    support_leg_gain[joint_name_to_id_["l_ank_pitch"] - 1] = 0.0;
    support_leg_gain[joint_name_to_id_["l_ank_roll"] - 1] = 0.0;
  }
  else if (walking_leg_ == RIGHT_LEG)
  {
    // Nếu chân phải đang di chuyển, hệ số gain cho các khớp chân phải là 0 và cho chân trái là 1
    support_leg_gain[joint_name_to_id_["r_hip_yaw"] - 1] = 0.0;
    support_leg_gain[joint_name_to_id_["r_hip_roll"] - 1] = 0.0;
    support_leg_gain[joint_name_to_id_["r_hip_pitch"] - 1] = 0.0;
    support_leg_gain[joint_name_to_id_["r_knee"] - 1] = 0.0;
    support_leg_gain[joint_name_to_id_["r_ank_pitch"] - 1] = 0.0;
    support_leg_gain[joint_name_to_id_["r_ank_roll"] - 1] = 0.0;

    support_leg_gain[joint_name_to_id_["l_hip_yaw"] - 1] = 1.0;
    support_leg_gain[joint_name_to_id_["l_hip_roll"] - 1] = 1.0;
    support_leg_gain[joint_name_to_id_["l_hip_pitch"] - 1] = 1.0;
    support_leg_gain[joint_name_to_id_["l_knee"] - 1] = 1.0;
    support_leg_gain[joint_name_to_id_["l_ank_pitch"] - 1] = 1.0;
    support_leg_gain[joint_name_to_id_["l_ank_roll"] - 1] = 1.0;
  }
  // Tính toán giá trị feedforward cho từng khớp chân và áp dụng hệ số gain
  for (int i = 0; i < number_of_joints_; i++)
    des_joint_feedforward_[i] = joint_feedforward_gain_[i] * feed_forward_value[0] * support_leg_gain[i];
}

void OnlineWalkingModule::sensoryFeedback(const double &rlGyroErr, const double &fbGyroErr, double *balance_angle)
{
  // Điều chỉnh độ lệch cân bằng bằng cách sử dụng hệ số lợi nội tại
  double internal_gain = 0.05;

  // Điều chỉnh góc cân bằng cho khớp R_HIP_ROLL và L_HIP_ROLL dựa trên lệch gia tốc quay theo hướng Roll
  balance_angle[joint_name_to_id_["r_hip_roll"] - 1] =
      -1.0 * internal_gain * rlGyroErr * balance_hip_roll_gain_; // R_HIP_ROLL
  balance_angle[joint_name_to_id_["l_hip_roll"] - 1] =
      -1.0 * internal_gain * rlGyroErr * balance_hip_roll_gain_; // L_HIP_ROLL

  // Điều chỉnh góc cân bằng cho khớp R_KNEE và L_KNEE dựa trên lệch gia tốc quay theo hướng Pitch
  balance_angle[joint_name_to_id_["r_knee"] - 1] =
      1.0 * internal_gain * fbGyroErr * balance_knee_gain_; // R_KNEE
  balance_angle[joint_name_to_id_["l_knee"] - 1] =
      -1.0 * internal_gain * fbGyroErr * balance_knee_gain_; // L_KNEE

  // Điều chỉnh góc cân bằng cho khớp R_ANKLE_PITCH và L_ANKLE_PITCH dựa trên lệch gia tốc quay theo hướng Pitch
  balance_angle[joint_name_to_id_["r_ank_pitch"] - 1] =
      -1.0 * internal_gain * fbGyroErr * balance_ankle_pitch_gain_; // R_ANKLE_PITCH
  balance_angle[joint_name_to_id_["l_ank_pitch"] - 1] =
      1.0 * internal_gain * fbGyroErr * balance_ankle_pitch_gain_; // L_ANKLE_PITCH

  // Điều chỉnh góc cân bằng cho khớp R_ANKLE_ROLL và L_ANKLE_ROLL dựa trên lệch gia tốc quay theo hướng Roll
  balance_angle[joint_name_to_id_["r_ank_roll"] - 1] =
      -1.0 * internal_gain * rlGyroErr * balance_ankle_roll_gain_; // R_ANKLE_ROLL
  balance_angle[joint_name_to_id_["l_ank_roll"] - 1] =
      -1.0 * internal_gain * rlGyroErr * balance_ankle_roll_gain_; // L_ANKLE_ROLL
}

void OnlineWalkingModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                                  std::map<std::string, double> sensors)
{
  // Nếu chức năng bị tắt, thoát khỏi hàm
  if (enable_ == false)
    return;

  // Khởi tạo mảng lưu độ lệch cân bằng cho từng khớp chân
  double balance_angle[number_of_joints_];
  for (int i = 0; i < number_of_joints_; i++)
    balance_angle[i] = 0.0;

  // Tính lệch gia tốc quay để điều chỉnh cân bằng
  double rl_gyro_err = 0.0 - sensors["gyro_x"];
  double fb_gyro_err = 0.0 - sensors["gyro_y"];

  // Áp dụng phản hồi từ cảm biến gia tốc quay để điều chỉnh góc cân bằng của các khớp chân
  sensoryFeedback(rl_gyro_err, fb_gyro_err, balance_angle);

  // Ghi vị trí hiện tại của các khớp chân
  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
       state_iter != result_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;

    // Lấy thông tin động cơ từ danh sách
    robotis_framework::Dynamixel *dxl = NULL;
    std::map<std::string, robotis_framework::Dynamixel *>::iterator dxl_it = dxls.find(joint_name);
    if (dxl_it != dxls.end())
      dxl = dxl_it->second;
    else
      continue;

    // Lấy vị trí hiện tại và vị trí mục tiêu của khớp chân
    double curr_joint_pos = dxl->dxl_state_->present_position_;
    double goal_joint_pos = dxl->dxl_state_->goal_position_;

    // Nếu chưa khởi tạo mục tiêu, gán mục tiêu bằng vị trí mục tiêu của động cơ
    if (goal_initialize_ == false)
      des_joint_pos_[joint_name_to_id_[joint_name] - 1] = goal_joint_pos;

    // Lưu vị trí hiện tại vào mảng
    curr_joint_pos_[joint_name_to_id_[joint_name] - 1] = curr_joint_pos;
  }

  goal_initialize_ = true;

  /* Trajectory Calculation */
  ros::Time begin = ros::Time::now();

  // Kiểm tra loại kiểm soát và thực hiện các bước tương ứng
  if (control_type_ == JOINT_CONTROL)
  {
    initJointControl(); // Khởi tạo kiểm soát theo khớp chân
    calcJointControl(); // Tính toán kiểm soát theo khớp chân
  }
  else if (control_type_ == WHOLEBODY_CONTROL)
  {
    initWholebodyControl(); // Khởi tạo kiểm soát toàn bộ cơ thể
    calcWholebodyControl(); // Tính toán kiểm soát toàn bộ cơ thể
  }
  else if (control_type_ == WALKING_CONTROL)
  {
    // Nếu đã khởi tạo đi bộ
    if (walking_initialize_ == true)
    {
      calcWalkingControl();    // Tính toán kiểm soát cho việc đi bộ
      setFeedforwardControl(); // Thiết lập kiểm soát feedforward
    }
  }
  else if (control_type_ == OFFSET_CONTROL)
  {
    initOffsetControl(); // Khởi tạo kiểm soát độ lệch
    calcOffsetControl(); // Tính toán kiểm soát độ lệch
  }

  // Tính toán vị trí của robot
  // calcRobotPose();

  // Nếu kiểm soát cân bằng được bật
  if (balance_type_ == ON)
  {
    initBalanceControl(); // Khởi tạo kiểm soát cân bằng
    calcBalanceControl(); // Tính toán kiểm soát cân bằng

    // Nếu thiết lập kiểm soát cân bằng không thành công
    if (setBalanceControl() == false)
    {
      // Đặt các biến trạng thái về giá trị mặc định
      is_moving_ = false;
      is_balancing_ = false;
      is_foot_step_2d_ = false;

      balance_type_ = OFF;
      control_type_ = NONE;

      resetBodyPose(); // Thiết lập lại vị trí cơ thể

      ROS_INFO("[FAIL] Task Space Control"); // Thông báo lỗi
    }
  }

  ros::Duration time_duration = ros::Time::now() - begin;

  // Kiểm tra thời gian tính toán và thông báo nếu thời gian lớn hơn ngưỡng
  if (time_duration.toSec() > 0.003)
    ROS_INFO("[Wholebody Module] Calc Time: %f", time_duration.toSec());

  setFeedbackControl(); // Thiết lập kiểm soát phản hồi

  // Áp dụng góc cân bằng cho các khớp chân
  for (int i = 0; i < number_of_joints_; i++)
    des_joint_pos_to_robot_[i] += balance_angle[i];

  sensor_msgs::JointState goal_joint_msg;     // Tin nhắn chứa mục tiêu của các khớp chân
  geometry_msgs::PoseStamped pelvis_pose_msg; // Tin nhắn chứa vị trí và hướng của pelvis

  goal_joint_msg.header.stamp = ros::Time::now();
  pelvis_pose_msg.header.stamp = ros::Time::now();

  // Thiết lập vị trí và hướng của pelvis
  pelvis_pose_msg.pose.position.x = des_body_pos_[0];
  pelvis_pose_msg.pose.position.y = des_body_pos_[1];
  pelvis_pose_msg.pose.position.z = des_body_pos_[2] - 0.0907;

  pelvis_pose_msg.pose.orientation.x = des_body_Q_[0];
  pelvis_pose_msg.pose.orientation.y = des_body_Q_[1];
  pelvis_pose_msg.pose.orientation.z = des_body_Q_[2];
  pelvis_pose_msg.pose.orientation.w = des_body_Q_[3];

  // Thiết lập dữ liệu cho các khớp chân
  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
       state_iter != result_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;
    result_[joint_name]->goal_position_ = des_joint_pos_to_robot_[joint_name_to_id_[joint_name] - 1];

    // Đưa dữ liệu vào tin nhắn mục tiêu của các khớp chân
    goal_joint_msg.name.push_back(joint_name);
    goal_joint_msg.position.push_back(des_joint_pos_[joint_name_to_id_[joint_name] - 1]);
  }

  // Xuất bản tin nhắn vị trí của pelvis và mục tiêu của các khớp chân
  pelvis_pose_pub_.publish(pelvis_pose_msg);
  goal_joint_state_pub_.publish(goal_joint_msg);
}

void OnlineWalkingModule::stop()
{
  // Đặt mục tiêu của các khớp chân, vận tốc và gia tốc về 0
  for (int i = 0; i < number_of_joints_; i++)
  {
    des_joint_pos_[i] = 0.0;
    des_joint_vel_[i] = 0.0;
    des_joint_accel_[i] = 0.0;
  }

  // Đặt lại trạng thái khởi tạo của mục tiêu
  goal_initialize_ = false;

  // Tắt cờ di chuyển và cân bằng
  is_moving_ = false;
  is_balancing_ = false;

  // Đặt lại trạng thái khởi tạo của các loại kiểm soát
  joint_control_initialize_ = false;
  wholebody_initialize_ = false;
  walking_initialize_ = false;
  balance_control_initialize_ = false;

  // Đặt loại kiểm soát về NONE
  control_type_ = NONE;

  return;
}

// Kiểm tra xem module có đang trong quá trình di chuyển hay không.
bool OnlineWalkingModule::isRunning()
{
  return is_moving_;
}

// Xuất bản một thông báo trạng thái của module.
void OnlineWalkingModule::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg status;
  status.header.stamp = ros::Time::now();
  status.type = type;
  status.module_name = "Wholebody";
  status.status_msg = msg;

  status_msg_pub_.publish(status);
}

// Đáp ứng yêu cầu nhận giá trị góc khớp của module.
bool OnlineWalkingModule::getJointPoseCallback(op3_online_walking_module_msgs::GetJointPose::Request &req,
                                               op3_online_walking_module_msgs::GetJointPose::Response &res)
{
  for (int i = 0; i < number_of_joints_; i++)
  {
    // Thêm tên và giá trị góc của từng khớp vào phản hồi của yêu cầu.
    res.pose.pose.name.push_back(joint_name_[i]);
    res.pose.pose.position.push_back(des_joint_pos_[i]);
  }

  // Trả về true để thông báo rằng yêu cầu đã được xử lý thành công.
  return true;
}

// Đáp ứng yêu cầu nhận giá trị vị trí và góc của các nhóm trong mô hình học của module.
bool OnlineWalkingModule::getKinematicsPoseCallback(op3_online_walking_module_msgs::GetKinematicsPose::Request &req,
                                                    op3_online_walking_module_msgs::GetKinematicsPose::Response &res)
{
  // Lấy tên của nhóm cần truy vấn.
  std::string group_name = req.name;

  // Tạo một biến để lưu giữ giá trị vị trí và góc của nhóm được truy vấn.
  geometry_msgs::Pose msg;

  // Kiểm tra nhóm và thiết lập giá trị vị trí và góc tương ứng.
  if (group_name == "body")
  {
    msg.position.x = des_body_pos_[0];
    msg.position.y = des_body_pos_[1];
    msg.position.z = des_body_pos_[2];

    msg.orientation.x = des_body_Q_[0];
    msg.orientation.y = des_body_Q_[1];
    msg.orientation.z = des_body_Q_[2];
    msg.orientation.w = des_body_Q_[3];
  }
  else if (group_name == "left_leg")
  {
    msg.position.x = des_l_leg_pos_[0];
    msg.position.y = des_l_leg_pos_[1];
    msg.position.z = des_l_leg_pos_[2];

    msg.orientation.x = des_l_leg_Q_[0];
    msg.orientation.y = des_l_leg_Q_[1];
    msg.orientation.z = des_l_leg_Q_[2];
    msg.orientation.w = des_l_leg_Q_[3];
  }
  else if (group_name == "right_leg")
  {
    msg.position.x = des_r_leg_pos_[0];
    msg.position.y = des_r_leg_pos_[1];
    msg.position.z = des_r_leg_pos_[2];

    msg.orientation.x = des_r_leg_Q_[0];
    msg.orientation.y = des_r_leg_Q_[1];
    msg.orientation.z = des_r_leg_Q_[2];
    msg.orientation.w = des_r_leg_Q_[3];
  }

  // Gán giá trị vị trí và góc vào phản hồi của yêu cầu.
  res.pose.pose = msg;

  // Trả về true để thông báo rằng yêu cầu đã được xử lý thành công.
  return true;
}

// Định nghĩa ma trận xem trước cho hệ thống điều khiển.
bool OnlineWalkingModule::definePreviewMatrix()
{
  // Định nghĩa ma trận K.
  std::vector<double_t> K;
  K.push_back(739.200064);
  K.push_back(24489.822984);
  K.push_back(3340.410380);
  K.push_back(69.798325);

  // Gán giá trị ma trận K và kích thước của nó.
  preview_response_K_ = K;
  preview_response_K_row_ = 1;
  preview_response_K_col_ = 4;

  // Định nghĩa ma trận P.
  std::vector<double_t> P;
  P.push_back(33.130169);
  P.push_back(531.738962);
  P.push_back(60.201291);
  P.push_back(0.327533);
  P.push_back(531.738962);
  P.push_back(10092.440286);
  P.push_back(1108.851055);
  P.push_back(7.388990);
  P.push_back(60.201291);
  P.push_back(1108.851055);
  P.push_back(130.194694);
  P.push_back(0.922502);
  P.push_back(0.327533);
  P.push_back(7.388990);
  P.push_back(0.922502);
  P.push_back(0.012336);

  // Gán giá trị ma trận P và kích thước của nó.
  preview_response_P_ = P;
  preview_response_P_row_ = 4;
  preview_response_P_col_ = 4;

  // Trả về true để thông báo rằng đã định nghĩa ma trận xem trước thành công.
  return true;
}

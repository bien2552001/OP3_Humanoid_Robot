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

#include "op3_walking_module/op3_walking_module.h"

namespace robotis_op
{

  // Constructor của lớp WalkingModule
  WalkingModule::WalkingModule()
      : control_cycle_msec_(8), // Đặt chu kỳ kiểm soát mặc định là 8 miligiây
        DEBUG(false)            // Chế độ debug được tắt mặc định
  {
    enable_ = false;                                    // Module không được kích hoạt (enable) khi mới tạo
    module_name_ = "walking_module";                    // Đặt tên cho module là "walking_module"
    control_mode_ = robotis_framework::PositionControl; // Chế độ kiểm soát là PositionControl (kiểm soát theo vị trí)

    init_pose_count_ = 0;             // Số lần khởi tạo pose ban đầu được đặt về 0
    walking_state_ = WalkingReady;    // Trạng thái của quá trình đi bộ được đặt là WalkingReady
    previous_x_move_amplitude_ = 0.0; // Biên độ di chuyển theo trục x trước đó được đặt là 0.0

    op3_kd_ = new OP3KinematicsDynamics(WholeBody); // Khởi tạo đối tượng OP3KinematicsDynamics với kiểu là WholeBody

    // result
    // Khởi tạo các đối tượng DynamixelState cho các khớp cơ bản
    result_["r_hip_yaw"] = new robotis_framework::DynamixelState();   // Đối tượng DynamixelState cho khớp r_hip_yaw
    result_["r_hip_roll"] = new robotis_framework::DynamixelState();  // Đối tượng DynamixelState cho khớp r_hip_roll
    result_["r_hip_pitch"] = new robotis_framework::DynamixelState(); // Đối tượng DynamixelState cho khớp r_hip_pitch
    result_["r_knee"] = new robotis_framework::DynamixelState();      // Đối tượng DynamixelState cho khớp r_knee
    result_["r_ank_pitch"] = new robotis_framework::DynamixelState(); // Đối tượng DynamixelState cho khớp r_ank_pitch
    result_["r_ank_roll"] = new robotis_framework::DynamixelState();  // Đối tượng DynamixelState cho khớp r_ank_roll

    result_["l_hip_yaw"] = new robotis_framework::DynamixelState();   // Đối tượng DynamixelState cho khớp l_hip_yaw
    result_["l_hip_roll"] = new robotis_framework::DynamixelState();  // Đối tượng DynamixelState cho khớp l_hip_roll
    result_["l_hip_pitch"] = new robotis_framework::DynamixelState(); // Đối tượng DynamixelState cho khớp l_hip_pitch
    result_["l_knee"] = new robotis_framework::DynamixelState();      // Đối tượng DynamixelState cho khớp l_knee
    result_["l_ank_pitch"] = new robotis_framework::DynamixelState(); // Đối tượng DynamixelState cho khớp l_ank_pitch
    result_["l_ank_roll"] = new robotis_framework::DynamixelState();  // Đối tượng DynamixelState cho khớp l_ank_roll

    result_["r_sho_pitch"] = new robotis_framework::DynamixelState(); // Đối tượng DynamixelState cho khớp r_sho_pitch
    result_["l_sho_pitch"] = new robotis_framework::DynamixelState(); // Đối tượng DynamixelState cho khớp l_sho_pitch

    // joint table
    // Khởi tạo bảng ánh xạ giữa tên khớp và chỉ số tương ứng trong ma trận thông tin khớp
    joint_table_["r_hip_yaw"] = 0;   // Chỉ số tương ứng của khớp r_hip_yaw trong ma trận thông tin khớp
    joint_table_["r_hip_roll"] = 1;  // Chỉ số tương ứng của khớp r_hip_roll trong ma trận thông tin khớp
    joint_table_["r_hip_pitch"] = 2; // Chỉ số tương ứng của khớp r_hip_pitch trong ma trận thông tin khớp
    joint_table_["r_knee"] = 3;      // Chỉ số tương ứng của khớp r_knee trong ma trận thông tin khớp
    joint_table_["r_ank_pitch"] = 4;
    joint_table_["r_ank_roll"] = 5;

    joint_table_["l_hip_yaw"] = 6;
    joint_table_["l_hip_roll"] = 7;
    joint_table_["l_hip_pitch"] = 8;
    joint_table_["l_knee"] = 9;
    joint_table_["l_ank_pitch"] = 10;
    joint_table_["l_ank_roll"] = 11;

    joint_table_["r_sho_pitch"] = 12;
    joint_table_["l_sho_pitch"] = 13;

    // Khởi tạo ma trận thông tin vị trí mục tiêu, vị trí mong muốn, vị trí khởi tạo, và hướng của các khớp
    target_position_ = Eigen::MatrixXd::Zero(1, result_.size());      // Ma trận vị trí mục tiêu, khởi tạo toàn bộ giá trị là 0
    goal_position_ = Eigen::MatrixXd::Zero(1, result_.size());        // Ma trận vị trí mong muốn, khởi tạo toàn bộ giá trị là 0
    init_position_ = Eigen::MatrixXd::Zero(1, result_.size());        // Ma trận vị trí khởi tạo, khởi tạo toàn bộ giá trị là 0
    joint_axis_direction_ = Eigen::MatrixXi::Zero(1, result_.size()); // Ma trận hướng của các khớp, khởi tạo toàn bộ giá trị là 0
  }

  // Hủy đối tượng WalkingModule: đợi cho đến khi thread queue_thread_ kết thúc trước khi tiếp tục hủy đối tượng.
  WalkingModule::~WalkingModule()
  {
    queue_thread_.join();
  }

  void WalkingModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
  {
    // Khởi tạo đối tượng thread để xử lý hàng đợi (queue)
    queue_thread_ = boost::thread(boost::bind(&WalkingModule::queueThread, this));

    // Lưu trữ chu kỳ kiểm soát và con trỏ đến đối tượng Robot
    control_cycle_msec_ = control_cycle_msec;

    // Thiết lập các giá trị khởi tạo cho các tham số của bước đi
    // Đơn vị: mét (m), giây (s), radian (rad)
    // Pose ban đầu
    // Các tham số khởi tạo vị trí và góc của robot khi bắt đầu đi bộ
    walking_param_.init_x_offset = -0.010;                  // Dịch chuyển ban đầu theo trục x
    walking_param_.init_y_offset = 0.005;                   // Dịch chuyển ban đầu theo trục y
    walking_param_.init_z_offset = 0.020;                   // Dịch chuyển ban đầu theo trục z (chiều cao)
    walking_param_.init_roll_offset = 0.0;                  // Góc roll ban đầu của robot
    walking_param_.init_pitch_offset = 0.0 * DEGREE2RADIAN; // Góc pitch ban đầu của robot (đổi từ độ sang radian)
    walking_param_.init_yaw_offset = 0.0 * DEGREE2RADIAN;   // Góc yaw ban đầu của robot (đổi từ độ sang radian)
    walking_param_.hip_pitch_offset = 13.0 * DEGREE2RADIAN; // Góc pitch của khớp hông khi bắt đầu đi bộ (đổi từ độ sang radian)

    // Các tham số thời gian điều chỉnh bước đi
    walking_param_.period_time = 600 * 0.001; // Thời gian cho một chu kỳ hoàn chỉnh của bước đi (đơn vị: giây)
    walking_param_.dsp_ratio = 0.1;           // Tỷ lệ thời gian chuyển đổi giữa giai đoạn Double Support và Single Support
    walking_param_.step_fb_ratio = 0.28;      // Tỷ lệ giữa chuyển động trước và chuyển động sau trong mỗi bước đi

    // Các tham số điều chỉnh chuyển động khi đi bộ
    walking_param_.x_move_amplitude = 0.0;     // Biên độ chuyển động di chuyển theo trục x
    walking_param_.y_move_amplitude = 0.0;     // Biên độ chuyển động di chuyển theo trục y
    walking_param_.z_move_amplitude = 0.040;   // Chiều cao của chân khi di chuyển
    walking_param_.angle_move_amplitude = 0.0; // Biên độ chuyển động xoay của cơ thể khi đi bộ

    // Các tham số điều chỉnh cân bằng trong bước đi
    walking_param_.balance_enable = false;              // Bật/tắt chức năng cân bằng
    walking_param_.balance_hip_roll_gain = 0.5;         // Hệ số cân bằng cho xoay hông theo trục roll
    walking_param_.balance_knee_gain = 0.3;             // Hệ số cân bằng cho góc khớp đầu gối
    walking_param_.balance_ankle_roll_gain = 1.0;       // Hệ số cân bằng cho xoay mắt cá chân theo trục roll
    walking_param_.balance_ankle_pitch_gain = 0.9;      // Hệ số cân bằng cho xoay mắt cá chân theo trục pitch
    walking_param_.y_swap_amplitude = 0.020;            // Biên độ chuyển động thay đổi hướng di chuyển theo trục y
    walking_param_.z_swap_amplitude = 0.005;            // Biên độ chuyển động thay đổi hướng di chuyển theo trục z
    walking_param_.pelvis_offset = 3.0 * DEGREE2RADIAN; // Độ lệch của cơ thể từ trục pelvis theo trục yaw
    walking_param_.arm_swing_gain = 1.5;                // Hệ số điều chỉnh chuyển động đuôi tay khi đi bộ

    // Biến thành viên để lưu trữ góc swing của cơ thể theo trục y và trục z
    body_swing_y = 0;
    body_swing_z = 0;

    // Các biến điều chỉnh pha và biên độ của các chuyển động
    x_swap_phase_shift_ = M_PI;         // Pha chuyển động thay đổi hướng di chuyển theo trục x
    x_swap_amplitude_shift_ = 0;        // Biên độ chuyển động thay đổi hướng di chuyển theo trục x
    x_move_phase_shift_ = M_PI / 2;     // Pha chuyển động di chuyển theo trục x
    x_move_amplitude_shift_ = 0;        // Biên độ chuyển động di chuyển theo trục x
    y_swap_phase_shift_ = 0;            // Pha chuyển động thay đổi hướng di chuyển theo trục y
    y_swap_amplitude_shift_ = 0;        // Biên độ chuyển động thay đổi hướng di chuyển theo trục y
    y_move_phase_shift_ = M_PI / 2;     // Pha chuyển động di chuyển theo trục y
    z_swap_phase_shift_ = M_PI * 3 / 2; // Pha chuyển động thay đổi hướng di chuyển theo trục z
    z_move_phase_shift_ = M_PI / 2;     // Pha chuyển động di chuyển theo trục z
    a_move_phase_shift_ = M_PI / 2;     // Pha chuyển động di chuyển theo góc xoay a

    // Biến để kiểm soát trạng thái chạy
    ctrl_running_ = false; // Trạng thái chạy của bộ điều khiển
    real_running_ = false; // Trạng thái chạy thực tế
    time_ = 0;             // Biến thời gian

    //                     R_HIP_YAW, R_HIP_ROLL, R_HIP_PITCH, R_KNEE, R_ANKLE_PITCH, R_ANKLE_ROLL,
    //                     L_HIP_YAW, L_HIP_ROLL, L_HIP_PITCH, L_KNEE, L_ANKLE_PITCH, L_ANKLE_ROLL,
    //                     R_ARM_SWING, L_ARM_SWING

    // Khởi tạo ma trận chỉ định hướng của các khớp và ma trận vị trí khởi tạo
    joint_axis_direction_ << -1, -1, -1, -1, 1, 1,
        -1, -1, 1, 1, -1, 1,
        1, -1;

    // Khởi tạo ma trận vị trí khởi tạo
    init_position_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        5.0, -5.0;

    // Chuyển đổi giá trị từ độ sang radian
    init_position_ *= DEGREE2RADIAN;

    // Khởi tạo đối tượng NodeHandle của ROS
    ros::NodeHandle ros_node;

    // Đặt đường dẫn mặc định cho tệp cấu hình là param.yaml trong gói op3_walking_module
    std::string default_param_path = ros::package::getPath("op3_walking_module") + "/config/param.yaml";

    // Đọc tham số 'walking_param_path' từ ROS Parameter Server, nếu không có, sử dụng đường dẫn mặc định
    ros_node.param<std::string>("walking_param_path", param_path_, default_param_path);

    // Tải cấu hình đi bộ từ tệp cấu hình được chỉ định
    loadWalkingParam(param_path_);

    // Cập nhật tham số thời gian
    updateTimeParam();

    // Cập nhật tham số chuyển động
    updateMovementParam();
  }

  void WalkingModule::queueThread()
  {
    // Tạo đối tượng NodeHandle và CallbackQueue của ROS
    ros::NodeHandle ros_node;
    ros::CallbackQueue callback_queue;

    // Liên kết CallbackQueue với NodeHandle
    ros_node.setCallbackQueue(&callback_queue);

    // Khởi tạo và đăng ký Publisher cho topic "robotis/status"
    status_msg_pub_ = ros_node.advertise<robotis_controller_msgs::StatusMsg>("robotis/status", 1);

    // Đăng ký ROS Service Callback Functions
    ros::ServiceServer get_walking_param_server = ros_node.advertiseService("/robotis/walking/get_params",
                                                                            &WalkingModule::getWalkigParameterCallback,
                                                                            this);

    // Đăng ký Subscriber cho topic "/robotis/walking/command" với hàm callback "walkingCommandCallback"
    ros::Subscriber walking_command_sub = ros_node.subscribe("/robotis/walking/command", 0,
                                                             &WalkingModule::walkingCommandCallback, this);

    // Đăng ký Subscriber cho topic "/robotis/walking/set_params" với hàm callback "walkingParameterCallback"
    ros::Subscriber walking_param_sub = ros_node.subscribe("/robotis/walking/set_params", 0,
                                                           &WalkingModule::walkingParameterCallback, this);

    // Thiết lập thời gian ngủ dựa trên chu kỳ kiểm soát
    ros::WallDuration duration(control_cycle_msec_ / 1000.0);

    // Vòng lặp chờ và xử lý các sự kiện từ CallbackQueue
    while (ros_node.ok())
      callback_queue.callAvailable(duration);
  }

  void WalkingModule::publishStatusMsg(unsigned int type, std::string msg)
  {
    // Tạo đối tượng StatusMsg để xuất bản
    robotis_controller_msgs::StatusMsg status_msg;

    // Đặt thời điểm cho thông điệp
    status_msg.header.stamp = ros::Time::now();

    // Đặt loại thông điệp và tên mô-đun
    status_msg.type = type;
    status_msg.module_name = "Walking";

    // Đặt nội dung thông điệp
    status_msg.status_msg = msg;

    // Xuất bản thông điệp
    status_msg_pub_.publish(status_msg);
  }

  void WalkingModule::walkingCommandCallback(const std_msgs::String::ConstPtr &msg)
  {
    // Kiểm tra xem module đi bộ đã sẵn sàng chưa
    if (enable_ == false)
    {
      ROS_WARN("walking module is not ready.");
      return;
    }

    // Xử lý lệnh bắt đầu đi bộ
    if (msg->data == "start")
      startWalking(); // Gọi hàm bắt đầu đi bộ
    // Xử lý lệnh dừng lại
    else if (msg->data == "stop")
      stop(); // Gọi hàm dừng lại
    // Kích hoạt cân bằng
    else if (msg->data == "balance on")
      walking_param_.balance_enable = true; // Bật chế độ cân bằng
    // Tắt cân bằng
    else if (msg->data == "balance off")
      walking_param_.balance_enable = false; // Tắt chế độ cân bằng
    // Lưu trạng thái và tham số đi bộ
    else if (msg->data == "save")
      saveWalkingParam(param_path_); // Gọi hàm lưu trạng thái và tham số đi bộ vào đường dẫn đã chỉ định
  }

  void WalkingModule::walkingParameterCallback(const op3_walking_module_msgs::WalkingParam::ConstPtr &msg)
  {
    // Hàm callback được gọi khi nhận được thông số đi bộ từ ROS topic
    walking_param_ = *msg; // Cập nhật thông số đi bộ với giá trị nhận được
  }

  bool WalkingModule::getWalkigParameterCallback(op3_walking_module_msgs::GetWalkingParam::Request &req,
                                                 op3_walking_module_msgs::GetWalkingParam::Response &res)
  {
    // Hàm callback để xử lý yêu cầu lấy thông số đi bộ
    res.parameters = walking_param_; // Trả về thông số đi bộ trong response
    return true;                     // Trả về true để xác nhận rằng yêu cầu đã được xử lý thành công
  }

  double WalkingModule::wSin(double time, double period, double period_shift, double mag, double mag_shift)
  {
    // Hàm tính giá trị của hàm sin có thể được sử dụng trong các tính toán của module đi bộ
    return mag * sin(2 * M_PI / period * time - period_shift) + mag_shift;
  }

  // Hàm tính toán giải phương trình nghịch đảo hình học (IK) cho chân robot
  // Đầu vào:
  //   - pos_x, pos_y, pos_z: tọa độ xyz của điểm đích
  //   - ori_roll, ori_pitch, ori_yaw: góc roll, pitch, yaw của điểm đích
  // Đầu ra:
  //   - out: mảng chứa giải phương trình IK (góc các khớp của chân robot)
  // Ghi chú:
  //   - Các giá trị đặc trưng của chân robot được sử dụng để tính toán (thigh_length, calf_length, ankle_length, leg_length)
  // m, rad
  // for default op3: it was used from previos version(OP2) but it's not using now.
  bool WalkingModule::computeIK(double *out, double pos_x, double pos_y, double pos_z, double ori_roll, double ori_pitch,
                                double ori_yaw)
  {
    // Chiều dài của đùi chân, cơ bắp chân và cổ chân (mét)
    double thigh_length = 93.0 * 0.001; // m
    double calf_length = 93.0 * 0.001;  // m
    double ankle_length = 33.5 * 0.001; // m
    // Tổng chiều dài của chân (đùi + cơ bắp + cổ chân)
    double leg_length = 219.5 * 0.001; // m (THIGH_LENGTH + CALF_LENGTH + ANKLE_LENGTH)

    // Ma trận biến đổi và vector đại diện cho các phép biến đổi hình học
    Eigen::MatrixXd transformation_ad, transformation_da, transformation_cd, transformation_dc, transformation_ac;
    Eigen::Vector3d vector;

    // Các biến phụ trợ trong quá trình tính toán
    double r_ac, acos_value, atan_value, value_k, value_l, value_m, value_n, value_s, value_c, theta;

    // Tạo ma trận biến đổi dựa trên tọa độ và góc quay của điểm đích
    transformation_ad = robotis_framework::getTransformationXYZRPY(pos_x, pos_y, pos_z, ori_roll, ori_pitch, ori_yaw);

    // Tính toán tọa độ của điểm chân cơ bản (vector) từ tọa độ của điểm đích
    // Vector đại diện cho tọa độ của điểm chân cơ bản
    vector << pos_x + transformation_ad.coeff(0, 2) * ankle_length,
        pos_y + transformation_ad.coeff(1, 2) * ankle_length,
        (pos_z - leg_length) + transformation_ad.coeff(2, 2) * ankle_length;

    // Tính góc Knee (Gối)
    // Tính độ dài từ điểm chân cơ bản đến khớp gối
    r_ac = vector.norm();

    // Tính góc Knee bằng công thức acos
    acos_value = acos((r_ac * r_ac - thigh_length * thigh_length - calf_length * calf_length) / (2 * thigh_length * calf_length));

    // Kiểm tra xem giá trị acos có là NaN không
    if (std::isnan(acos_value) == 1)
      return false;

    // Lưu giá trị góc Knee vào mảng out
    *(out + 3) = acos_value;

    // Tính góc Ankle Roll

    // Tính ma trận nghịch đảo của ma trận transformation_ad
    transformation_da = robotis_framework::getInverseTransformation(transformation_ad);

    // Lấy giá trị tọa độ y và z từ ma trận transformation_da
    double tda_y = transformation_da.coeff(1, 3);
    double tda_z = transformation_da.coeff(2, 3);

    // Tính các giá trị trung gian và kiểm tra giới hạn
    value_k = sqrt(tda_y * tda_y + tda_z * tda_z);
    value_l = sqrt(tda_y * tda_y + (tda_z - ankle_length) * (tda_z - ankle_length));
    value_m = (value_k * value_k - value_l * value_l - ankle_length * ankle_length) / (2 * value_l * ankle_length);

    // Giới hạn giá trị value_m trong khoảng [-1, 1]
    if (value_m > 1.0)
      value_m = 1.0;
    else if (value_m < -1.0)
      value_m = -1.0;

    // Tính góc Ankle Roll bằng công thức acos
    acos_value = acos(value_m);

    // Kiểm tra xem giá trị acos có là NaN không
    if (std::isnan(acos_value) == 1)
      return false;

    // Gán giá trị góc Ankle Roll vào mảng out, với điều chỉnh dựa trên hướng của tda_y
    if (tda_y < 0.0)
      *(out + 5) = -acos_value;
    else
      *(out + 5) = acos_value;

    // Tính góc Hip Yaw

    // Tạo ma trận biến đổi cho phần chân trên cơ thể
    transformation_cd = robotis_framework::getTransformationXYZRPY(0.0, 0.0, -ankle_length, *(out + 5), 0.0, 0.0);

    // Tính ma trận nghịch đảo của ma trận transformation_cd
    transformation_dc = robotis_framework::getInverseTransformation(transformation_cd);

    // Tính ma trận biến đổi cuối cùng cho chân dưới cơ thể
    transformation_ac = transformation_ad * transformation_dc;

    // Tính góc atan2 dựa trên các phần tử của ma trận transformation_ac
    atan_value = atan2(-transformation_ac.coeff(0, 1), transformation_ac.coeff(1, 1));

    // Kiểm tra xem giá trị atan có là vô cùng không
    if (std::isinf(atan_value) == 1)
      return false;

    // Gán giá trị góc Hip Yaw vào mảng out
    *(out) = atan_value;

    // Tính góc Hip Roll
    // Tính góc atan2 dựa trên các phần tử của ma trận transformation_ac
    atan_value = atan2(transformation_ac.coeff(2, 1),
                       -transformation_ac.coeff(0, 1) * sin(*(out)) + transformation_ac.coeff(1, 1) * cos(*(out)));

    // Kiểm tra xem giá trị atan có là vô cùng không
    if (std::isinf(atan_value) == 1)
      return false;

    // Gán giá trị góc Hip Roll vào mảng out
    *(out + 1) = atan_value;

    // Tính góc Hip Pitch và Ankle Pitch

    // Tính góc atan2 dựa trên các phần tử của ma trận transformation_ac
    atan_value = atan2(transformation_ac.coeff(0, 2) * cos(*(out)) + transformation_ac.coeff(1, 2) * sin(*(out)),
                       transformation_ac.coeff(0, 0) * cos(*(out)) + transformation_ac.coeff(1, 0) * sin(*(out)));

    // Kiểm tra xem giá trị atan có là vô cùng không
    if (std::isinf(atan_value) == 1)
      return false;

    // Lưu giá trị góc vào biến theta
    theta = atan_value;

    // Các biến phụ trợ trong quá trình tính toán
    value_k = sin(*(out + 3)) * calf_length;
    value_l = -thigh_length - cos(*(out + 3)) * calf_length;
    value_m = cos(*(out)) * vector.x() + sin(*(out)) * vector.y();
    value_n = cos(*(out + 1)) * vector.z() + sin(*(out)) * sin(*(out + 1)) * vector.x() - cos(*(out)) * sin(*(out + 1)) * vector.y();

    // Tính giá trị value_s và value_c
    value_s = (value_k * value_n + value_l * value_m) / (value_k * value_k + value_l * value_l);
    value_c = (value_n - value_k * value_s) / value_l;

    // Tính góc atan2 dựa trên giá trị value_s và value_c
    atan_value = atan2(value_s, value_c);

    // Kiểm tra xem giá trị atan có là vô cùng không
    if (std::isinf(atan_value) == 1)
      return false;

    // Gán giá trị góc Hip Pitch vào mảng out
    *(out + 2) = atan_value;

    // Tính giá trị góc Ankle Pitch và gán giá trị vào mảng out
    *(out + 4) = theta - *(out + 3) - *(out + 2);

    // Trả về true để báo hiệu rằng tính toán đã thành công
    return true;
  }

  // Cập nhật các tham số thời gian cho chế độ đi bộ

  void WalkingModule::updateTimeParam()
  {
    // Cập nhật thời gian chu kỳ và tỷ lệ DSP (Double Support Phase) và SSP (Single Support Phase)

    // Thời gian chu kỳ chuyển động
    period_time_ = walking_param_.period_time; // * 1000;   // s -> ms

    // Tỷ lệ thời gian hỗ trợ đôi (DSP)
    dsp_ratio_ = walking_param_.dsp_ratio;

    // Tỷ lệ thời gian hỗ trợ đơn (SSP)
    ssp_ratio_ = 1 - dsp_ratio_;

    // Các thông số thời gian cho pha di chuyển theo trục x
    x_swap_period_time_ = period_time_ / 2;          // Thời gian trao đổi chân theo trục x
    x_move_period_time_ = period_time_ * ssp_ratio_; // Thời gian di chuyển theo trục x

    // Các thông số thời gian cho pha di chuyển theo trục y
    y_swap_period_time_ = period_time_;              // Thời gian trao đổi chân theo trục y
    y_move_period_time_ = period_time_ * ssp_ratio_; // Thời gian di chuyển theo trục y

    // Các thông số thời gian cho pha di chuyển theo trục z
    z_swap_period_time_ = period_time_ / 2;              // Thời gian trao đổi chân theo trục z
    z_move_period_time_ = period_time_ * ssp_ratio_ / 2; // Thời gian di chuyển theo trục z

    // Thời gian cho pha di chuyển cơ bản
    a_move_period_time_ = period_time_ * ssp_ratio_; // Thời gian di chuyển hướng

    // Thời gian SSP (Single Support Phase)
    ssp_time_ = period_time_ * ssp_ratio_; // Thời gian pha hỗ trợ đơn (SSP)

    // Thời gian bắt đầu và kết thúc SSP (Single Support Phase) cho chân trái và chân phải

    // Thời gian bắt đầu SSP chân trái
    l_ssp_start_time_ = (1 - ssp_ratio_) * period_time_ / 4;

    // Thời gian kết thúc SSP chân trái
    l_ssp_end_time_ = (1 + ssp_ratio_) * period_time_ / 4;

    // Thời gian bắt đầu SSP chân phải
    r_ssp_start_time_ = (3 - ssp_ratio_) * period_time_ / 4;

    // Thời gian kết thúc SSP chân phải
    r_ssp_end_time_ = (3 + ssp_ratio_) * period_time_ / 4;

    // Thời gian giữa các pha di chuyển
    phase1_time_ = (l_ssp_start_time_ + l_ssp_end_time_) / 2; // Thời gian giữa SSP chân trái và chân phải
    phase2_time_ = (l_ssp_end_time_ + r_ssp_start_time_) / 2; // Thời gian giữa SSP chân phải và chân trái
    phase3_time_ = (r_ssp_start_time_ + r_ssp_end_time_) / 2; // Thời gian giữa SSP chân phải và chân trái

    // Các thông số liên quan đến độ nghiêng của cơ thể
    pelvis_offset_ = walking_param_.pelvis_offset; // Offset của cơ thể
    pelvis_swing_ = pelvis_offset_ * 0.35;         // Biên độ nghiêng của cơ thể

    // Hệ số điều chỉnh sự dao động của cánh tay
    arm_swing_gain_ = walking_param_.arm_swing_gain; // Hệ số điều chỉnh sự dao động của cánh tay
  }

  // Cập nhật các thông số biên độ di chuyển cho chế độ đi bộ

  void WalkingModule::updateMovementParam()
  {
    // Cập nhật biên độ di chuyển theo trục x
    x_move_amplitude_ = walking_param_.x_move_amplitude;
    x_swap_amplitude_ = walking_param_.x_move_amplitude * walking_param_.step_fb_ratio;

    // Giảm độ biên độ khi bắt đầu chuyển động (nếu giá trị trước đó là 0)
    if (previous_x_move_amplitude_ == 0)
    {
      x_move_amplitude_ *= 0.5;
      x_swap_amplitude_ *= 0.5;
    }

    // Cập nhật biên độ di chuyển theo trục y
    y_move_amplitude_ = walking_param_.y_move_amplitude / 2;
    // Dịch chuyển biên độ theo trục y nếu giá trị biên độ là dương
    if (y_move_amplitude_ > 0)
      y_move_amplitude_shift_ = y_move_amplitude_;
    else
      y_move_amplitude_shift_ = -y_move_amplitude_;
    // Cập nhật biên độ trao đổi chân theo trục y
    y_swap_amplitude_ = walking_param_.y_swap_amplitude + y_move_amplitude_shift_ * 0.04;

    // Cập nhật biên độ di chuyển theo trục z
    z_move_amplitude_ = walking_param_.z_move_amplitude / 2;
    z_move_amplitude_shift_ = z_move_amplitude_ / 2;
    // Cập nhật biên độ trao đổi chân theo trục z
    z_swap_amplitude_ = walking_param_.z_swap_amplitude;
    z_swap_amplitude_shift_ = z_swap_amplitude_;

    // Cập nhật biên độ hướng di chuyển (góc xoay)
    if (walking_param_.move_aim_on == false)
    {
      // Nếu không có hướng di chuyển cụ thể, sử dụng nửa biên độ được xác định từ tham số
      a_move_amplitude_ = walking_param_.angle_move_amplitude / 2;
      // Dịch chuyển biên độ theo hướng di chuyển nếu giá trị biên độ là dương
      if (a_move_amplitude_ > 0)
        a_move_amplitude_shift_ = a_move_amplitude_;
      else
        a_move_amplitude_shift_ = -a_move_amplitude_;
    }
    else
    {
      // Ngược lại, nếu có hướng di chuyển cụ thể, sử dụng nửa biên độ đối diện từ tham số
      a_move_amplitude_ = -walking_param_.angle_move_amplitude / 2;
      // Dịch chuyển biên độ theo hướng di chuyển nếu giá trị biên độ là dương
      if (a_move_amplitude_ > 0)
        a_move_amplitude_shift_ = -a_move_amplitude_;
      else
        a_move_amplitude_shift_ = a_move_amplitude_;
    }
  }

  // Cập nhật các giá trị offset cho tọa độ và góc xoay ban đầu của robot

  void WalkingModule::updatePoseParam()
  {
    // Cập nhật offset cho tọa độ x
    x_offset_ = walking_param_.init_x_offset;

    // Cập nhật offset cho tọa độ y
    y_offset_ = walking_param_.init_y_offset;

    // Cập nhật offset cho tọa độ z
    z_offset_ = walking_param_.init_z_offset;

    // Cập nhật offset cho góc Roll
    r_offset_ = walking_param_.init_roll_offset;

    // Cập nhật offset cho góc Pitch
    p_offset_ = walking_param_.init_pitch_offset;

    // Cập nhật offset cho góc Yaw
    a_offset_ = walking_param_.init_yaw_offset;

    // Cập nhật offset cho hip pitch
    hit_pitch_offset_ = walking_param_.hip_pitch_offset;
  }

  // Bắt đầu chế độ đi bộ
  void WalkingModule::startWalking()
  {
    // Đặt trạng thái điều khiển và thực tế là đang chạy
    ctrl_running_ = true;
    real_running_ = true;

    // Xuất thông điệp trạng thái: Bắt đầu đi bộ
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Start walking");
  }

  // Dừng chế độ đi bộ
  void WalkingModule::stop()
  {
    // Đặt trạng thái điều khiển là không chạy
    ctrl_running_ = false;

    // Xuất thông điệp trạng thái: Dừng đi bộ
    publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, "Stop walking");
  }

  // Kiểm tra xem chế độ đi bộ có đang chạy hay không
  bool WalkingModule::isRunning()
  {
    // Trả về true nếu chế độ thực tế đang chạy hoặc trạng thái đi bộ là WalkingInitPose
    return real_running_ || (walking_state_ == WalkingInitPose);
  }

  // default [angle : radian, length : m]
  // Xử lý chuyển động và cân bằng của robot
  void WalkingModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                              std::map<std::string, double> sensors)
  {
    // Kiểm tra xem module điều khiển có được kích hoạt không
    if (enable_ == false)
      return;

    // Đơn vị thời gian sử dụng trong tính toán (chuyển đổi từ ms sang s)
    const double time_unit = control_cycle_msec_ * 0.001; // ms -> s

    // Số lượng cấp độ tự do (joints) trong robot
    int joint_size = result_.size();

    // Khởi tạo mảng lưu giữ giá trị góc của joints và giá trị góc cân bằng
    double angle[joint_size];
    double balance_angle[joint_size];

    // Đặt tất cả giá trị góc và góc cân bằng ban đầu là 0
    for (int _idx = 0; _idx < joint_size; _idx++)
    {
      angle[_idx] = 0.0f;
      balance_angle[_idx] = 0.0f;
    }

    // Xử lý chuyển động đến tư thế khởi tạo nếu trạng thái đi bộ là WalkingInitPose

    // Kiểm tra nếu robot đang trong trạng thái đi bộ là WalkingInitPose
    if (walking_state_ == WalkingInitPose)
    {
      // Tính tổng số bước trong quá trình tạo ra dãy số góc joint
      int total_count = calc_joint_tra_.rows();

      // Lặp qua từng joint để đặt giá trị góc mục tiêu từ dãy số góc đã tính toán
      for (int id = 1; id <= result_.size(); id++)
        target_position_.coeffRef(0, id) = calc_joint_tra_(init_pose_count_, id);

      // Tăng biến đếm cho bước thời gian của tư thế khởi tạo
      init_pose_count_ += 1;

      // Kiểm tra xem đã đến cuối dãy số góc chưa
      if (init_pose_count_ >= total_count)
      {
        // Nếu đã đến cuối, chuyển trạng thái đi bộ về WalkingReady
        walking_state_ = WalkingReady;

        // In thông báo nếu ở chế độ DEBUG
        if (DEBUG)
          std::cout << "End moving to Init : " << init_pose_count_ << std::endl;
      }
    }

    // Xử lý khi robot ở trong trạng thái WalkingReady hoặc WalkingEnable
    else if (walking_state_ == WalkingReady || walking_state_ == WalkingEnable)
    {
      // present angle
      // Lặp qua mỗi joint để cập nhật giá trị góc mục tiêu từ trạng thái hiện tại của động cơ
      for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
           state_iter != result_.end(); state_iter++)
      {
        // Lấy tên joint và chỉ số joint
        std::string _joint_name = state_iter->first;
        int joint_index = joint_table_[_joint_name];

        // Lấy thông tin động cơ từ danh sách động cơ
        robotis_framework::Dynamixel *dxl = NULL;
        std::map<std::string, robotis_framework::Dynamixel *>::iterator dxl_it = dxls.find(_joint_name);
        if (dxl_it != dxls.end())
          dxl = dxl_it->second;
        else
          continue;

        // Cập nhật giá trị góc mục tiêu cho joint từ trạng thái hiện tại của động cơ
        goal_position_.coeffRef(0, joint_index) = dxl->dxl_state_->goal_position_;
      }

      // Xử lý các pha chuyển động và tính toán các góc của chân và cánh tay

      // Xử lý các pha chuyển động (gọi hàm processPhase)
      processPhase(time_unit);

      // Biến để kiểm tra việc tính toán góc của chân
      bool get_angle = false;

      // Tính toán góc của chân (gọi hàm computeLegAngle) và kiểm tra kết quả
      get_angle = computeLegAngle(&angle[0]);

      // Tính toán góc của cánh tay (gọi hàm computeArmAngle)
      computeArmAngle(&angle[12]);

      // Tính toán sai lệch giữa giá trị mong muốn và giá trị thực tế của cảm biến gyroscope
      double rl_gyro_err = 0.0 - sensors["gyro_x"];
      double fb_gyro_err = 0.0 - sensors["gyro_y"];

      // Cập nhật phản hồi từ cảm biến gyroscope (gọi hàm sensoryFeedback)
      sensoryFeedback(rl_gyro_err, fb_gyro_err, balance_angle);

      // Tính toán sai lệch tổng và sai lệch lớn nhất giữa giá trị mong muốn và giá trị thực tế của các joint
      double err_total = 0.0, err_max = 0.0;

      // Đặt giá trị mong muốn của joint (goal position)
      for (int idx = 0; idx < 14; idx++)
      {
        double goal_position = 0.0;

        // Nếu không tính được góc chân và index < 12, sử dụng giá trị mong muốn hiện tại
        if (get_angle == false && idx < 12)
          goal_position = goal_position_.coeff(0, idx);
        else
          // Ngược lại, tính toán giá trị mong muốn bằng cách cộng góc chân và cân bằng
          goal_position = init_position_.coeff(0, idx) + angle[idx] + balance_angle[idx];

        // Đặt giá trị mong muốn cho joint
        target_position_.coeffRef(0, idx) = goal_position;

        // Tính toán và cập nhật sai lệch tương ứng
        double err = fabs(target_position_.coeff(0, idx) - goal_position_.coeff(0, idx)) * RADIAN2DEGREE;
        if (err > err_max)
          err_max = err;
        err_total += err;
      }

      // Check Enable
      // Kiểm tra điều kiện enable và sai lệch tổng lớn hơn ngưỡng (5 độ)
      if (walking_state_ == WalkingEnable && err_total > 5.0)
      {
        // Xuất thông báo debug nếu được kích hoạt
        if (DEBUG)
          std::cout << "Check Err : " << err_max << std::endl;

        // Tạo quỹ đạo cho tư thế khởi tạo với thời gian chuyển động tính toán từ sai lệch lớn nhất
        int mov_time = err_max / 30;
        iniPoseTraGene(mov_time < 1 ? 1 : mov_time);

        // Đặt giá trị mong muốn của các joint bằng giá trị mong muốn cuối cùng
        target_position_ = goal_position_;

        // Chuyển trạng thái sang WalkingInitPose
        walking_state_ = WalkingInitPose;

        // Xuất thông báo debug cho các thông số của hệ thống
        ROS_WARN_STREAM_COND(DEBUG, "x_offset: " << walking_param_.init_x_offset);
        ROS_WARN_STREAM_COND(DEBUG, "y_offset: " << walking_param_.init_y_offset);
        ROS_WARN_STREAM_COND(DEBUG, "z_offset: " << walking_param_.init_z_offset);
        ROS_WARN_STREAM_COND(DEBUG, "roll_offset: " << walking_param_.init_roll_offset * RADIAN2DEGREE);
        ROS_WARN_STREAM_COND(DEBUG, "pitch_offset: " << walking_param_.init_pitch_offset * RADIAN2DEGREE);
        ROS_WARN_STREAM_COND(DEBUG, "yaw_offset: " << walking_param_.init_yaw_offset * RADIAN2DEGREE);
        ROS_WARN_STREAM_COND(DEBUG, "hip_pitch_offset: " << walking_param_.hip_pitch_offset * RADIAN2DEGREE);
        ROS_WARN_STREAM_COND(DEBUG, "period_time: " << walking_param_.period_time * 1000);
        ROS_WARN_STREAM_COND(DEBUG, "dsp_ratio: " << walking_param_.dsp_ratio);
        ROS_WARN_STREAM_COND(DEBUG, "step_forward_back_ratio: " << walking_param_.step_fb_ratio);
        ROS_WARN_STREAM_COND(DEBUG, "foot_height: " << walking_param_.z_move_amplitude);
        ROS_WARN_STREAM_COND(DEBUG, "swing_right_left: " << walking_param_.y_swap_amplitude);
        ROS_WARN_STREAM_COND(DEBUG, "swing_top_down: " << walking_param_.z_swap_amplitude);
        ROS_WARN_STREAM_COND(DEBUG, "pelvis_offset: " << walking_param_.pelvis_offset * RADIAN2DEGREE);
        ROS_WARN_STREAM_COND(DEBUG, "arm_swing_gain: " << walking_param_.arm_swing_gain);
        ROS_WARN_STREAM_COND(DEBUG, "balance_hip_roll_gain: " << walking_param_.balance_hip_roll_gain);
        ROS_WARN_STREAM_COND(DEBUG, "balance_knee_gain: " << walking_param_.balance_knee_gain);
        ROS_WARN_STREAM_COND(DEBUG, "balance_ankle_roll_gain: " << walking_param_.balance_ankle_roll_gain);
        ROS_WARN_STREAM_COND(DEBUG, "balance_ankle_pitch_gain: " << walking_param_.balance_ankle_pitch_gain);
        ROS_WARN_STREAM_COND(DEBUG, "balance : " << (walking_param_.balance_enable ? "TRUE" : "FALSE"));
      }
      else
      {
        // Nếu không đạt điều kiện, chuyển sang trạng thái WalkingReady
        walking_state_ = WalkingReady;
      }
    }

    // set result
// Thiết lập giá trị mong muốn cho các joint và có thể thêm cài đặt PID ở đây nếu cần
for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_it = result_.begin();
     state_it != result_.end(); state_it++)
{
  std::string joint_name = state_it->first;
  int joint_index = joint_table_[joint_name];

  // Thiết lập giá trị mong muốn cho joint
  result_[joint_name]->goal_position_ = target_position_.coeff(0, joint_index);

  // Todo: Cài đặt các thông số PID cho các joint của chân nếu cần
  // result_[joint_name]->position_p_gain_ = walking_param_.p_gain;
  // result_[joint_name]->position_i_gain_ = walking_param_.i_gain;
  // result_[joint_name]->position_d_gain_ = walking_param_.d_gain;
}

// Cập nhật thời gian
if (real_running_ == true)
{
  time_ += time_unit;
  if (time_ >= period_time_)
  {
    time_ = 0;
    // Đặt giá trị biên độ di chuyển x trước đó
    previous_x_move_amplitude_ = walking_param_.x_move_amplitude * 0.5;
  }
}

  }





  void WalkingModule::processPhase(const double &time_unit)
  {
    // Update walk parameters
    if (time_ == 0)
    {
      updateTimeParam();
      phase_ = PHASE0;
      if (ctrl_running_ == false)
      {
        if (x_move_amplitude_ == 0 && y_move_amplitude_ == 0 && a_move_amplitude_ == 0)
        {
          real_running_ = false;
        }
        else
        {
          // set walking param to init
          walking_param_.x_move_amplitude = 0;
          walking_param_.y_move_amplitude = 0;
          walking_param_.angle_move_amplitude = 0;

          previous_x_move_amplitude_ = 0;
        }
      }
    }



    
    else if (time_ >= (phase1_time_ - time_unit / 2) && time_ < (phase1_time_ + time_unit / 2)) // the position of left foot is the highest.
    {
      updateMovementParam();
      phase_ = PHASE1;
    }
    else if (time_ >= (phase2_time_ - time_unit / 2) && time_ < (phase2_time_ + time_unit / 2)) // middle of double support state
    {
      updateTimeParam();

      time_ = phase2_time_;
      phase_ = PHASE2;
      if (ctrl_running_ == false)
      {
        if (x_move_amplitude_ == 0 && y_move_amplitude_ == 0 && a_move_amplitude_ == 0)
        {
          real_running_ = false;
        }
        else
        {
          // set walking param to init
          walking_param_.x_move_amplitude = previous_x_move_amplitude_;
          walking_param_.y_move_amplitude = 0;
          walking_param_.angle_move_amplitude = 0;
        }
      }
    }
    else if (time_ >= (phase3_time_ - time_unit / 2) && time_ < (phase3_time_ + time_unit / 2)) // the position of right foot is the highest.
    {
      updateMovementParam();
      phase_ = PHASE3;
    }
  }

  bool WalkingModule::computeLegAngle(double *leg_angle)
  {
    Pose3D swap, right_leg_move, left_leg_move;
    double pelvis_offset_r, pelvis_offset_l;
    double ep[12];

    updatePoseParam();

    // Compute endpoints
    swap.x = wSin(time_, x_swap_period_time_, x_swap_phase_shift_, x_swap_amplitude_, x_swap_amplitude_shift_);
    swap.y = wSin(time_, y_swap_period_time_, y_swap_phase_shift_, y_swap_amplitude_, y_swap_amplitude_shift_);
    swap.z = wSin(time_, z_swap_period_time_, z_swap_phase_shift_, z_swap_amplitude_, z_swap_amplitude_shift_);
    swap.roll = 0.0;
    swap.pitch = 0.0;
    swap.yaw = 0.0;

    if (time_ <= l_ssp_start_time_)
    {
      left_leg_move.x = wSin(l_ssp_start_time_, x_move_period_time_,
                             x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * l_ssp_start_time_, x_move_amplitude_,
                             x_move_amplitude_shift_);
      left_leg_move.y = wSin(l_ssp_start_time_, y_move_period_time_,
                             y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * l_ssp_start_time_, y_move_amplitude_,
                             y_move_amplitude_shift_);
      left_leg_move.z = wSin(l_ssp_start_time_, z_move_period_time_,
                             z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * l_ssp_start_time_, z_move_amplitude_,
                             z_move_amplitude_shift_);
      left_leg_move.yaw = wSin(l_ssp_start_time_, a_move_period_time_,
                               a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * l_ssp_start_time_,
                               a_move_amplitude_, a_move_amplitude_shift_);
      right_leg_move.x = wSin(l_ssp_start_time_, x_move_period_time_,
                              x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * l_ssp_start_time_,
                              -x_move_amplitude_, -x_move_amplitude_shift_);
      right_leg_move.y = wSin(l_ssp_start_time_, y_move_period_time_,
                              y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * l_ssp_start_time_,
                              -y_move_amplitude_, -y_move_amplitude_shift_);
      right_leg_move.z = wSin(r_ssp_start_time_, z_move_period_time_,
                              z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * r_ssp_start_time_, z_move_amplitude_,
                              z_move_amplitude_shift_);
      right_leg_move.yaw = wSin(l_ssp_start_time_, a_move_period_time_,
                                a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * l_ssp_start_time_,
                                -a_move_amplitude_, -a_move_amplitude_shift_);
      pelvis_offset_l = 0;
      pelvis_offset_r = 0;
    }
    else if (time_ <= l_ssp_end_time_)
    {
      left_leg_move.x = wSin(time_, x_move_period_time_,
                             x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * l_ssp_start_time_, x_move_amplitude_,
                             x_move_amplitude_shift_);
      left_leg_move.y = wSin(time_, y_move_period_time_,
                             y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * l_ssp_start_time_, y_move_amplitude_,
                             y_move_amplitude_shift_);
      left_leg_move.z = wSin(time_, z_move_period_time_,
                             z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * l_ssp_start_time_, z_move_amplitude_,
                             z_move_amplitude_shift_);
      left_leg_move.yaw = wSin(time_, a_move_period_time_,
                               a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * l_ssp_start_time_,
                               a_move_amplitude_, a_move_amplitude_shift_);
      right_leg_move.x = wSin(time_, x_move_period_time_,
                              x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * l_ssp_start_time_,
                              -x_move_amplitude_, -x_move_amplitude_shift_);
      right_leg_move.y = wSin(time_, y_move_period_time_,
                              y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * l_ssp_start_time_,
                              -y_move_amplitude_, -y_move_amplitude_shift_);
      right_leg_move.z = wSin(r_ssp_start_time_, z_move_period_time_,
                              z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * r_ssp_start_time_, z_move_amplitude_,
                              z_move_amplitude_shift_);
      right_leg_move.yaw = wSin(time_, a_move_period_time_,
                                a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * l_ssp_start_time_,
                                -a_move_amplitude_, -a_move_amplitude_shift_);
      pelvis_offset_l = wSin(time_, z_move_period_time_,
                             z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * l_ssp_start_time_, pelvis_swing_ / 2,
                             pelvis_swing_ / 2);
      pelvis_offset_r = wSin(time_, z_move_period_time_,
                             z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * l_ssp_start_time_,
                             -pelvis_offset_ / 2, -pelvis_offset_ / 2);
    }
    else if (time_ <= r_ssp_start_time_)
    {
      left_leg_move.x = wSin(l_ssp_end_time_, x_move_period_time_,
                             x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * l_ssp_start_time_, x_move_amplitude_,
                             x_move_amplitude_shift_);
      left_leg_move.y = wSin(l_ssp_end_time_, y_move_period_time_,
                             y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * l_ssp_start_time_, y_move_amplitude_,
                             y_move_amplitude_shift_);
      left_leg_move.z = wSin(l_ssp_end_time_, z_move_period_time_,
                             z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * l_ssp_start_time_, z_move_amplitude_,
                             z_move_amplitude_shift_);
      left_leg_move.yaw = wSin(l_ssp_end_time_, a_move_period_time_,
                               a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * l_ssp_start_time_,
                               a_move_amplitude_, a_move_amplitude_shift_);
      right_leg_move.x = wSin(l_ssp_end_time_, x_move_period_time_,
                              x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * l_ssp_start_time_,
                              -x_move_amplitude_, -x_move_amplitude_shift_);
      right_leg_move.y = wSin(l_ssp_end_time_, y_move_period_time_,
                              y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * l_ssp_start_time_,
                              -y_move_amplitude_, -y_move_amplitude_shift_);
      right_leg_move.z = wSin(r_ssp_start_time_, z_move_period_time_,
                              z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * r_ssp_start_time_, z_move_amplitude_,
                              z_move_amplitude_shift_);
      right_leg_move.yaw = wSin(l_ssp_end_time_, a_move_period_time_,
                                a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * l_ssp_start_time_,
                                -a_move_amplitude_, -a_move_amplitude_shift_);
      pelvis_offset_l = 0;
      pelvis_offset_r = 0;
    }
    else if (time_ <= r_ssp_end_time_)
    {
      left_leg_move.x = wSin(time_, x_move_period_time_,
                             x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * r_ssp_start_time_ + M_PI,
                             x_move_amplitude_, x_move_amplitude_shift_);
      left_leg_move.y = wSin(time_, y_move_period_time_,
                             y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * r_ssp_start_time_ + M_PI,
                             y_move_amplitude_, y_move_amplitude_shift_);
      left_leg_move.z = wSin(l_ssp_end_time_, z_move_period_time_,
                             z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * l_ssp_start_time_, z_move_amplitude_,
                             z_move_amplitude_shift_);
      left_leg_move.yaw = wSin(time_, a_move_period_time_,
                               a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * r_ssp_start_time_ + M_PI,
                               a_move_amplitude_, a_move_amplitude_shift_);
      right_leg_move.x = wSin(time_, x_move_period_time_,
                              x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * r_ssp_start_time_ + M_PI,
                              -x_move_amplitude_, -x_move_amplitude_shift_);
      right_leg_move.y = wSin(time_, y_move_period_time_,
                              y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * r_ssp_start_time_ + M_PI,
                              -y_move_amplitude_, -y_move_amplitude_shift_);
      right_leg_move.z = wSin(time_, z_move_period_time_,
                              z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * r_ssp_start_time_, z_move_amplitude_,
                              z_move_amplitude_shift_);
      right_leg_move.yaw = wSin(time_, a_move_period_time_,
                                a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * r_ssp_start_time_ + M_PI,
                                -a_move_amplitude_, -a_move_amplitude_shift_);
      pelvis_offset_l = wSin(time_, z_move_period_time_,
                             z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * r_ssp_start_time_, pelvis_offset_ / 2,
                             pelvis_offset_ / 2);
      pelvis_offset_r = wSin(time_, z_move_period_time_,
                             z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * r_ssp_start_time_, -pelvis_swing_ / 2,
                             -pelvis_swing_ / 2);
    }
    else
    {
      left_leg_move.x = wSin(r_ssp_end_time_, x_move_period_time_,
                             x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * r_ssp_start_time_ + M_PI,
                             x_move_amplitude_, x_move_amplitude_shift_);
      left_leg_move.y = wSin(r_ssp_end_time_, y_move_period_time_,
                             y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * r_ssp_start_time_ + M_PI,
                             y_move_amplitude_, y_move_amplitude_shift_);
      left_leg_move.z = wSin(l_ssp_end_time_, z_move_period_time_,
                             z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * l_ssp_start_time_, z_move_amplitude_,
                             z_move_amplitude_shift_);
      left_leg_move.yaw = wSin(r_ssp_end_time_, a_move_period_time_,
                               a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * r_ssp_start_time_ + M_PI,
                               a_move_amplitude_, a_move_amplitude_shift_);
      right_leg_move.x = wSin(r_ssp_end_time_, x_move_period_time_,
                              x_move_phase_shift_ + 2 * M_PI / x_move_period_time_ * r_ssp_start_time_ + M_PI,
                              -x_move_amplitude_, -x_move_amplitude_shift_);
      right_leg_move.y = wSin(r_ssp_end_time_, y_move_period_time_,
                              y_move_phase_shift_ + 2 * M_PI / y_move_period_time_ * r_ssp_start_time_ + M_PI,
                              -y_move_amplitude_, -y_move_amplitude_shift_);
      right_leg_move.z = wSin(r_ssp_end_time_, z_move_period_time_,
                              z_move_phase_shift_ + 2 * M_PI / z_move_period_time_ * r_ssp_start_time_, z_move_amplitude_,
                              z_move_amplitude_shift_);
      right_leg_move.yaw = wSin(r_ssp_end_time_, a_move_period_time_,
                                a_move_phase_shift_ + 2 * M_PI / a_move_period_time_ * r_ssp_start_time_ + M_PI,
                                -a_move_amplitude_, -a_move_amplitude_shift_);
      pelvis_offset_l = 0;
      pelvis_offset_r = 0;
    }

    left_leg_move.roll = 0;
    left_leg_move.pitch = 0;
    right_leg_move.roll = 0;
    right_leg_move.pitch = 0;

    double leg_length = op3_kd_->thigh_length_m_ + op3_kd_->calf_length_m_ + op3_kd_->ankle_length_m_;

    // mm, rad
    ep[0] = swap.x + right_leg_move.x + x_offset_;
    ep[1] = swap.y + right_leg_move.y - y_offset_ / 2;
    ep[2] = swap.z + right_leg_move.z + z_offset_ - leg_length;
    ep[3] = swap.roll + right_leg_move.roll - r_offset_ / 2;
    ep[4] = swap.pitch + right_leg_move.pitch + p_offset_;
    ep[5] = swap.yaw + right_leg_move.yaw - a_offset_ / 2;
    ep[6] = swap.x + left_leg_move.x + x_offset_;
    ep[7] = swap.y + left_leg_move.y + y_offset_ / 2;
    ep[8] = swap.z + left_leg_move.z + z_offset_ - leg_length;
    ep[9] = swap.roll + left_leg_move.roll + r_offset_ / 2;
    ep[10] = swap.pitch + left_leg_move.pitch + p_offset_;
    ep[11] = swap.yaw + left_leg_move.yaw + a_offset_ / 2;

    // Compute body swing
    if (time_ <= l_ssp_end_time_)
    {
      body_swing_y = -ep[7];
      body_swing_z = ep[8];
    }
    else
    {
      body_swing_y = -ep[1];
      body_swing_z = ep[2];
    }
    body_swing_z -= leg_length;

    // right leg
    if (op3_kd_->calcInverseKinematicsForRightLeg(&leg_angle[0], ep[0], ep[1], ep[2], ep[3], ep[4], ep[5]) == false)
    {
      printf("IK not Solved EPR : %f %f %f %f %f %f\n", ep[0], ep[1], ep[2], ep[3], ep[4], ep[5]);
      return false;
    }

    if (op3_kd_->calcInverseKinematicsForLeftLeg(&leg_angle[6], ep[6], ep[7], ep[8], ep[9], ep[10], ep[11]) == false)
    {
      printf("IK not Solved EPL : %f %f %f %f %f %f\n", ep[6], ep[7], ep[8], ep[9], ep[10], ep[11]);
      return false;
    }

    // Compute dxls angle
    for (int i = 0; i < 12; i++)
    {
      // offset : rad
      double offset = 0;

      if (i == joint_table_["r_hip_roll"]) // R_HIP_ROLL
        offset += op3_kd_->getJointDirection("r_hip_roll") * pelvis_offset_r;
      else if (i == joint_table_["l_hip_roll"]) // L_HIP_ROLL
        offset += op3_kd_->getJointDirection("l_hip_roll") * pelvis_offset_l;
      else if (i == joint_table_["r_hip_pitch"])
        offset -= op3_kd_->getJointDirection("r_hip_pitch") * hit_pitch_offset_;
      else if (i == joint_table_["l_hip_pitch"]) // R_HIP_PITCH or L_HIP_PITCH
        offset -= op3_kd_->getJointDirection("l_hip_pitch") * hit_pitch_offset_;

      leg_angle[i] += offset;
    }

    return true;
  }

  void WalkingModule::computeArmAngle(double *arm_angle)
  {
    // Compute arm swing
    if (x_move_amplitude_ == 0)
    {
      arm_angle[0] = 0; // Right
      arm_angle[1] = 0; // Left
    }
    else
    {
      arm_angle[0] = wSin(time_, period_time_, M_PI * 1.5, -x_move_amplitude_ * arm_swing_gain_ * 1000,
                          0) *
                     op3_kd_->getJointDirection("r_sho_pitch") * DEGREE2RADIAN;
      arm_angle[1] = wSin(time_, period_time_, M_PI * 1.5, x_move_amplitude_ * arm_swing_gain_ * 1000,
                          0) *
                     op3_kd_->getJointDirection("l_sho_pitch") * DEGREE2RADIAN;
    }
  }

  void WalkingModule::sensoryFeedback(const double &rlGyroErr, const double &fbGyroErr, double *balance_angle)
  {
    // adjust balance offset
    if (walking_param_.balance_enable == false)
      return;

    double internal_gain = 0.05;

    balance_angle[joint_table_["r_hip_roll"]] = op3_kd_->getJointDirection("r_hip_roll") * internal_gain * rlGyroErr * walking_param_.balance_hip_roll_gain; // R_HIP_ROLL
    balance_angle[joint_table_["l_hip_roll"]] = op3_kd_->getJointDirection("l_hip_roll") * internal_gain * rlGyroErr * walking_param_.balance_hip_roll_gain; // L_HIP_ROLL

    balance_angle[joint_table_["r_knee"]] = -op3_kd_->getJointDirection("r_knee") * internal_gain * fbGyroErr * walking_param_.balance_knee_gain; // R_KNEE
    balance_angle[joint_table_["l_knee"]] = -op3_kd_->getJointDirection("l_knee") * internal_gain * fbGyroErr * walking_param_.balance_knee_gain; // L_KNEE

    balance_angle[joint_table_["r_ank_pitch"]] = -op3_kd_->getJointDirection("r_ank_pitch") * internal_gain * fbGyroErr * walking_param_.balance_ankle_pitch_gain; // R_ANKLE_PITCH
    balance_angle[joint_table_["l_ank_pitch"]] = -op3_kd_->getJointDirection("l_ank_pitch") * internal_gain * fbGyroErr * walking_param_.balance_ankle_pitch_gain; // L_ANKLE_PITCH

    balance_angle[joint_table_["r_ank_roll"]] = -op3_kd_->getJointDirection("r_ank_roll") * internal_gain * rlGyroErr * walking_param_.balance_ankle_roll_gain; // R_ANKLE_ROLL
    balance_angle[joint_table_["l_ank_roll"]] = -op3_kd_->getJointDirection("l_ank_roll") * internal_gain * rlGyroErr * walking_param_.balance_ankle_roll_gain; // L_ANKLE_ROLL
  }

  void WalkingModule::loadWalkingParam(const std::string &path)
  {
    YAML::Node doc;
    try
    {
      // load yaml
      doc = YAML::LoadFile(path.c_str());
    }
    catch (const std::exception &e)
    {
      ROS_ERROR("Fail to load yaml file.");
      return;
    }

    // parse movement time
    walking_param_.init_x_offset = doc["x_offset"].as<double>();
    walking_param_.init_y_offset = doc["y_offset"].as<double>();
    walking_param_.init_z_offset = doc["z_offset"].as<double>();
    walking_param_.init_roll_offset = doc["roll_offset"].as<double>() * DEGREE2RADIAN;
    walking_param_.init_pitch_offset = doc["pitch_offset"].as<double>() * DEGREE2RADIAN;
    walking_param_.init_yaw_offset = doc["yaw_offset"].as<double>() * DEGREE2RADIAN;
    walking_param_.hip_pitch_offset = doc["hip_pitch_offset"].as<double>() * DEGREE2RADIAN;
    // time
    walking_param_.period_time = doc["period_time"].as<double>() * 0.001; // ms -> s
    walking_param_.dsp_ratio = doc["dsp_ratio"].as<double>();
    walking_param_.step_fb_ratio = doc["step_forward_back_ratio"].as<double>();
    // walking
    // walking_param_.x_move_amplitude
    // walking_param_.y_move_amplitude
    walking_param_.z_move_amplitude = doc["foot_height"].as<double>();
    // walking_param_.angle_move_amplitude
    // walking_param_.move_aim_on

    // balance
    // walking_param_.balance_enable
    walking_param_.balance_hip_roll_gain = doc["balance_hip_roll_gain"].as<double>();
    walking_param_.balance_knee_gain = doc["balance_knee_gain"].as<double>();
    walking_param_.balance_ankle_roll_gain = doc["balance_ankle_roll_gain"].as<double>();
    walking_param_.balance_ankle_pitch_gain = doc["balance_ankle_pitch_gain"].as<double>();
    walking_param_.y_swap_amplitude = doc["swing_right_left"].as<double>();
    walking_param_.z_swap_amplitude = doc["swing_top_down"].as<double>();
    walking_param_.pelvis_offset = doc["pelvis_offset"].as<double>() * DEGREE2RADIAN;
    walking_param_.arm_swing_gain = doc["arm_swing_gain"].as<double>();

    // gain
    walking_param_.p_gain = doc["p_gain"].as<int>();
    walking_param_.i_gain = doc["i_gain"].as<int>();
    walking_param_.d_gain = doc["d_gain"].as<int>();
  }

  void WalkingModule::saveWalkingParam(std::string &path)
  {
    YAML::Emitter out_emitter;

    out_emitter << YAML::BeginMap;
    out_emitter << YAML::Key << "x_offset" << YAML::Value << walking_param_.init_x_offset;
    out_emitter << YAML::Key << "y_offset" << YAML::Value << walking_param_.init_y_offset;
    out_emitter << YAML::Key << "z_offset" << YAML::Value << walking_param_.init_z_offset;
    out_emitter << YAML::Key << "roll_offset" << YAML::Value << walking_param_.init_roll_offset * RADIAN2DEGREE;
    out_emitter << YAML::Key << "pitch_offset" << YAML::Value << walking_param_.init_pitch_offset * RADIAN2DEGREE;
    out_emitter << YAML::Key << "yaw_offset" << YAML::Value << walking_param_.init_yaw_offset * RADIAN2DEGREE;
    out_emitter << YAML::Key << "hip_pitch_offset" << YAML::Value << walking_param_.hip_pitch_offset * RADIAN2DEGREE;
    out_emitter << YAML::Key << "period_time" << YAML::Value << walking_param_.period_time * 1000;
    out_emitter << YAML::Key << "dsp_ratio" << YAML::Value << walking_param_.dsp_ratio;
    out_emitter << YAML::Key << "step_forward_back_ratio" << YAML::Value << walking_param_.step_fb_ratio;
    out_emitter << YAML::Key << "foot_height" << YAML::Value << walking_param_.z_move_amplitude;
    out_emitter << YAML::Key << "swing_right_left" << YAML::Value << walking_param_.y_swap_amplitude;
    out_emitter << YAML::Key << "swing_top_down" << YAML::Value << walking_param_.z_swap_amplitude;
    out_emitter << YAML::Key << "pelvis_offset" << YAML::Value << walking_param_.pelvis_offset * RADIAN2DEGREE;
    out_emitter << YAML::Key << "arm_swing_gain" << YAML::Value << walking_param_.arm_swing_gain;
    out_emitter << YAML::Key << "balance_hip_roll_gain" << YAML::Value << walking_param_.balance_hip_roll_gain;
    out_emitter << YAML::Key << "balance_knee_gain" << YAML::Value << walking_param_.balance_knee_gain;
    out_emitter << YAML::Key << "balance_ankle_roll_gain" << YAML::Value << walking_param_.balance_ankle_roll_gain;
    out_emitter << YAML::Key << "balance_ankle_pitch_gain" << YAML::Value << walking_param_.balance_ankle_pitch_gain;

    out_emitter << YAML::Key << "p_gain" << YAML::Value << walking_param_.p_gain;
    out_emitter << YAML::Key << "i_gain" << YAML::Value << walking_param_.i_gain;
    out_emitter << YAML::Key << "d_gain" << YAML::Value << walking_param_.d_gain;
    out_emitter << YAML::EndMap;

    // output to file
    std::ofstream fout(path.c_str());
    fout << out_emitter.c_str();
  }

  void WalkingModule::onModuleEnable()
  {
    walking_state_ = WalkingEnable;
    ROS_INFO("Walking Enable");
  }

  void WalkingModule::onModuleDisable()
  {
    ROS_INFO("Walking Disable");
    walking_state_ = WalkingDisable;
  }

  void WalkingModule::iniPoseTraGene(double mov_time)
  {
    double smp_time = control_cycle_msec_ * 0.001;
    int all_time_steps = int(mov_time / smp_time) + 1;
    calc_joint_tra_.resize(all_time_steps, result_.size() + 1);

    for (int id = 0; id <= result_.size(); id++)
    {
      double ini_value = goal_position_.coeff(0, id);
      double tar_value = target_position_.coeff(0, id);

      Eigen::MatrixXd tra;

      tra = robotis_framework::calcMinimumJerkTra(ini_value, 0.0, 0.0, tar_value, 0.0, 0.0, smp_time, mov_time);

      calc_joint_tra_.block(0, id, all_time_steps, 1) = tra;
    }

    if (DEBUG)
      std::cout << "Generate Trajecotry : " << mov_time << "s [" << all_time_steps << "]" << std::endl;

    init_pose_count_ = 0;
  }
}

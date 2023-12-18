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

#ifndef HEAD_CONTROL_MODULE_H_
#define HEAD_CONTROL_MODULE_H_

#include <cstdlib>
#include <ctime>
#include <boost/thread.hpp>
#include <eigen3/Eigen/Eigen>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>

#include "robotis_controller_msgs/StatusMsg.h"
#include "robotis_framework_common/motion_module.h"
#include "robotis_math/robotis_math.h"

namespace robotis_op
{

  // Lớp quản lý điều khiển đầu robot
  class HeadControlModule : public robotis_framework::MotionModule, public robotis_framework::Singleton<HeadControlModule>
  {
  public:
    // Hàm khởi tạo
    HeadControlModule();
    // Hàm hủy
    virtual ~HeadControlModule();

    // Hàm khởi tạo module
    void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
    // Hàm xử lý chính của module
    void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);

    // Hàm dừng module
    void stop();
    // Kiểm tra xem module có đang chạy hay không
    bool isRunning();

    // Hàm được gọi khi module được kích hoạt
    void onModuleEnable();
    // Hàm được gọi khi module bị vô hiệu hóa
    void onModuleDisable();

  private:
    // Các trạng thái của quá trình quét đầu
    enum
    {
      NoScan = 0,
      TopLeft = 1,
      BottomRight = 2,
      BottomLeft = 3,
      TopRight = 4,
    };

    /* Các hàm xử lý sự kiện từ các topic ROS */

    // Callback khi có tin nhắn điều chỉnh vị trí đầu
    void setHeadJointCallback(const sensor_msgs::JointState::ConstPtr &msg);
    // Callback khi có tin nhắn điều chỉnh vị trí đầu tương đối
    void setHeadJointOffsetCallback(const sensor_msgs::JointState::ConstPtr &msg);
    // Callback khi có yêu cầu quét đầu
    void setHeadScanCallback(const std_msgs::String::ConstPtr &msg);

    // Hàm xử lý hàng đợi ROS
    void queueThread();
    // Hàm tạo luồng quét đầu
    void jointTraGeneThread();
    // Hàm thiết lập vị trí đầu
    void setHeadJoint(const sensor_msgs::JointState::ConstPtr &msg, bool is_offset);
    // Hàm kiểm tra giới hạn góc của khớp
    bool checkAngleLimit(const int joint_index, double &goal_position);
    // Hàm tạo quỹ đạo quét đầu
    void generateScanTra(const int head_direction);

    // Các hàm quản lý trạng thái chuyển động của đầu
    void startMoving();
    void finishMoving();
    void stopMoving();

    // Hàm xuất thông báo trạng thái của module
    void publishStatusMsg(unsigned int type, std::string msg);

    // Hàm tính toán quỹ đạo với tốc độ và gia tốc tối thiểu (Minimum Jerk)
    Eigen::MatrixXd calcMinimumJerkTraPVA(double pos_start, double vel_start, double accel_start, double pos_end,
                                          double vel_end, double accel_end, double smp_time, double mov_time);

    // Các biến thành viên

    // Chu kỳ kiểm soát (control_cycle_msec_)
    int control_cycle_msec_;

    // Luồng xử lý hàng đợi (queue_thread_)
    boost::thread queue_thread_;

    // Luồng xử lý tạo gen (tra_gene_thread_)
    boost::thread *tra_gene_thread_;

    // Khóa mutex cho quá trình tạo gen (tra_lock_)
    boost::mutex tra_lock_;

    // Đối tượng Publisher để xuất bản thông điệp trạng thái (status_msg_pub_)
    ros::Publisher status_msg_pub_;

    // Chế độ gỡ lỗi (DEBUG)
    const bool DEBUG;

    // Cờ dừng quá trình (stop_process_)
    bool stop_process_;

    // Trạng thái di chuyển (is_moving_)
    bool is_moving_;

    // Chế độ kiểm soát trực tiếp (is_direct_control_)
    bool is_direct_control_;

    // Số lượng và kích thước của quỹ đạo (tra_count_, tra_size_)
    int tra_count_, tra_size_;

    // Thời gian di chuyển (moving_time_)
    double moving_time_;

    // Trạng thái của quét (scan_state_)
    int scan_state_;

    // Kiểm tra xem có vị trí mục tiêu hay không (has_goal_position_)
    bool has_goal_position_;

    // Đơn vị góc (angle_unit_)
    double angle_unit_;

    // Các ma trận vị trí, vận tốc, gia tốc, quỹ đạo của khớp đầu

    // Ma trận vị trí mục tiêu (target_position_)
    Eigen::MatrixXd target_position_;

    // Ma trận vị trí hiện tại (current_position_)
    Eigen::MatrixXd current_position_;

    // Ma trận vị trí mục tiêu (goal_position_)
    Eigen::MatrixXd goal_position_;

    // Ma trận vận tốc mục tiêu (goal_velocity_)
    Eigen::MatrixXd goal_velocity_;

    // Ma trận gia tốc mục tiêu (goal_acceleration_)
    Eigen::MatrixXd goal_acceleration_;

    // Ma trận quỹ đạo của khớp đầu (calc_joint_tra_)
    Eigen::MatrixXd calc_joint_tra_;

    // Ma trận vận tốc quỹ đạo của khớp đầu (calc_joint_vel_tra_)
    Eigen::MatrixXd calc_joint_vel_tra_;

    // Ma trận gia tốc quỹ đạo của khớp đầu (calc_joint_accel_tra_)
    Eigen::MatrixXd calc_joint_accel_tra_;

    // Danh sách tên khớp được sử dụng
    std::map<std::string, int> using_joint_name_;
    
    // Giới hạn góc của từng khớp
    std::map<int, double> max_angle_;
    std::map<int, double> min_angle_;

    // Thời điểm nhận được tin nhắn cuối cùng
    ros::Time last_msg_time_;
    // Tin nhắn cuối cùng nhận được
    std::string last_msg_;
  };

}

#endif /* HEAD_CONTROL_MODULE_H_ */

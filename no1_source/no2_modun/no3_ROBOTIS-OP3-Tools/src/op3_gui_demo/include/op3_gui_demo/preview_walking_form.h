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

/* Author: Kayman Jung */

#ifndef PREVIEW_WALKING_FORM_H
#define PREVIEW_WALKING_FORM_H

#include <iostream>
#include <QWidget>

#include "ui_preview_walking_form.h"
#include "qnode.hpp"

namespace Ui
{
  class PreviewWalkingForm;
}

class PreviewWalkingForm : public QWidget
{
  Q_OBJECT

public:
  explicit PreviewWalkingForm(QWidget *parent = 0);
  ~PreviewWalkingForm();

  bool setQNode(robotis_op::QNodeOP3 *qnode);
  bool init(robotis_op::QNodeOP3 *qnode);

public Q_SLOTS:



  /******************************************
  ** Auto-connections (connectSlotsByName())
  *******************************************/

  // Hàm được gọi khi nút quay trái được nhấn
  void on_button_p_walking_turn_l_clicked(bool check);

  // Hàm được gọi khi nút quay phải được nhấn
  void on_button_p_walking_turn_r_clicked(bool check);

  // Hàm được gọi khi nút đi về phía trước được nhấn
  void on_button_p_walking_forward_clicked(bool check);

  // Hàm được gọi khi nút đi về phía sau được nhấn
  void on_button_p_walking_backward_clicked(bool check);

  // Hàm được gọi khi nút dừng lại được nhấn
  void on_button_p_walking_stop_clicked(bool check);

  // Hàm được gọi khi nút rẽ trái được nhấn
  void on_button_p_walking_left_clicked(bool check);

  // Hàm được gọi khi nút rẽ phải được nhấn
  void on_button_p_walking_right_clicked(bool check);

  // Hàm được gọi khi nút "Set Walking Parameter" được nhấn
  void on_button_set_walking_param_clicked(bool check);

  // Hàm được gọi khi nút "Send Body Offset" được nhấn
  void on_button_send_body_offset_clicked(bool check);

  // Hàm được gọi khi nút "Send Foot Distance" được nhấn
  void on_button_send_foot_distance_clicked(bool check);

  // Hàm được gọi khi nút "Walking Init Pose" được nhấn
  void on_button_p_walking_init_pose_clicked(bool check);

  // Hàm được gọi khi nút "Walking Balance On" được nhấn
  void on_button_p_walking_balance_on_clicked(bool check);

  // Hàm được gọi khi nút "Walking Balance Off" được nhấn
  void on_button_p_walking_balance_off_clicked(bool check);

  // Hàm được gọi khi nút "Set Marker" được nhấn
  void on_button_marker_set_clicked(bool check);

  // Hàm được gọi khi nút "Clear Marker" được nhấn
  void on_button_marker_clear_clicked(bool check);

  // Hàm được gọi khi nút "Footstep Plan" được nhấn
  void on_button_footstep_plan_clicked(bool check);

  // Hàm được gọi khi nút "Footstep Clear" được nhấn
  void on_button_footstep_clear_clicked(bool check);

  // Hàm được gọi khi nút "Footstep Go" được nhấn
  void on_button_footstep_go_clicked(bool check);

  // Hàm được gọi khi giá trị của dSpinBox_marker_pos_x thay đổi
  void on_dSpinBox_marker_pos_x_valueChanged(double value);

  // Hàm được gọi khi giá trị của dSpinBox_marker_pos_y thay đổi
  void on_dSpinBox_marker_pos_y_valueChanged(double value);

  // Hàm được gọi khi giá trị của dSpinBox_marker_pos_z thay đổi
  void on_dSpinBox_marker_pos_z_valueChanged(double value);

  // Hàm được gọi khi giá trị của dSpinBox_marker_ori_r thay đổi
  void on_dSpinBox_marker_ori_r_valueChanged(double value);

  // Hàm được gọi khi giá trị của dSpinBox_marker_ori_p thay đổi
  void on_dSpinBox_marker_ori_p_valueChanged(double value);

  // Hàm được gọi khi giá trị của dSpinBox_marker_ori_y thay đổi
  void on_dSpinBox_marker_ori_y_valueChanged(double value);

  // Interactive marker
  // Hàm cập nhật Panel cho điểm (Point) tương tác
  void updatePointPanel(const geometry_msgs::Point point);

  // Hàm cập nhật Panel cho vị trí (Pose) tương tác
  void updatePosePanel(const geometry_msgs::Pose pose);







private:
  Ui::PreviewWalkingForm *p_walking_ui;
  robotis_op::QNodeOP3 *qnode_op3_;

  // preview walking
  // Hàm gửi lệnh điều khiển cho chế độ xem trước đi bộ
  void sendPWalkingCommand(const std::string &command, bool set_start_foot = true);

  // interactive marker
  // Hàm tạo đối tượng tương tác (Interactive Marker)
  void makeInteractiveMarker();

  // Hàm cập nhật đối tượng tương tác (Interactive Marker)
  void updateInteractiveMarker();

  // Hàm xóa thông tin trên Panel hiển thị đối tượng tương tác
  void clearMarkerPanel();

  // update marker UI
  // Hàm lấy vị trí (Pose) từ Panel hiển thị đối tượng tương tác
  void getPoseFromMarkerPanel(geometry_msgs::Pose &current);

  // Hàm đặt vị trí (Pose) lên Panel hiển thị đối tượng tương tác
  void setPoseToMarkerPanel(const geometry_msgs::Pose &current);

  // Hàm lấy điểm (Point) từ Panel hiển thị đối tượng tương tác
  void getPointFromMarkerPanel(geometry_msgs::Point &current);

  // Hàm đặt điểm (Point) lên Panel hiển thị đối tượng tương tác
  void setPointToMarkerPanel(const geometry_msgs::Point &current);






  /******************************************
   ** Transformation
   *******************************************/

  // Chuyển ma trận quay thành góc Roll-Pitch-Yaw (RPY)
  Eigen::Vector3d rotation2rpy(const Eigen::MatrixXd &rotation);

  // Chuyển góc Roll-Pitch-Yaw (RPY) thành ma trận quay
  Eigen::MatrixXd rpy2rotation(const double &roll, const double &pitch, const double &yaw);

  // Chuyển góc Roll-Pitch-Yaw (RPY) thành đơn vị quaternion
  Eigen::Quaterniond rpy2quaternion(const Eigen::Vector3d &euler);

  // Chuyển góc Roll-Pitch-Yaw (RPY) thành đơn vị quaternion
  Eigen::Quaterniond rpy2quaternion(const double &roll, const double &pitch, const double &yaw);

  // Chuyển ma trận quay thành đơn vị quaternion
  Eigen::Quaterniond rotation2quaternion(const Eigen::MatrixXd &rotation);

  // Chuyển đơn vị quaternion thành góc Roll-Pitch-Yaw (RPY)
  Eigen::Vector3d quaternion2rpy(const Eigen::Quaterniond &quaternion);

  // Chuyển đối tượng quaternion từ geometry_msgs::Quaternion thành góc Roll-Pitch-Yaw (RPY)
  Eigen::Vector3d quaternion2rpy(const geometry_msgs::Quaternion &quaternion);

  // Chuyển đơn vị quaternion thành ma trận quay
  Eigen::MatrixXd quaternion2rotation(const Eigen::Quaterniond &quaternion);

  // Tạo ma trận quay quanh trục X với góc quay là angle radians
  Eigen::MatrixXd rotationX(const double &angle);

  // Tạo ma trận quay quanh trục Y với góc quay là angle radians
  Eigen::MatrixXd rotationY(const double &angle);

  // Tạo ma trận quay quanh trục Z với góc quay là angle radians
  Eigen::MatrixXd rotationZ(const double &angle);

  // Biến kiểm tra trạng thái cập nhật
  bool is_updating_;


};

#endif // PREVIEW_WALKING_FORM_H

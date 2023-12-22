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

/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef OP3_DEMO_QNODE_HPP_
#define OP3_DEMO_QNODE_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/
#ifndef Q_MOC_RUN

#include <string>
#include <sstream>

#include <QThread>
#include <QStringListModel>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <interactive_markers/interactive_marker_server.h>
#include <eigen_conversions/eigen_msg.h>
#include <boost/thread.hpp>
#include <eigen3/Eigen/Eigen>
#include <yaml-cpp/yaml.h>

#include "robotis_controller_msgs/JointCtrlModule.h"
#include "robotis_controller_msgs/GetJointModule.h"
#include "robotis_controller_msgs/StatusMsg.h"
#include "robotis_controller_msgs/SyncWriteItem.h"

// walking demo
#include "op3_walking_module_msgs/WalkingParam.h"
#include "op3_walking_module_msgs/GetWalkingParam.h"
#include "op3_walking_module_msgs/SetWalkingParam.h"

// Preview walking
#include "op3_online_walking_module_msgs/FootStepCommand.h"
#include "op3_online_walking_module_msgs/WalkingParam.h"
#include "op3_online_walking_module_msgs/JointPose.h"
#include "op3_online_walking_module_msgs/Step2DArray.h"
#include "humanoid_nav_msgs/PlanFootsteps.h"

#endif

#define DEG2RAD   (M_PI / 180.0)
#define RAD2DEG   (180.0 / M_PI)
/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace robotis_op
{

/*****************************************************************************
 ** Class
 *****************************************************************************/

// Lớp QNodeOP3 kế thừa từ lớp QThread trong Qt framework.
class QNodeOP3 : public QThread
{
  Q_OBJECT  // Macro Q_OBJECT hỗ trợ tính năng quản lý sự kiện và các tính năng khác của QObject trong Qt framework.
public:

  // Enum LogLevel để định nghĩa mức độ log.
  enum LogLevel
  {
    Debug = 0,   // Cấp độ Debug: Các thông điệp log thông tin chi tiết cho việc debug ứng dụng.
    Info = 1,    // Cấp độ Info: Các thông điệp log thông tin chung về quá trình thực thi của ứng dụng.
    Warn = 2,    // Cấp độ Warn: Cảnh báo, thông điệp log để thông báo về các tình huống không mong muốn nhưng không phải lỗi nghiêm trọng.
    Error = 3,   // Cấp độ Error: Các thông điệp log thông báo về lỗi trong quá trình thực thi của ứng dụng.
    Fatal = 4    // Cấp độ Fatal: Các thông điệp log thông báo về lỗi nghiêm trọng dẫn đến việc chương trình không thể tiếp tục thực thi.
  };


  // Constructor: Khởi tạo đối tượng QNodeOP3 với tham số dòng lệnh
QNodeOP3(int argc, char** argv);

// Destructor: Hàm hủy, giải phóng tài nguyên khi đối tượng QNodeOP3 bị hủy
virtual ~QNodeOP3();

// Khởi tạo lớp QNodeOP3, trả về true nếu thành công, false nếu có lỗi
bool init();

// Hàm ảo của QThread, chứa mã nguồn thực thi khi luồng bắt đầu
void run();

// Trả về con trỏ đến đối tượng QStringListModel để lưu trữ thông điệp log trong ứng dụng
QStringListModel* loggingModel()
{
  return &logging_model_;
}

// Hàm log() để ghi thông điệp log với mức độ log, thông điệp và người gửi cụ thể.
void log(const LogLevel &level, const std::string &msg, std::string sender = "Demo");

// Hàm clearLog() để xóa sạch các thông điệp log.
void clearLog();

// Hàm assemble_lidar() để thực hiện việc lắp ráp LiDAR (Light Detection and Ranging).
void assemble_lidar();

// Hàm setJointControlMode() để đặt chế độ điều khiển cho các khớp robot dựa trên thông điệp JointCtrlModule.
void setJointControlMode(const robotis_controller_msgs::JointCtrlModule &msg);

// Hàm setControlMode() để đặt chế độ điều khiển tổng quát cho robot dựa trên tên chế độ.
void setControlMode(const std::string &mode);

// Hàm getJointNameFromID() để lấy tên của khớp dựa trên ID của khớp.
bool getJointNameFromID(const int &id, std::string &joint_name);

// Hàm getIDFromJointName() để lấy ID của khớp dựa trên tên của khớp.
bool getIDFromJointName(const std::string &joint_name, int &id);

// Hàm getIDJointNameFromIndex() để lấy ID và tên của khớp từ một chỉ số cụ thể.
bool getIDJointNameFromIndex(const int &index, int &id, std::string &joint_name);

// Hàm getModeName() để lấy tên chế độ dựa trên một chỉ số cụ thể.
std::string getModeName(const int &index);

// Hàm getModeIndex() để lấy chỉ số của một chế độ dựa trên tên chế độ.
int getModeIndex(const std::string &mode_name);

// Hàm getModeSize() để lấy kích thước của danh sách chế độ.
int getModeSize();

// Hàm getJointSize() để lấy kích thước của danh sách các khớp.
int getJointSize();

// Hàm clearUsingModule() để xóa sạch danh sách các module đang sử dụng.
void clearUsingModule();

// Hàm isUsingModule() để kiểm tra xem một module có đang sử dụng hay không.
bool isUsingModule(std::string module_name);

// Hàm moveInitPose() để thực hiện chuyển động khởi tạo của robot.
void moveInitPose();

// Hàm init_default_demo() để khởi tạo các giá trị mặc định cho demo sử dụng ROS NodeHandle.
void init_default_demo(ros::NodeHandle &ros_node);


  // Head control
// Hàm setHeadJoint() để đặt góc nghiêng và quay của đầu robot.
void setHeadJoint(double pan, double tilt);


  // Walking
// Hàm setWalkingCommand() để đặt lệnh điều khiển cho chuyển động đi bộ dựa trên một chuỗi lệnh.
void setWalkingCommand(const std::string &command);

// Hàm refreshWalkingParam() để làm mới các thông số liên quan đến chuyển động đi bộ.
void refreshWalkingParam();

// Hàm saveWalkingParam() để lưu các thông số liên quan đến chuyển động đi bộ.
void saveWalkingParam();

// Hàm applyWalkingParam() để áp dụng các thông số liên quan đến chuyển động đi bộ dựa trên một cấu trúc WalkingParam.
void applyWalkingParam(const op3_walking_module_msgs::WalkingParam &walking_param);

// Hàm initGyro() để khởi tạo cảm biến giảm tốc (gyro) cho chuyển động đi bộ.
void initGyro();


  // Preview Walking
// Hàm init_preview_walking() để khởi tạo chế độ xem trước chuyển động đi bộ với một ROS NodeHandle.
void init_preview_walking(ros::NodeHandle &ros_node);

// Hàm sendFootStepCommandMsg() để gửi thông điệp lệnh bước chân cho chuyển động đi bộ trực tuyến.
void sendFootStepCommandMsg(op3_online_walking_module_msgs::FootStepCommand msg);

// Hàm sendWalkingParamMsg() để gửi thông điệp các tham số chuyển động đi bộ cho chuyển động đi bộ trực tuyến.
void sendWalkingParamMsg(op3_online_walking_module_msgs::WalkingParam msg);

// Hàm sendBodyOffsetMsg() để gửi thông điệp vị trí offset của cơ thể cho chuyển động đi bộ trực tuyến.
void sendBodyOffsetMsg(geometry_msgs::Pose msg);

// Hàm sendFootDistanceMsg() để gửi thông điệp khoảng cách giữa hai chân cho chuyển động đi bộ trực tuyến.
void sendFootDistanceMsg(std_msgs::Float64 msg);

// Hàm sendResetBodyMsg() để gửi thông điệp đặt lại vị trí cơ thể cho chuyển động đi bộ trực tuyến.
void sendResetBodyMsg(std_msgs::Bool msg);

// Hàm sendWholebodyBalanceMsg() để gửi thông điệp cân bằng toàn bộ cơ thể cho chuyển động đi bộ trực tuyến.
void sendWholebodyBalanceMsg(std_msgs::String msg);

// Hàm parseIniPoseData() để phân tích dữ liệu vị trí khởi tạo từ một đường dẫn tệp cụ thể.
void parseIniPoseData(const std::string &path);

// Hàm sendJointPoseMsg() để gửi thông điệp vị trí khớp cụ thể cho chuyển động đi bộ trực tuyến.
void sendJointPoseMsg(op3_online_walking_module_msgs::JointPose msg);


  // Preview /w footstep
// Hàm makeFootstepUsingPlanner() để tạo bước chân sử dụng bộ lập kế hoạch mặc định.
void makeFootstepUsingPlanner();

// Hàm makeFootstepUsingPlanner() để tạo bước chân sử dụng bộ lập kế hoạch với một vị trí chân mục tiêu cụ thể.
void makeFootstepUsingPlanner(const geometry_msgs::Pose &target_foot_pose);

// Hàm visualizePreviewFootsteps() để hiển thị trước bước chân, với tùy chọn xóa các bước chân hiện tại.
void visualizePreviewFootsteps(bool clear);

// Hàm clearFootsteps() để xóa sạch tất cả các bước chân đã được tạo.
void clearFootsteps();

// Hàm setWalkingFootsteps() để đặt bước chân cho chuyển động đi bộ với thời gian giữa các bước được chỉ định.
void setWalkingFootsteps(const double &step_time);




  // Demo
// Hàm setDemoCommand() để đặt lệnh demo cho robot dựa trên một chuỗi lệnh cụ thể.
void setDemoCommand(const std::string &command);

// Hàm setActionModuleBody() để đặt module hành động cho cơ thể robot.
void setActionModuleBody();

// Hàm setModuleToDemo() để đặt module cho chế độ demo của robot.
void setModuleToDemo();


  // Interactive marker
// Hàm makeInteractiveMarker() để tạo một điểm đánh dấu tương tác với vị trí được xác định.
void makeInteractiveMarker(const geometry_msgs::Pose& marker_pose);

// Hàm updateInteractiveMarker() để cập nhật vị trí của điểm đánh dấu tương tác với vị trí được xác định.
bool updateInteractiveMarker(const geometry_msgs::Pose& pose);

// Hàm getInteractiveMarkerPose() để lấy vị trí hiện tại của điểm đánh dấu tương tác.
void getInteractiveMarkerPose();

// Hàm clearInteractiveMarker() để xóa điểm đánh dấu tương tác hiện tại.
void clearInteractiveMarker();

// Bảng ánh xạ module và motion
std::map<int, std::string> module_table_;          // Bảng ánh xạ ID module với tên module tương ứng.
std::map<int, std::string> motion_table_;          // Bảng ánh xạ ID motion với tên motion tương ứng.
std::map<int, int> motion_shortcut_table_;         // Bảng ánh xạ ID motion với phím tắt tương ứng.


public Q_SLOTS:
  // Hàm getJointControlMode() để lấy chế độ điều khiển của các khớp.
  void getJointControlMode();

  // Hàm playMotion() để phát một chuyển động dựa trên chỉ số motion_index.
  void playMotion(int motion_index);


Q_SIGNALS:
  // Tín hiệu loggingUpdated() được phát khi dữ liệu log được cập nhật.
  void loggingUpdated();

  // Tín hiệu rosShutdown() được phát khi ROS được tắt.
  void rosShutdown();

  // Tín hiệu updateCurrentJointControlMode() được phát khi chế độ điều khiển của các khớp được cập nhật.
  void updateCurrentJointControlMode(std::vector<int> mode);



  // Head
// Hàm updateHeadAngles() để cập nhật góc quay và nghiêng của đầu robot.
void updateHeadAngles(double pan, double tilt);


  // Walking
// Hàm updateWalkingParameters() để cập nhật các tham số liên quan đến chuyển động đi bộ dựa trên thông điệp WalkingParam.
void updateWalkingParameters(op3_walking_module_msgs::WalkingParam params);

  // Interactive marker
// Hàm updateDemoPoint() để cập nhật điểm demo dựa trên thông điệp Point.
void updateDemoPoint(const geometry_msgs::Point point);

// Hàm updateDemoPose() để cập nhật vị trí demo dựa trên thông điệp Pose.
void updateDemoPose(const geometry_msgs::Pose pose);

private:
  // Hàm này được sử dụng để phân tích tên của các khớp từ một tệp YAML
  void parseJointNameFromYaml(const std::string &path);

  // Hàm này được sử dụng để phân tích bản đồ chuyển động từ một tệp YAML
  void parseMotionMapFromYaml(const std::string &path);

  // Hàm này được gọi để cập nhật callback khi thông điệp về điều khiển khớp hiện tại được nhận
  void refreshCurrentJointControlCallback(const robotis_controller_msgs::JointCtrlModule::ConstPtr &msg);

  // Hàm này được gọi khi callback cập nhật trạng thái của các khớp đầu của robot
  void updateHeadJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);

  // Hàm này được gọi khi callback nhận thông điệp trạng thái từ hệ thống điều khiển robot
  void statusMsgCallback(const robotis_controller_msgs::StatusMsg::ConstPtr &msg);


  // interactive marker
  // Hàm này được gọi khi callback nhận thông điệp chứa dữ liệu điểm (point) và thông tin đánh dấu thời gian (stamped)
  void pointStampedCallback(const geometry_msgs::PointStamped::ConstPtr &msg);

  // Hàm này được gọi khi callback nhận thông điệp chứa phản hồi từ điều khiển tương tác (interactive marker)
  void interactiveMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  // localization
  bool transformPose(const std::string &from_id, const std::string &to_id, const geometry_msgs::Pose &from_pose,
                     geometry_msgs::Pose &to_pose, bool inverse = false);

  int init_argc_;
  char** init_argv_;
  bool debug_;
  double body_height_;

  // interactive marker
  ros::Subscriber rviz_clicked_point_sub_;
  std::string frame_id_;
  std::string marker_name_;
  geometry_msgs::Pose pose_from_ui_;
  geometry_msgs::Pose current_pose_;
  geometry_msgs::Pose curr_pose_msg_;
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> interactive_marker_server_;
  boost::shared_ptr<tf::TransformListener> tf_listener_;

  op3_walking_module_msgs::WalkingParam walking_param_;

  ros::Publisher init_pose_pub_;
  ros::Publisher module_control_pub_;
  ros::Publisher module_control_preset_pub_;
  ros::Publisher init_gyro_pub_;
  ros::Subscriber status_msg_sub_;
  ros::Subscriber init_ft_foot_sub_;
  ros::Subscriber both_ft_foot_sub_;
  ros::Subscriber current_module_control_sub_;
  ros::ServiceClient get_module_control_client_;

  // Head
  ros::Publisher set_head_joint_angle_pub_;
  ros::Subscriber current_joint_states_sub_;

  // Walking
  ros::Publisher set_walking_command_pub;
  ros::Publisher set_walking_param_pub;
  ros::ServiceClient get_walking_param_client_;

  // preview walking
  ros::ServiceClient humanoid_footstep_client_;
  ros::Publisher foot_step_command_pub_;
  ros::Publisher set_walking_footsteps_pub_;
  ros::Publisher walking_param_pub_;
  ros::Publisher body_offset_pub_;
  ros::Publisher foot_distance_pub_;
  ros::Publisher wholebody_balance_pub_;
  ros::Publisher reset_body_msg_pub_;
  ros::Publisher joint_pose_msg_pub_;

  ros::Publisher marker_pub_;

  std::vector<geometry_msgs::Pose2D> preview_foot_steps_;
  std::vector<int> preview_foot_types_;

  // Action
  ros::Publisher motion_index_pub_;

  // Demo
  ros::Publisher demo_command_pub_;

  ros::Time start_time_;

  QStringListModel logging_model_;
  std::map<int, std::string> id_joint_table_;
  std::map<std::string, int> joint_id_table_;

  std::map<int, std::string> index_mode_table_;
  std::map<std::string, int> mode_index_table_;
  std::map<std::string, bool> using_mode_table_;
};

}  // namespace robotis_op

template<typename T>
T deg2rad(T deg)
{
  return deg * M_PI / 180;
}

template<typename T>
T rad2deg(T rad)
{
  return rad * 180 / M_PI;
}
#endif /* OP3_DEMO_QNODE_HPP_ */

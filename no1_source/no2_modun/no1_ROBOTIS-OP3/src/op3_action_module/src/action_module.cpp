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

#include <stdio.h>
#include <sstream>
#include "op3_action_module/action_module.h"

namespace robotis_op
{

  // Hàm chuyển đổi một số nguyên thành chuỗi.
  std::string ActionModule::convertIntToString(int n)
  {
    // Sử dụng stringstream để chuyển đổi số nguyên thành chuỗi.
    std::ostringstream ostr;
    ostr << n;
    return ostr.str(); // Trả về chuỗi đã chuyển đổi.
  }

  // Hàm khởi tạo của lớp ActionModule.
  ActionModule::ActionModule()
      : control_cycle_msec_(8),
        PRE_SECTION(0),
        MAIN_SECTION(1),
        POST_SECTION(2),
        PAUSE_SECTION(3),
        ZERO_FINISH(0),
        NONE_ZERO_FINISH(1),
        DEBUG_PRINT(false)
  {
    /////////////// Const Variable
    /**************************************
     * Section             /----\
     *                    /|    |\
     *        /+---------/ |    | \
     *       / |        |  |    |  \
     * -----/  |        |  |    |   \----
     *      PRE  MAIN   PRE MAIN POST PAUSE
     ***************************************/

    // Các biến hằng và cờ được khởi tạo ở đây.

    enable_ = false;                                    // Không kích hoạt mô-đun khi mới tạo.
    module_name_ = "action_module";                     // Đặt tên duy nhất cho mô-đun.
    control_mode_ = robotis_framework::PositionControl; // Chế độ kiểm soát.

    //////////////////////////////////
    action_file_ = 0;               // Không có tệp hành động được mở khi mới tạo.
    playing_ = false;               // Không đang chơi trang hành động.
    first_driving_start_ = false;   // Chưa bắt đầu lái đầu tiên.
    playing_finished_ = true;       // Chưa bắt đầu chơi.
    page_step_count_ = 0;           // Số bước của trang hiện tại.
    play_page_idx_ = 0;             // Chỉ số trang đang chơi.
    stop_playing_ = true;           // Đang dừng chơi.
    action_module_enabled_ = false; // Mô-đun chưa được kích hoạt.
    previous_running_ = false;      // Trạng thái chạy trước đó.
    present_running_ = false;       // Trạng thái chạy hiện tại.
  }

  // Hàm hủy của lớp ActionModule.
  ActionModule::~ActionModule()
  {
    queue_thread_.join(); // Đợi đến khi luồng hàng đợi kết thúc.

    ////////////////////////////////////////
    if (action_file_ != 0)
      fclose(action_file_); // Đóng tệp hành động nếu nó đã được mở.
  }

  // Hàm khởi tạo của lớp ActionModule.
  void ActionModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
  {
    control_cycle_msec_ = control_cycle_msec;                                     // Thiết lập chu kỳ kiểm soát.
    queue_thread_ = boost::thread(boost::bind(&ActionModule::queueThread, this)); // Bắt đầu luồng hàng đợi.

    // Khởi tạo kết quả và bảng id của các động cơ.
    for (std::map<std::string, robotis_framework::Dynamixel *>::iterator it = robot->dxls_.begin();
         it != robot->dxls_.end(); it++)
    {
      std::string joint_name = it->first;
      robotis_framework::Dynamixel *dxl_info = it->second;

      joint_name_to_id_[joint_name] = dxl_info->id_;
      joint_id_to_name_[dxl_info->id_] = joint_name;
      action_result_[joint_name] = new robotis_framework::DynamixelState();
      action_result_[joint_name]->goal_position_ = dxl_info->dxl_state_->goal_position_;
      result_[joint_name] = new robotis_framework::DynamixelState();
      result_[joint_name]->goal_position_ = dxl_info->dxl_state_->goal_position_;
      action_joints_enable_[joint_name] = false;
    }

    ros::NodeHandle ros_node;

    // Đặt đường dẫn mặc định của tệp hành động.
    std::string path = ros::package::getPath("op3_action_module") + "/data/motion_4095.bin";
    std::string action_file_path = ros_node.param<std::string>("action_file_path", path);

    loadFile(action_file_path); // Nạp tệp hành động.

    playing_ = false; // Chưa bắt đầu chơi.
  }

  // Hàm chạy trong luồng hàng đợi.
  void ActionModule::queueThread()
  {
    ros::NodeHandle ros_node;
    ros::CallbackQueue callback_queue;

    ros_node.setCallbackQueue(&callback_queue);

    /* publisher */
    status_msg_pub_ = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 0);
    done_msg_pub_ = ros_node.advertise<std_msgs::String>("/robotis/movement_done", 1);

    /* subscriber */
    ros::Subscriber action_page_sub = ros_node.subscribe("/robotis/action/page_num", 0, &ActionModule::pageNumberCallback,
                                                         this);
    ros::Subscriber start_action_sub = ros_node.subscribe("/robotis/action/start_action", 0,
                                                          &ActionModule::startActionCallback, this);

    /* ROS Service Callback Functions */
    ros::ServiceServer is_running_server = ros_node.advertiseService("/robotis/action/is_running",
                                                                     &ActionModule::isRunningServiceCallback, this);

    ros::WallDuration duration(control_cycle_msec_ / 1000.0);
    while (ros_node.ok())
      callback_queue.callAvailable(duration);
  }

  // Hàm callback cho dịch vụ kiểm tra trạng thái chạy.
  bool ActionModule::isRunningServiceCallback(op3_action_module_msgs::IsRunning::Request &req,
                                              op3_action_module_msgs::IsRunning::Response &res)
  {
    res.is_running = isRunning(); // Gọi hàm kiểm tra trạng thái chạy và trả về kết quả.
    return true;
  }

  // Hàm callback khi nhận được số trang hành động từ ROS.
  void ActionModule::pageNumberCallback(const std_msgs::Int32::ConstPtr &msg)
  {
    // Kiểm tra mô-đun có được kích hoạt không.
    if (enable_ == false)
    {
      std::string status_msg = "Action Module is not enabled";
      ROS_INFO_STREAM(status_msg);
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
      return;
    }

    // Xử lý theo giá trị số trang nhận được.
    if (msg->data == -1)
    {
      stop(); // Dừng lại nếu số trang là -1.
    }
    else if (msg->data == -2)
    {
      brake(); // Kích hoạt chế độ phanh nếu số trang là -2.
    }
    else
    {
      // Kích hoạt tất cả các khớp.
      for (std::map<std::string, bool>::iterator joints_enable_it = action_joints_enable_.begin();
           joints_enable_it != action_joints_enable_.end(); joints_enable_it++)
        joints_enable_it->second = true;

      // Bắt đầu chạy hành động trang được chỉ định.
      if (start(msg->data) == true)
      {
        std::string status_msg = "Succeed to start page " + convertIntToString(msg->data);
        ROS_INFO_STREAM(status_msg);
        publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
      }
      else
      {
        std::string status_msg = "Failed to start page " + convertIntToString(msg->data);
        ROS_ERROR_STREAM(status_msg);
        publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
        publishDoneMsg("action_failed");
      }
    }
  }

  // Hàm callback khi nhận yêu cầu bắt đầu hành động từ ROS.
  void ActionModule::startActionCallback(const op3_action_module_msgs::StartAction::ConstPtr &msg)
  {
    // Kiểm tra mô-đun có được kích hoạt không.
    if (enable_ == false)
    {
      std::string status_msg = "Action Module is not enabled";
      ROS_INFO_STREAM(status_msg);
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
      return;
    }

    // Xử lý dựa trên giá trị số trang nhận được.
    if (msg->page_num == -1)
    {
      stop(); // Dừng lại nếu số trang là -1.
    }
    else if (msg->page_num == -2)
    {
      brake(); // Kích hoạt chế độ phanh nếu số trang là -2.
    }
    else
    {
      // Tắt tất cả các khớp trước khi bắt đầu hành động mới.
      for (std::map<std::string, bool>::iterator joints_enable_it = action_joints_enable_.begin();
           joints_enable_it != action_joints_enable_.end(); joints_enable_it++)
        joints_enable_it->second = false;

      // Kích hoạt các khớp được chỉ định trong yêu cầu.
      int joint_name_array_size = msg->joint_name_array.size();
      std::map<std::string, bool>::iterator joints_enable_it = action_joints_enable_.begin();
      for (int joint_idx = 0; joint_idx < joint_name_array_size; joint_idx++)
      {
        joints_enable_it = action_joints_enable_.find(msg->joint_name_array[joint_idx]);
        if (joints_enable_it == action_joints_enable_.end())
        {
          std::string status_msg = "Invalid Joint Name : " + msg->joint_name_array[joint_idx];
          ROS_INFO_STREAM(status_msg);
          publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
          publishDoneMsg("action_failed");
          return;
        }
        else
        {
          joints_enable_it->second = true;
        }
      }

      // Bắt đầu chạy hành động trang được chỉ định.
      if (start(msg->page_num) == true)
      {
        std::string status_msg = "Succeed to start page " + convertIntToString(msg->page_num);
        ROS_INFO_STREAM(status_msg);
        publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
      }
      else
      {
        std::string status_msg = "Failed to start page " + convertIntToString(msg->page_num);
        ROS_ERROR_STREAM(status_msg);
        publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
        publishDoneMsg("action_failed");
      }
    }
  }

  // Hàm xử lý chính của ActionModule.
  void ActionModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                             std::map<std::string, double> sensors)
  {
    // Kiểm tra xem mô-đun có được kích hoạt không.
    if (enable_ == false)
      return;

    // Kiểm tra xem mô-đun hành động có được kích hoạt không.
    if (action_module_enabled_ == true)
    {
      // Cập nhật giá trị goal_position của các khớp trong action_result_.
      for (std::map<std::string, robotis_framework::Dynamixel *>::iterator dxls_it = dxls.begin(); dxls_it != dxls.end();
           dxls_it++)
      {
        std::string joint_name = dxls_it->first;

        std::map<std::string, robotis_framework::DynamixelState *>::iterator result_it = result_.find(joint_name);
        if (result_it == result_.end())
          continue;
        else
        {
          result_it->second->goal_position_ = dxls_it->second->dxl_state_->goal_position_;
          action_result_[joint_name]->goal_position_ = dxls_it->second->dxl_state_->goal_position_;
        }
      }
      action_module_enabled_ = false;
    }

    // Xử lý quá trình chơi hành động.
    actionPlayProcess(dxls);

    // Cập nhật goal_position của các khớp được kích hoạt trong action_joints_enable_.
    for (std::map<std::string, bool>::iterator action_enable_it = action_joints_enable_.begin();
         action_enable_it != action_joints_enable_.end(); action_enable_it++)
    {
      if (action_enable_it->second == true)
        result_[action_enable_it->first]->goal_position_ = action_result_[action_enable_it->first]->goal_position_;
    }

    // Cập nhật trạng thái chạy trước và hiện tại.
    previous_running_ = present_running_;
    present_running_ = isRunning();

    // Kiểm tra xem trạng thái chạy có thay đổi không.
    if (present_running_ != previous_running_)
    {
      // Nếu bắt đầu chạy hành động.
      if (present_running_ == true)
      {
        std::string status_msg = "Action_Start";
        // ROS_INFO_STREAM(status_msg);
        publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
      }
      // Nếu kết thúc chạy hành động.
      else
      {
        // Cập nhật goal_position của các khớp trong action_result_.
        for (std::map<std::string, robotis_framework::DynamixelState *>::iterator action_result_it =
                 action_result_.begin();
             action_result_it != action_result_.end(); action_result_it++)
          action_result_it->second->goal_position_ = result_[action_result_it->first]->goal_position_;

        std::string status_msg = "Action_Finish";
        // ROS_INFO_STREAM(status_msg);
        publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_INFO, status_msg);
        publishDoneMsg("action");
      }
    }
  }

  // Hàm kích hoạt mô-đun hành động.
  void ActionModule::onModuleEnable()
  {
    action_module_enabled_ = true;
  }

  // Hàm vô hiệu hóa mô-đun hành động và thực hiện phanh.
  void ActionModule::onModuleDisable()
  {
    action_module_enabled_ = false;
    brake();
  }

  // Hàm dừng chơi hành động.
  void ActionModule::stop()
  {
    stop_playing_ = true;
  }

  // Hàm kiểm tra xem mô-đun hành động có đang chạy hay không.
  bool ActionModule::isRunning()
  {
    return playing_;
  }

  // Hàm chuyển đổi giá trị góc từ rad sang w4095.
  int ActionModule::convertRadTow4095(double rad)
  {
    return static_cast<int>((rad + M_PI) * 2048.0 / M_PI);
  }

  // Hàm chuyển đổi giá trị w4095 sang rad.
  double ActionModule::convertw4095ToRad(int w4095)
  {
    return (w4095 - 2048) * M_PI / 2048.0;
  }

  // Hàm kiểm tra checksum của một trang hành động.
  bool ActionModule::verifyChecksum(action_file_define::Page *page)
  {
    unsigned char checksum = 0x00;
    unsigned char *pt = reinterpret_cast<unsigned char *>(page);

    for (unsigned int i = 0; i < sizeof(action_file_define::Page); i++)
    {
      checksum += *pt;
      pt++;
    }

    // So sánh checksum với giá trị kiểm tra, trả về kết quả.
    return (checksum == 0xff);
  }

  // Hàm đặt checksum cho một trang hành động.
  void ActionModule::setChecksum(action_file_define::Page *page)
  {
    unsigned char checksum = 0x00;
    unsigned char *pt = reinterpret_cast<unsigned char *>(page);

    page->header.checksum = 0x00;

    for (unsigned int i = 0; i < sizeof(action_file_define::Page); i++)
    {
      checksum += *pt;
      pt++;
    }

    page->header.checksum = static_cast<unsigned char>(0xff - checksum);
  }

  // Hàm tải tệp hành động từ đường dẫn được chỉ định.
  bool ActionModule::loadFile(std::string file_name)
  {
    // Mở tệp hành động ở chế độ đọc và ghi nhị phân.
    FILE *action = fopen(file_name.c_str(), "r+b");
    if (action == nullptr)
    {
      std::string status_msg = "Cannot open Action file!";
      ROS_ERROR_STREAM(status_msg);
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
      return false;
    }

    // Di chuyển con trỏ tệp về cuối để kiểm tra kích thước.
    fseek(action, 0, SEEK_END);
    if (ftell(action) != static_cast<long>(sizeof(action_file_define::Page) * action_file_define::MAXNUM_PAGE))
    {
      std::string status_msg = "It's not an Action file!";
      ROS_ERROR_STREAM(status_msg);
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
      fclose(action);
      return false;
    }

    // Đóng tệp trước đó nếu đã mở.
    if (action_file_ != nullptr)
      fclose(action_file_);

    // Gán con trỏ tệp mới.
    action_file_ = action;
    return true;
  }

  // Hàm tạo một tệp hành động mới với đường dẫn được chỉ định.
  bool ActionModule::createFile(std::string file_name)
  {
    // Mở tệp hành động để ghi vào cuối tệp.
    FILE *action = fopen(file_name.c_str(), "ab");
    if (action == nullptr)
    {
      std::string status_msg = "Cannot create Action file!";
      ROS_ERROR_STREAM(status_msg);
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
      return false;
    }

    // Tạo trang mẫu và ghi nó vào tệp để đảm bảo kích thước đúng.
    action_file_define::Page page;
    resetPage(&page);

    for (int i = 0; i < action_file_define::MAXNUM_PAGE; i++)
      fwrite(static_cast<const void *>(&page), 1, sizeof(action_file_define::Page), action);

    // Đóng tệp trước đó nếu đã mở.
    if (action_file_ != nullptr)
      fclose(action_file_);

    // Gán con trỏ tệp mới.
    action_file_ = action;

    return true;
  }

  // Hàm bắt đầu chơi một trang hành động dựa trên số trang được chỉ định.
  bool ActionModule::start(int page_number)
  {
    // Kiểm tra tính hợp lệ của số trang.
    if (page_number < 1 || page_number >= action_file_define::MAXNUM_PAGE)
    {
      std::string status_msg = "Cannot play page. (" + convertIntToString(page_number) + " is an invalid index)";
      ROS_ERROR_STREAM(status_msg);
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
      return false;
    }

    // Tạo một trang mới và nạp dữ liệu từ tệp.
    action_file_define::Page page;
    if (loadPage(page_number, &page) == false)
      return false;

    // Gọi hàm bắt đầu chơi với trang mới tạo.
    return start(page_number, &page);
  }

  // Hàm bắt đầu chơi một trang hành động dựa trên tên trang.
  bool ActionModule::start(std::string page_name)
  {
    int index;
    action_file_define::Page page;

    // Tìm trang dựa trên tên trang.
    for (index = 1; index < action_file_define::MAXNUM_PAGE; index++)
    {
      if (loadPage(index, &page) == false)
        return false;

      if (strcmp(page_name.c_str(), (char *)page.header.name) == 0)
        break;
    }

    // Kiểm tra xem trang có tồn tại không.
    if (index == action_file_define::MAXNUM_PAGE)
    {
      std::string status_msg = "Cannot play page. (" + page_name + " is an invalid name)\n";
      ROS_ERROR_STREAM(status_msg);
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
      return false;
    }
    else
      return start(index, &page); // Gọi hàm bắt đầu chơi với số trang được tìm thấy.
  }

  // Hàm bắt đầu chơi một trang hành động dựa trên số trang và dữ liệu trang được chỉ định.
  bool ActionModule::start(int page_number, action_file_define::Page *page)
  {
    // Kiểm tra tính hợp lệ của mô-đun.
    if (enable_ == false)
    {
      std::string status_msg = "Action Module is disabled";
      ROS_ERROR_STREAM(status_msg);
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
      return false;
    }

    // Kiểm tra xem có đang chơi trang nào khác không.
    if (playing_ == true)
    {
      std::string status_msg = "Cannot play page " + convertIntToString(page_number) + ". (Now playing)";
      ROS_ERROR_STREAM(status_msg);
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
      return false;
    }

    // Gán trang chơi là trang được chọn.
    play_page_ = *page;

    // Kiểm tra xem trang có thuộc tính lặp lại hay số bước hành động là 0 không.
    if (play_page_.header.repeat == 0 || play_page_.header.stepnum == 0)
    {
      std::string status_msg = "Page " + convertIntToString(page_number) + " has no action\n";
      ROS_ERROR_STREAM(status_msg);
      publishStatusMsg(robotis_controller_msgs::StatusMsg::STATUS_ERROR, status_msg);
      return false;
    }

    // Cập nhật thông tin chơi.
    play_page_idx_ = page_number;
    first_driving_start_ = true;
    playing_ = true;

    return true;
  }

  // Hàm giảm tốc độ chơi và dừng ngay lập tức.
  void ActionModule::brake()
  {
    playing_ = false;
  }

  // Hàm kiểm tra xem mô-đun có đang chơi hay không.
  bool ActionModule::isRunning(int *playing_page_num, int *playing_step_num)
  {
    // Nếu có tham số truyền vào, cập nhật giá trị của chúng.
    if (playing_page_num != 0)
      *playing_page_num = play_page_idx_;

    if (playing_step_num != 0)
      *playing_step_num = page_step_count_ - 1;

    // Gọi hàm kiểm tra xem mô-đun đang chơi hay không.
    return isRunning();
  }

  // Hàm nạp dữ liệu trang từ tệp hành động vào dữ liệu trang được chỉ định.
  bool ActionModule::loadPage(int page_number, action_file_define::Page *page)
  {
    // Kiểm tra tính hợp lệ của số trang.
    if (page_number < 0 || page_number >= action_file_define::MAXNUM_PAGE)
      return false;

    // Đặt vị trí đọc từ tệp hành động.
    long position = (long)(sizeof(action_file_define::Page) * page_number);

    // Di chuyển con trỏ đọc đến vị trí đã đặt.
    if (fseek(action_file_, position, SEEK_SET) != 0)
      return false;

    // Đọc dữ liệu trang từ tệp.
    if (fread(page, 1, sizeof(action_file_define::Page), action_file_) != sizeof(action_file_define::Page))
      return false;

    // Kiểm tra tính toàn vẹn của dữ liệu và reset nếu cần.
    if (verifyChecksum(page) == false)
      resetPage(page);

    return true;
  }

  // Hàm lưu trang với số trang và dữ liệu trang được chỉ định.
  bool ActionModule::savePage(int page_number, action_file_define::Page *page)
  {
    // Đặt vị trí ghi từ tệp hành động.
    long position = (long)(sizeof(action_file_define::Page) * page_number);

    // Kiểm tra và đặt checksum nếu cần.
    if (verifyChecksum(page) == false)
      setChecksum(page);

    // Di chuyển con trỏ ghi đến vị trí đã đặt.
    if (fseek(action_file_, position, SEEK_SET) != 0)
      return false;

    // Ghi dữ liệu trang vào tệp hành động.
    if (fwrite(page, 1, sizeof(action_file_define::Page), action_file_) != sizeof(action_file_define::Page))
      return false;

    return true;
  }

  // Hàm đặt lại trang với dữ liệu trang được chỉ định.
  void ActionModule::resetPage(action_file_define::Page *page)
  {
    // Thiết lập tất cả byte trong trang về giá trị 0.
    unsigned char *pt = (unsigned char *)page;
    for (unsigned int i = 0; i < sizeof(action_file_define::Page); i++)
    {
      *pt = 0x00;
      pt++;
    }

    // Thiết lập các giá trị mặc định cho các trường trong phần Header của trang.
    page->header.schedule = action_file_define::TIME_BASE_SCHEDULE; // Giả sử là thời gian làm cơ sở.
    page->header.repeat = 1;
    page->header.speed = 32;
    page->header.accel = 32;

    // Thiết lập giá trị mặc định cho các hằng số PGain của các động cơ trong Header.
    for (int i = 0; i < action_file_define::MAXNUM_JOINTS; i++)
      page->header.pgain[i] = 0x55;

    // Thiết lập các giá trị mặc định cho các bước trong trang.
    for (int i = 0; i < action_file_define::MAXNUM_STEP; i++)
    {
      // Thiết lập tất cả giá trị vị trí của các động cơ trong mỗi bước là giá trị không hợp lệ.
      for (int j = 0; j < action_file_define::MAXNUM_JOINTS; j++)
        page->step[i].position[j] = action_file_define::INVALID_BIT_MASK;

      // Thiết lập giá trị mặc định cho thời gian pause và thời gian thực hiện trong mỗi bước.
      page->step[i].pause = 0;
      page->step[i].time = 0;
    }

    // Thiết lập checksum cho trang.
    setChecksum(page);
  }

  // Hàm kích hoạt tất cả các động cơ trong danh sách động cơ có thể chạy.
  void ActionModule::enableAllJoints()
  {
    for (std::map<std::string, bool>::iterator it = action_joints_enable_.begin(); it != action_joints_enable_.end();
         it++)
    {
      it->second = true;
    }
  }

  void ActionModule::actionPlayProcess(std::map<std::string, robotis_framework::Dynamixel *> dxls)
  {
    // Biến cục bộ
    uint8_t id;
    uint32_t total_time_256t;                     // Tổng thời gian trong đơn vị thời gian cơ bản (256ms)
    uint32_t pre_section_time_256t;               // Thời gian tại phần tiền đề
    uint32_t main_time_256t;                      // Thời gian tại phần chính
    int32_t start_speed1024_pre_time_256t;        // Tốc độ ban đầu tại phần tiền đề
    int32_t moving_angle_speed1024_scale_256t_2t; // Góc di chuyển nhân với tốc độ, được chia cho 2t (đơn vị thời gian cơ bản)
    int32_t divider1, divider2;                   // Bộ chia

    int16_t max_angle;
    int16_t max_speed;
    int16_t tmp;
    int16_t prev_target_angle; // Góc bắt đầu
    int16_t curr_target_angle; // Góc đích
    int16_t next_target_angle; // Góc đích tiếp theo
    uint8_t direction_changed; // Hướng di chuyển
    int16_t speed_n;           // Tốc độ

    // Biến tĩnh
    static uint16_t start_angle[action_file_define::MAXNUM_JOINTS];   // Góc bắt đầu của quá trình nội suy
    static uint16_t target_angle[action_file_define::MAXNUM_JOINTS];  // Góc đích của quá trình nội suy
    static int16_t moving_angle[action_file_define::MAXNUM_JOINTS];   // Tổng góc di chuyển
    static int16_t main_angle[action_file_define::MAXNUM_JOINTS];     // Góc di chuyển ở phần Tốc độ Constant
    static int16_t accel_angle[action_file_define::MAXNUM_JOINTS];    // Góc di chuyển ở phần Tăng tốc
    static int16_t main_speed[action_file_define::MAXNUM_JOINTS];     // Tốc độ constant của mục tiêu
    static int16_t last_out_speed[action_file_define::MAXNUM_JOINTS]; // Tốc độ của Trạng thái Trước đó
    static int16_t goal_speed[action_file_define::MAXNUM_JOINTS];     // Tốc độ mục tiêu
    static uint8_t finish_type[action_file_define::MAXNUM_JOINTS];    // Trạng thái mong muốn tại góc đích

    static uint16_t unit_time_count;     // Đếm thời gian trong đơn vị thời gian cơ bản
    static uint16_t unit_time_num;       // Số lượng đơn vị thời gian trong một phần (Step)
    static uint16_t pause_time;          // Thời gian nghỉ
    static uint16_t unit_time_total_num; // Tổng số lượng đơn vị thời gian
    static uint16_t accel_step;          // Bước tăng tốc
    static uint8_t section;              // Phần của trang đang chơi
    static uint8_t play_repeat_count;    // Số lần lặp khi chơi
    static uint16_t next_play_page;      // Trang tiếp theo sẽ chơi

    /////////////// Const Variable
    /**************************************
     * Section             /----\
     *                    /|    |\
     *        /+---------/ |    | \
     *       / |        |  |    |  \
     * -----/  |        |  |    |   \----
     *      PRE  MAIN   PRE MAIN POST PAUSE
     ***************************************/

    if (playing_ == false)
    {
      for (std::map<std::string, robotis_framework::Dynamixel *>::iterator dxls_it = dxls.begin(); dxls_it != dxls.end();
           dxls_it++)
      {
        std::string joint_name = dxls_it->first;

        std::map<std::string, robotis_framework::DynamixelState *>::iterator result_it = action_result_.find(joint_name);
        if (result_it == result_.end())
          continue;
        else
        {
          result_it->second->goal_position_ = dxls_it->second->dxl_state_->goal_position_;
        }
      }
      return;
    }

    if (first_driving_start_ == true) // First start
    {
      first_driving_start_ = false; // First Process end
      playing_finished_ = false;
      stop_playing_ = false;
      unit_time_count = 0;
      unit_time_num = 0;
      pause_time = 0;
      section = PAUSE_SECTION;
      page_step_count_ = 0;
      play_repeat_count = play_page_.header.repeat;
      next_play_page = 0;

      for (unsigned int joint_index = 0; joint_index < action_file_define::MAXNUM_JOINTS; joint_index++)
      {
        id = joint_index;
        std::string joint_name = "";

        std::map<int, std::string>::iterator id_to_name_it = joint_id_to_name_.find(id);
        if (id_to_name_it == joint_id_to_name_.end())
          continue;
        else
          joint_name = id_to_name_it->second;

        std::map<std::string, robotis_framework::Dynamixel *>::iterator dxls_it = dxls.find(joint_name);
        if (dxls_it == dxls.end())
          continue;
        else
        {
          double goal_joint_angle_rad = dxls_it->second->dxl_state_->goal_position_;
          target_angle[id] = convertRadTow4095(goal_joint_angle_rad);
          last_out_speed[id] = 0;
          moving_angle[id] = 0;
          goal_speed[id] = 0;
        }
      }
    }

    if (unit_time_count < unit_time_num) // Quá trình đang diễn ra
    {
      unit_time_count++;
      if (section == PAUSE_SECTION)
      {
        // Phần tạm dừng, không có thao tác nào.
      }
      else
      {
        // Lặp qua từng động cơ để cập nhật goal_position_ tương ứng.
        for (unsigned int joint_index = 0; joint_index < action_file_define::MAXNUM_JOINTS; joint_index++)
        {
          id = joint_index;
          std::string joint_name = "";

          // Lấy tên của động cơ từ id.
          std::map<int, std::string>::iterator id_to_name_it = joint_id_to_name_.find(id);
          if (id_to_name_it == joint_id_to_name_.end())
            continue;
          else
            joint_name = id_to_name_it->second;

          // Tìm động cơ trong danh sách.
          std::map<std::string, robotis_framework::Dynamixel *>::iterator dxls_it = dxls.find(joint_name);
          if (dxls_it == dxls.end())
          {
            continue;
          }
          else
          {
            if (moving_angle[id] == 0)
            {
              // Nếu động cơ không chuyển động, giữ nguyên góc.
              action_result_[joint_name]->goal_position_ = convertw4095ToRad(start_angle[id]);
            }
            else
            {
              if (section == PRE_SECTION)
              {
                // Phần PRE_SECTION, tính toán góc mới dựa trên tốc độ và gia tốc.
                speed_n = (short)(((long)(main_speed[id] - last_out_speed[id]) * unit_time_count) / unit_time_num);
                goal_speed[id] = last_out_speed[id] + speed_n;
                accel_angle[id] = (short)((((long)(last_out_speed[id] + (speed_n >> 1)) * unit_time_count * 144) / 15) >> 9);

                action_result_[joint_name]->goal_position_ = convertw4095ToRad(start_angle[id] + accel_angle[id]);
              }
              else if (section == MAIN_SECTION)
              {
                // Phần MAIN_SECTION, tính toán góc mới dựa trên tốc độ chuyển động ổn định.
                action_result_[joint_name]->goal_position_ = convertw4095ToRad(
                    start_angle[id] + (short int)(((long)(main_angle[id]) * unit_time_count) / unit_time_num));

                goal_speed[id] = main_speed[id];
              }
              else // POST_SECTION
              {
                if (unit_time_count == (unit_time_num - 1))
                {
                  // Sử dụng góc đích để giảm thiểu sai số ở bước cuối cùng.
                  action_result_[joint_name]->goal_position_ = convertw4095ToRad(target_angle[id]);
                }
                else
                {
                  if (finish_type[id] == ZERO_FINISH)
                  {
                    // Phần kết thúc với góc đích 0, tính toán góc mới dựa trên tốc độ và gia tốc.
                    speed_n = (short int)(((long)(0 - last_out_speed[id]) * unit_time_count) / unit_time_num);
                    goal_speed[id] = last_out_speed[id] + speed_n;

                    action_result_[joint_name]->goal_position_ =
                        convertw4095ToRad(
                            start_angle[id] + (short)((((long)(last_out_speed[id] + (speed_n >> 1)) * unit_time_count * 144) / 15) >> 9));
                  }
                  else // NONE_ZERO_FINISH
                  {
                    // Tương tự như MAIN Section vì một số động cơ cần xoay, những cái khác không cần.
                    action_result_[joint_name]->goal_position_ = convertw4095ToRad(
                        start_angle[id] + (short int)(((long)(main_angle[id]) * unit_time_count) / unit_time_num));

                    goal_speed[id] = main_speed[id];
                  }
                }
              }
            }
          }
        }
      }
    }

    // Khi phần hiện tại đã hoàn thành
    else if (unit_time_count >= unit_time_num)
    {
      // Đặt lại bộ đếm thời gian đơn vị
      unit_time_count = 0;

      // Cập nhật thông tin cho từng động cơ
      for (unsigned int joint_index = 0; joint_index < action_file_define::MAXNUM_JOINTS; joint_index++)
      {
        id = joint_index;
        std::string joint_name = "";

        // Tìm tên của động cơ
        std::map<int, std::string>::iterator id_to_name_it = joint_id_to_name_.find(id);
        if (id_to_name_it == joint_id_to_name_.end())
          continue;
        else
          joint_name = id_to_name_it->second;

        // Tìm thông tin động cơ trong danh sách
        std::map<std::string, robotis_framework::Dynamixel *>::iterator dxls_it = dxls.find(joint_name);
        if (dxls_it == dxls.end())
          continue;
        else
        {
          // Lấy thông tin góc mục tiêu của động cơ
          double _goal_joint_angle_rad = dxls_it->second->dxl_state_->goal_position_;

          // Cập nhật giá trị khởi đầu và vận tốc cuối cùng
          start_angle[id] = convertRadTow4095(_goal_joint_angle_rad);
          last_out_speed[id] = goal_speed[id];
        }
      }

      // Cập nhật phần (PRE -> MAIN -> POST -> (PAUSE hoặc PRE) ...)
      if (section == PRE_SECTION)
      {
        // Chuẩn bị cho phần MAIN
        section = MAIN_SECTION;
        unit_time_num = unit_time_total_num - (accel_step << 1);

        // Cập nhật thông tin cho từng động cơ
        for (unsigned int joint_index = 0; joint_index < action_file_define::MAXNUM_JOINTS; joint_index++)
        {
          id = joint_index;

          // Nếu là NONE_ZERO_FINISH
          if (finish_type[id] == NONE_ZERO_FINISH)
          {
            // Nếu không có phần di chuyển ở tốc độ không đổi
            if ((unit_time_total_num - accel_step) == 0)
              main_angle[id] = 0;
            else
              main_angle[id] = (short)(((long)(moving_angle[id] - accel_angle[id]) * unit_time_num) /
                                       (unit_time_total_num - accel_step));
          }
          else // ZERO_FINISH
          {
            main_angle[id] = moving_angle[id] - accel_angle[id] -
                             (short int)((((long)main_speed[id] * accel_step * 12) / 5) >> 8);
          }
        }
      }

      // Ngược lại, nếu phần hiện tại là MAIN_SECTION
      else if (section == MAIN_SECTION)
      {
        // Chuẩn bị cho phần POST Section
        section = POST_SECTION;
        unit_time_num = accel_step;

        // Cập nhật thông tin cho từng động cơ
        for (unsigned int joint_index = 0; joint_index < action_file_define::MAXNUM_JOINTS; joint_index++)
        {
          id = joint_index;
          main_angle[id] = moving_angle[id] - main_angle[id] - accel_angle[id];
        }
      }
      // Nếu đang ở phần POST_SECTION
      else if (section == POST_SECTION)
      {
        // Quyết định dựa trên việc có tồn tại thời gian tạm dừng hay không
        if (pause_time)
        {
          section = PAUSE_SECTION;
          unit_time_num = pause_time;
        }
        else
        {
          section = PRE_SECTION;
        }
      }
      // Nếu đang ở phần PAUSE_SECTION
      else if (section == PAUSE_SECTION)
      {
        // Chuẩn bị cho phần PRE Section
        section = PRE_SECTION;

        // Cập nhật thông tin cho từng động cơ
        for (unsigned int joint_index = 0; joint_index < action_file_define::MAXNUM_JOINTS; joint_index++)
        {
          id = joint_index;
          last_out_speed[id] = 0;
        }
      }

      // Chuẩn bị cho tất cả trong phần PRE Section
      if (section == PRE_SECTION)
      {
        // Nếu chuyển động đã kết thúc
        if (playing_finished_ == true)
        {
          playing_ = false;
          return;
        }

        page_step_count_++;

        // Nếu chuyển động của trang hiện tại đã kết thúc
        if (page_step_count_ > play_page_.header.stepnum)
        {
          // Sao chép trang tiếp theo
          play_page_ = next_play_page_;
          if (play_page_idx_ != next_play_page)
            play_repeat_count = play_page_.header.repeat;
          page_step_count_ = 1;
          play_page_idx_ = next_play_page;
        }

        // Nếu đây là bước cuối cùng
        if (page_step_count_ == play_page_.header.stepnum)
        {
          // Nếu có lệnh dừng
          if (stop_playing_ == true)
          {
            next_play_page = play_page_.header.exit; // Điều hướng đến trang Exit
          }
          else
          {
            play_repeat_count--;
            // Nếu còn lượt lặp lại
            if (play_repeat_count > 0)
              next_play_page = play_page_idx_; // Đặt trang tiếp theo thành trang hiện tại
            else
              next_play_page = play_page_.header.next; // Đặt trang tiếp theo

            // Nếu không có trang tiếp theo, chuyển động sẽ kết thúc sau bước hiện tại
            if (next_play_page == 0)
              playing_finished_ = true;
            else
            {
              // Nếu trang hiện tại không phải là trang tiếp theo
              if (play_page_idx_ != next_play_page)
                loadPage(next_play_page, &next_play_page_);
              else
                next_play_page_ = play_page_;

              // Nếu không có thông tin chuyển động, chuyển động sẽ kết thúc sau bước hiện tại
              if (next_play_page_.header.repeat == 0 || next_play_page_.header.stepnum == 0)
                playing_finished_ = true;
            }
          }
        }

        //////// Tính toán Tham số Bước
        pause_time = (((unsigned short)play_page_.step[page_step_count_ - 1].pause) << 5) / play_page_.header.speed;
        max_speed = ((unsigned short)play_page_.step[page_step_count_ - 1].time * (unsigned short)play_page_.header.speed) >> 5;
        if (max_speed == 0)
          max_speed = 1;
        max_angle = 0;

        //////// Tính toán tham số của từng Khớp
        for (unsigned int joint_index = 0; joint_index < action_file_define::MAXNUM_JOINTS; joint_index++)
        {
          id = joint_index;
          // Tính toán quỹ đạo sử dụng dữ liệu trước, hiện tại và tương lai
          accel_angle[id] = 0;

          // Tìm góc mục tiêu hiện tại
          if (play_page_.step[page_step_count_ - 1].position[id] & action_file_define::INVALID_BIT_MASK)
            curr_target_angle = target_angle[id];
          else
            curr_target_angle = play_page_.step[page_step_count_ - 1].position[id];

          // Cập nhật start, prev_target, curr_target
          start_angle[id] = target_angle[id];
          prev_target_angle = target_angle[id];
          target_angle[id] = curr_target_angle;

          // Tìm sự chuyển động (Moving offset)
          moving_angle[id] = (int)(target_angle[id] - start_angle[id]);

          // Tìm Góc mục tiêu tiếp theo
          if (page_step_count_ == play_page_.header.stepnum) // Nếu bước hiện tại là bước cuối cùng
          {
            if (playing_finished_ == true) // Nếu sẽ kết thúc
              next_target_angle = curr_target_angle;
            else
            {
              if (next_play_page_.step[0].position[id] & action_file_define::INVALID_BIT_MASK)
                next_target_angle = curr_target_angle;
              else
                next_target_angle = next_play_page_.step[0].position[id];
            }
          }
          else
          {
            if (play_page_.step[page_step_count_].position[id] & action_file_define::INVALID_BIT_MASK)
              next_target_angle = curr_target_angle;
            else
              next_target_angle = play_page_.step[page_step_count_].position[id];
          }

          // Xác định sự thay đổi hướng
          if (((prev_target_angle < curr_target_angle) && (curr_target_angle < next_target_angle)) ||
              ((prev_target_angle > curr_target_angle) && (curr_target_angle > next_target_angle)))
          {
            // Cùng hướng
            direction_changed = 0;
          }
          else
          {
            direction_changed = 1;
          }

          // Xác định loại kết thúc
          if (direction_changed || pause_time || playing_finished_ == true)
            finish_type[id] = ZERO_FINISH;
          else
            finish_type[id] = NONE_ZERO_FINISH;

          // Cập nhật MaxAngle1024 nếu lịch trình dựa trên tốc độ (SPEED_BASE_SCHEDULE)
          if (play_page_.header.schedule == action_file_define::SPEED_BASE_SCHEDULE)
          {
            // Cập nhật MaxAngle1024
            if (moving_angle[id] < 0)
              tmp = -moving_angle[id];
            else
              tmp = moving_angle[id];

            if (tmp > max_angle)
              max_angle = tmp;
          }
        }

        // Tính thời gian. Sau đó, thời gian tính toán sẽ được chia cho 7.8msec (<< 7) - tính toán có bao nhiêu lần 7.8msec
        // Sau khi chuyển đơn vị, tính góc/ tốc độ, và mã nguồn sau tính toán có bao nhiêu đơn vị 7.8s xuất hiện trong thời gian cụ thể
        // Chuyển đổi đơn vị --- góc: 1024->300độ, tốc độ: 256 ->720
        // wUnitTimeNum = ((wMaxAngle1024*300/1024) /(wMaxSpeed256 * 720/256)) /7.8msec;
        //             = ((128*wMaxAngle1024*300/1024) /(wMaxSpeed256 * 720/256)) ;    (/7.8msec == *128)
        //             = (wMaxAngle1024*40) /(wMaxSpeed256 *3);
        if (play_page_.header.schedule == action_file_define::TIME_BASE_SCHEDULE)
          unit_time_total_num = max_speed; // TIME BASE 051025
        else
          unit_time_total_num = (max_angle * 40) / (max_speed * 3);

        accel_step = play_page_.header.accel;
        if (unit_time_total_num <= (accel_step << 1))
        {
          if (unit_time_total_num == 0)
          {
            accel_step = 0;
          }
          else
          {
            accel_step = (unit_time_total_num - 1) >> 1;
            if (accel_step == 0)
              unit_time_total_num = 0; // Bước tăng tốc và bước chuyển động ổn định phải lớn hơn một để có thể di chuyển
          }
        }

        // Tính thời gian tổng cộng cho 256t (đơn vị thời gian có thể)
        total_time_256t = ((unsigned long)unit_time_total_num) << 1; // /128 * 256

        // Tính thời gian cho phần tiền định theo đơn vị 256t
        pre_section_time_256t = ((unsigned long)accel_step) << 1; // /128 * 256

        // Tính thời gian chính cho 256t
        main_time_256t = total_time_256t - pre_section_time_256t;

        // Tính các giá trị chia để sử dụng sau này
        divider1 = pre_section_time_256t + (main_time_256t << 1);
        divider2 = (main_time_256t << 1);

        // Đảm bảo rằng các giá trị chia không bằng 0 để tránh phép chia cho 0 sau này
        if (divider1 == 0)
          divider1 = 1;

        if (divider2 == 0)
          divider2 = 1;

        // Lặp qua các khớp (giả sử là điều khiển các khớp nào đó)
        for (unsigned int joint_index = 0; joint_index < action_file_define::MAXNUM_JOINTS; joint_index++)
        {
          // Đặt ID cho khớp
          id = joint_index;

          // Tính tốc độ ban đầu trong đơn vị 1024 trên đơn vị thời gian (256t)
          start_speed1024_pre_time_256t = (long)last_out_speed[id] * pre_section_time_256t; //  *300/1024 * 1024/720 * 256 * 2

          // Tính giá trị có tỷ lệ cho góc di chuyển theo đơn vị 256t^2
          moving_angle_speed1024_scale_256t_2t = (((long)moving_angle[id]) * 2560L) / 12;

          // Tính tốc độ chính dựa trên loại kết thúc
          if (finish_type[id] == ZERO_FINISH)
            main_speed[id] = (short int)((moving_angle_speed1024_scale_256t_2t - start_speed1024_pre_time_256t) / divider2);
          else
            main_speed[id] = (short int)((moving_angle_speed1024_scale_256t_2t - start_speed1024_pre_time_256t) / divider1);

          // Đảm bảo rằng tốc độ chính nằm trong giới hạn
          if (main_speed[id] > 1023)
            main_speed[id] = 1023;

          if (main_speed[id] < -1023)
            main_speed[id] = -1023;
        }

        // Đặt unit_time_num bằng accel_step, có thể đại diện cho thời gian phần tiền định
        unit_time_num = accel_step; // Phần Tiền Định
      }
    }
  }

  // Hàm này xuất bản một thông điệp trạng thái với các thông số được cung cấp
  void ActionModule::publishStatusMsg(unsigned int type, std::string msg)
  {
    // Tạo một đối tượng StatusMsg từ gói tin của robotis_controller_msgs
    robotis_controller_msgs::StatusMsg status;

    // Đặt thời gian (timestamp) của thông điệp là thời gian hiện tại
    status.header.stamp = ros::Time::now();

    // Thiết lập loại thông điệp trạng thái
    status.type = type;

    // Thiết lập tên của module, ở đây là "Action"
    status.module_name = "Action";

    // Thiết lập nội dung của thông điệp trạng thái
    status.status_msg = msg;

    // Xuất bản thông điệp trạng thái thông qua publisher đã được khai báo trước đó
    status_msg_pub_.publish(status);
  }

  // Hàm này xuất bản một thông điệp kết quả (done message) với nội dung được cung cấp
  void ActionModule::publishDoneMsg(std::string msg)
  {
    // Tạo một đối tượng String từ gói tin của std_msgs
    std_msgs::String done_msg;

    // Thiết lập dữ liệu của thông điệp kết quả
    done_msg.data = msg;

    // Xuất bản thông điệp kết quả thông qua publisher đã được khai báo trước đó
    done_msg_pub_.publish(done_msg);
  }

}

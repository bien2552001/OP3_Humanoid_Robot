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

#ifndef OP3_DEMO_MAIN_WINDOW_H
#define OP3_DEMO_MAIN_WINDOW_H

/*****************************************************************************
 ** Includes
 *****************************************************************************/
#ifndef Q_MOC_RUN

// #include <QtGui/QMainWindow>
#include <QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

#endif
/*****************************************************************************
 ** Namespace
 *****************************************************************************/

namespace robotis_op
{

#define DEGREE2RADIAN (M_PI / 180.0)
#define RADIAN2DEGREE (180.0 / M_PI)

  /*****************************************************************************
   ** Interface [MainWindow]
   *****************************************************************************/
  /**
   * @brief Qt central, all operations relating to the view part here.
   */
  class MainWindow : public QMainWindow
  {
    Q_OBJECT

  public:
    // Constructor của lớp MainWindow, có tham số để xử lý dòng lệnh và một tham số con trỏ đến widget cha.
    MainWindow(int argc, char **argv, QWidget *parent = 0);

    // Destructor của lớp MainWindow.
    ~MainWindow();

    // Phương thức để đọc các thiết lập của chương trình Qt khi chương trình được khởi động.
    void readSettings();

    // Phương thức để lưu các thiết lập của chương trình Qt khi chương trình được đóng.
    void writeSettings();

    // Phương thức nạp chồng (overloaded) để xử lý sự kiện đóng cửa sổ.
    void closeEvent(QCloseEvent *event);

    // Phương thức để hiển thị thông báo khi không có master (chủ) được xác định.
    void showNoMasterMessage();

  public Q_SLOTS:

    /******************************************
     ** Auto-connections (connectSlotsByName())
     *******************************************/
    // Hàm này được tự động kết nối thông qua auto-connection khi action "About" được kích hoạt.
    void on_actionAbout_triggered();

    // Hàm này được tự động kết nối thông qua auto-connection khi nút "Clear Log" được nhấn,
    // và giá trị boolean check được truyền vào.
    void on_button_clear_log_clicked(bool check);

    // Hàm này được tự động kết nối thông qua auto-connection khi nút "Init Pose" được nhấn,
    // và giá trị boolean check được truyền vào.
    void on_button_init_pose_clicked(bool check);

    // Walking

    // Phương thức này được gọi khi nút "Init Gyro" được nhấn.
    void on_button_init_gyro_clicked(bool check);

    // Phương thức này được gọi khi nút "Walking Start" được nhấn.
    void on_button_walking_start_clicked(bool check);

    // Phương thức này được gọi khi nút "Walking Stop" được nhấn.
    void on_button_walking_stop_clicked(bool check);

    // Parameter Management

    // Phương thức này được gọi khi nút "Param Refresh" được nhấn.
    void on_button_param_refresh_clicked(bool check);

    // Phương thức này được gọi khi nút "Param Apply" được nhấn.
    void on_button_param_apply_clicked(bool check);

    // Phương thức này được gọi khi nút "Param Save" được nhấn.
    void on_button_param_save_clicked(bool check);

    // Balance Control

    // Phương thức này được gọi khi ô kiểm "Balance On" được nhấn.
    void on_checkBox_balance_on_clicked(bool check);

    // Phương thức này được gọi khi ô kiểm "Balance Off" được nhấn.
    void on_checkBox_balance_off_clicked(bool check);

    // Head Control

    // Phương thức này được gọi khi nút "Head Center" được nhấn.
    void on_head_center_button_clicked(bool check);

    // Demo

    // Phương thức này được gọi khi nút "Demo Start" được nhấn.
    void on_button_demo_start_clicked(bool check);

    // Phương thức này được gọi khi nút "Demo Stop" được nhấn.
    void on_button_demo_stop_clicked(bool check);

    // Phương thức này được gọi khi nút "Right Kick" được nhấn.
    void on_button_r_kick_clicked(bool check);

    // Phương thức này được gọi khi nút "Left Kick" được nhấn.
    void on_button_l_kick_clicked(bool check);

    // Phương thức này được gọi khi nút "Get Up Front" được nhấn.
    void on_button_getup_front_clicked(bool check);

    // Phương thức này được gọi khi nút "Get Up Back" được nhấn.
    void on_button_getup_back_clicked(bool check);

    /******************************************
     ** Manual connections
     *******************************************/

    // Phương thức này được gọi để cập nhật giao diện xem logging.
    // Lưu ý: không rõ tại sao phải kết nối thủ công, có thể do một số vấn đề đặc biệt trong việc kết nối tự động.
    void updateLoggingView();

    // Phương thức này được gọi khi cần thiết lập một chế độ, có thể được kết nối bằng cách thủ công.
    void setMode(bool check);

    // Phương thức này được gọi khi cần cập nhật chế độ hiện tại của các khớp cụ thể.
    // Tham số `mode` là một vector chứa các giá trị chế độ cho các khớp.
    void updateCurrentJointMode(std::vector<int> mode);

    // Phương thức này được gọi khi cần thiết lập một chế độ, nhưng truyền vào một chuỗi `mode_name`.
    void setMode(QString mode_name);

    // Head Control
    // Phương thức này được gọi để cập nhật góc đầu, với các tham số là góc pan (xoay ngang) và tilt (nghiêng).

    void updateHeadAngles(double pan, double tilt);

    // Walking
    // Phương thức này được gọi để cập nhật các tham số đi bộ, với tham số là một đối tượng WalkingParam.
    void updateWalkingParams(op3_walking_module_msgs::WalkingParam params);

    void walkingCommandShortcut();
    // Phương thức này có thể được gọi để tạo lối tắt cho các lệnh đi bộ.

  protected Q_SLOTS:
    // Phương thức được bảo vệ (protected slot) được gọi để thiết lập góc đầu.
    void setHeadAngle();

// Một enum (kiểu liệt kê) có tên là Motion_Index được định nghĩa để chứa các giá trị được liệt kê.
// Các giá trị này có thể được sử dụng để định danh các chế độ chuyển động khác nhau.
private:
  enum Motion_Index
  {
    InitPose = 1,        // Chế độ chuyển động khởi tạo
    WalkingReady = 9,    // Chế độ chuyển động sẵn sàng đi bộ
    GetUpFront = 122,    // Chế độ chuyển động đứng dậy với hướng lên trước
    GetUpBack = 123,     // Chế độ chuyển động đứng dậy với hướng lên sau
    RightKick = 121,     // Chế độ chuyển động đá bóng bằng chân phải
    LeftKick = 120,      // Chế độ chuyển động đá bóng bằng chân trái
    Ceremony = 85,       // Chế độ chuyển động lễ kỳ
  };



    
    // Phương thức này được gọi để thiết lập lối tắt người dùng. Có thể liên quan đến việc đặt các lệnh hoặc chức năng dựa trên tùy chọn người dùng.
    void setUserShortcut();

    // Phương thức này được gọi để khởi tạo đơn vị chế độ. Có thể liên quan đến việc thiết lập các chế độ hoạt động cho ứng dụng.
    void initModeUnit();

    // Phương thức này được gọi để khởi tạo đơn vị chuyển động. Có thể liên quan đến việc khởi tạo các phương pháp hoặc thông số cho chuyển động.
    void initMotionUnit();

    // Phương thức này được gọi để cập nhật giao diện người dùng của mô-đun. Có thể liên quan đến việc hiển thị thông tin mới hoặc cập nhật trạng thái trực quan.
    void updateModuleUI();

    // Phương thức này được gọi để thiết lập góc đầu, với các tham số là góc pan (xoay ngang) và tilt (nghiêng). Có thể sử dụng để kiểm soát góc đầu của robot hoặc đối tượng khác.
    void setHeadAngle(double pan, double tilt);

    // Phương thức này được gọi để áp dụng các tham số đi bộ đã cập nhật. Có thể liên quan đến việc thiết lập các thông số đi bộ như tốc độ, bước chân, v.v.
    void applyWalkingParams();





    // Đối tượng ui_ thuộc lớp MainWindowDesign được sử dụng để truy cập các thành phần giao diện của MainWindow.
    Ui::MainWindowDesign ui_;

    // Đối tượng qnode_op3_ thuộc lớp QNodeOP3 được sử dụng để quản lý các kết nối và tương tác với nút robot OP3.
    QNodeOP3 qnode_op3_;

    // Biến debug_ dùng để kiểm soát chế độ debug trong ứng dụng. Có thể được sử dụng để bật hoặc tắt các thông báo gỡ lỗi hoặc chế độ debug khác.
    bool debug_;

    // Biến is_updating_ biểu thị trạng thái của quá trình cập nhật. Có thể được sử dụng để kiểm soát xem ứng dụng đang trong quá trình cập nhật hay không.
    bool is_updating_;

    // Biến is_walking_ biểu thị trạng thái của quá trình đi bộ. Có thể được sử dụng để kiểm soát xem robot đang trong quá trình đi bộ hay không.
    bool is_walking_;

    // Biến module_ui_table_ là một bảng ánh xạ (map) sử dụng chuỗi làm khóa để ánh xạ đến danh sách các đối tượng QWidget. Có thể được sử dụng để quản lý giao diện người dùng của các mô-đun khác nhau trong ứng dụng.
    std::map<std::string, QList<QWidget *>> module_ui_table_;




  };

} // namespace robotis_op

#endif // OP3_DEMO_MAIN_WINDOW_H

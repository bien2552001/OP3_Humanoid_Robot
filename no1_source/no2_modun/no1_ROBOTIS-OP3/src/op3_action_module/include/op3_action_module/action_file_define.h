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

#ifndef ACTION_FILE_DEFINE_H_
#define ACTION_FILE_DEFINE_H_

namespace robotis_op
{
namespace action_file_define
{

const int MAXNUM_PAGE   = 256;       // Số trang tối đa
const int MAXNUM_STEP   = 7;         // Số bước tối đa trong mỗi trang
const int MAXNUM_NAME   = 13;        // Số ký tự tối đa cho tên trang
const int MAXNUM_JOINTS = 31;        // Số lượng khớp tối đa

const int SPEED_BASE_SCHEDULE = 0;   // Lịch trình dựa trên tốc độ
const int TIME_BASE_SCHEDULE  = 0x0a; // Lịch trình dựa trên thời gian

const int INVALID_BIT_MASK    = 0x4000; // Bit mask cho trạng thái không hợp lệ
const int TORQUE_OFF_BIT_MASK = 0x2000; // Bit mask để tắt moment

typedef struct // Cấu trúc tiêu đề trang (tổng cộng 64 unsigned char)
{
  unsigned char name[MAXNUM_NAME+1];  // Tên             0~13
  unsigned char reserved1;            // Đã dự trữ        14
  unsigned char repeat;               // Số lần lặp      15
  unsigned char schedule;             // Lịch trình       16
  unsigned char reserved2[3];         // Đã dự trữ        17~19
  unsigned char stepnum;              // Số bước         20
  unsigned char reserved3;            // Đã dự trữ        21
  unsigned char speed;                // Tốc độ          22
  unsigned char reserved4;            // Đã dự trữ        23
  unsigned char accel;                // Thời gian gia tốc 24
  unsigned char next;                 // Liên kết đến bước tiếp theo 25
  unsigned char exit;                 // Liên kết đến bước thoát 26
  unsigned char reserved5[4];         // Đã dự trữ        27~30
  unsigned char checksum;             // Checksum         31
  unsigned char pgain[MAXNUM_JOINTS]; // pgain            32~62
  unsigned char reserved6;            // Đã dự trữ        63
} PageHeader;

typedef struct // Cấu trúc bước (tổng cộng 64 unsigned char)
{
  unsigned short position[MAXNUM_JOINTS]; // Vị trí của khớp   0~61
  unsigned char pause;                    // Thời gian tạm dừng 62
  unsigned char time;                     // Thời gian         63
} Step;

typedef struct // Cấu trúc trang (tổng cộng 512 unsigned char)
{
  PageHeader header;             // Tiêu đề trang  0~63
  Step       step[MAXNUM_STEP];   // Các bước trong trang  61~501
} Page;

}

}

#endif /* ACTION_FILE_DEFINE_H_ */

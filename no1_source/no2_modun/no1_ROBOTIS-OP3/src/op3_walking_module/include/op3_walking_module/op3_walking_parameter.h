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

#ifndef OP3_WALKING_PARAMETER_H_
#define OP3_WALKING_PARAMETER_H_

// Lớp định nghĩa các tham số thời gian cho việc đi bộ
class WalkingTimeParameter
{
public:
  // Enum định nghĩa các trạng thái của bước đi
  enum
  {
    PHASE0 = 0,
    PHASE1 = 1,
    PHASE2 = 2,
    PHASE3 = 3
  };

private:
  double periodtime;         // Thời gian một chu kỳ
  double dsp_ratio;          // Tỉ lệ thời gian đi qua giai đoạn chu kỳ đầu (DSP)
  double ssp_ratio;          // Tỉ lệ thời gian tiếp xúc chân trên mặt đất (SSP)
  double x_swap_periodtime;  // Thời gian chuyển động theo phương x khi đổi chân
  double x_move_periodtime;  // Thời gian chuyển động theo phương x khi không đổi chân
  double y_swap_periodtime;  // Thời gian chuyển động theo phương y khi đổi chân
  double y_move_periodtime;  // Thời gian chuyển động theo phương y khi không đổi chân
  double z_swap_periodtime;  // Thời gian chuyển động theo phương z khi đổi chân
  double z_move_periodtime;  // Thời gian chuyển động theo phương z khi không đổi chân
  double a_move_periodtime;  // Thời gian chuyển động theo góc quay khi không đổi chân
  double ssp_time;           // Thời gian tiếp xúc chân trên mặt đất
  double ssp_time_start_l;   // Thời gian bắt đầu tiếp xúc chân trái
  double ssp_time_end_l;     // Thời gian kết thúc tiếp xúc chân trái
  double ssp_time_start_r;   // Thời gian bắt đầu tiếp xúc chân phải
  double ssp_time_end_r;     // Thời gian kết thúc tiếp xúc chân phải
  double phase1_time;        // Thời gian giai đoạn 1
  double phase2_time;        // Thời gian giai đoạn 2
  double phase3_time;        // Thời gian giai đoạn 3
};

// Lớp chứa các tham số chuyển động
class WalkingMovementParameter
{
private:
  // Các tham số chuyển động
};

// Lớp chứa các tham số cân bằng
class WalkingBalanceParameter
{
  // Các tham số cân bằng
};


#endif /* OP3_WALKING_PARAMETER_H_ */

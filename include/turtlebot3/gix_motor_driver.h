/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
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

#ifndef GIX_MOTOR_DRIVER_H_
#define GIX_MOTOR_DRIVER_H_

#include <Dynamixel2Arduino.h>


enum GIXMotorLocation{
  GIX = 0,
  N_MOTORS
};

typedef struct
{
  int32_t value[N_MOTORS];
} gix_position_info_t;

typedef struct
{
  int32_t value[N_MOTORS];
} gix_velocity_info_t;

typedef struct
{
  int16_t value[N_MOTORS];
} gix_current_info_t;

typedef struct
{
  uint32_t value[N_MOTORS];
} gix_accel_info_t;


class GIXMotorDriver
{
 public:
  GIXMotorDriver(Dynamixel2Arduino &dxl_param);
  ~GIXMotorDriver();
  
  bool init(void);
  void close(void);

  bool is_connected();  

  bool set_torque(bool onoff);
  bool get_torque(void);

  bool read_goal_position(gix_position_info_t &position_info);
  bool read_present_position(gix_position_info_t &position_info);
  bool read_present_velocity(gix_velocity_info_t &velocity_info);
  bool read_present_current(gix_current_info_t &current_info);
  bool read_profile_acceleration(gix_accel_info_t &accel_info);
  bool read_profile_velocity(gix_accel_info_t &accel_info);
  bool read_goal_current(gix_current_info_t &current_info);

  bool write_goal_position_joint(gix_position_info_t &position_info);
  bool write_profile_acceleration_joint(gix_accel_info_t &accel_info);
  bool write_profile_velocity_joint(gix_accel_info_t &accel_info);
  bool write_goal_current_joint(gix_current_info_t &current_info);

 private:
  
  bool is_ready(void);

  uint8_t motor_id_[N_MOTORS];
  bool torque_;
  bool is_init_;
  bool is_connected_;
  Dynamixel2Arduino &dxl;
};

#endif // GIX_MOTOR_DRIVER_H_

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

#include "../../include/turtlebot3/gix_motor_driver.h"


/* DYNAMIXEL Information for controlling motors and  */
const uint8_t DXL_MOTOR_ID_GIX = 3; 

static ParamForSyncReadInst_t gix_sync_read_param;
static ParamForSyncWriteInst_t gix_sync_write_param;
static RecvInfoFromStatusInst_t gix_read_result;



GIXMotorDriver::GIXMotorDriver(Dynamixel2Arduino &dxl_param)
: dxl(dxl_param),
  torque_(false),
  is_init_(false),
  is_connected_(false)
{
  motor_id_[GIX] = DXL_MOTOR_ID_GIX;
}

GIXMotorDriver::~GIXMotorDriver()
{
  close();
}

bool GIXMotorDriver::init(void)
{
  gix_sync_write_param.id_count = N_MOTORS;
  for (int i=0; i<N_MOTORS; i++){
    gix_sync_write_param.xel[i].id = motor_id_[i];
  }

  gix_sync_read_param.addr = 132;
  gix_sync_read_param.length = 4;
  gix_sync_read_param.id_count = N_MOTORS;
  for (int i=0; i<N_MOTORS; i++){
    gix_sync_read_param.xel[i].id = motor_id_[i];    
  }

  // Enable Dynamixel Torque
  set_torque(true);

  is_init_ = true;

  return true;
}

bool GIXMotorDriver::is_connected()
{
  bool ret = true;

  if (is_init_ == false) return false;

  for (int i=0; i<N_MOTORS; i++) {
    if (dxl.ping(motor_id_[i]) == false) {
      ret = false;
      break;
    }
  }

  is_connected_ = ret;
  
  return ret;
}

bool GIXMotorDriver::is_ready(void)
{
  if (is_init_ == false) return false;
  if (is_connected_ == false) return false;

  return true;
}

bool GIXMotorDriver::set_torque(bool onoff)
{
  bool ret = false;
  
  if (is_ready() == false) return false;

  gix_sync_write_param.addr = 64;
  gix_sync_write_param.length = 1;
  gix_sync_write_param.id_count = N_MOTORS;

  for (int i=0; i<N_MOTORS; i++){
    gix_sync_write_param.xel[i].data[0] = onoff;
  }

  if(dxl.syncWrite(gix_sync_write_param) == true){
    ret = true;
    torque_ = onoff;
  }

  return ret;
}

bool GIXMotorDriver::get_torque()
{
  bool ret = true;

  if (is_ready() == false) return false;

  for (int i=0; i<N_MOTORS; i++) {
    if (dxl.readControlTableItem(ControlTableItem::TORQUE_ENABLE, motor_id_[i]) == false) {
      ret = false;
      break;
    }
  }

  torque_ = ret;

  return torque_;
}

void GIXMotorDriver::close(void)
{
  if (is_ready() == false) return;

  // Disable Dynamixel Torque
  set_torque(false);
}

bool GIXMotorDriver::read_goal_position(gix_position_info_t &position_info)
{
  bool ret = false;

  if (is_ready() == false) return false;

  gix_sync_read_param.addr = 116;
  gix_sync_read_param.length = 4;
  gix_sync_read_param.id_count = N_MOTORS;

  if(dxl.syncRead(gix_sync_read_param, gix_read_result)){
    for (int i=0; i<gix_sync_read_param.id_count; i++) {
      memcpy(&position_info.value[i], gix_read_result.xel[i].data, gix_read_result.xel[i].length);
    }
    ret = true;
  }

  return ret;
}

bool GIXMotorDriver::read_present_position(gix_position_info_t &position_info)
{
  bool ret = false;

  if (is_ready() == false) return false;

  gix_sync_read_param.addr = 132;
  gix_sync_read_param.length = 4;
  gix_sync_read_param.id_count = N_MOTORS;

  if(dxl.syncRead(gix_sync_read_param, gix_read_result)){
    for (int i=0; i<gix_sync_read_param.id_count; i++) {
      memcpy(&position_info.value[i], gix_read_result.xel[i].data, gix_read_result.xel[i].length);
    }
    ret = true;
  }

  return ret;
}

bool GIXMotorDriver::read_present_velocity(gix_velocity_info_t &velocity_info)
{
  bool ret = false;

  if (is_ready() == false) return false;

  gix_sync_read_param.addr = 128;
  gix_sync_read_param.length = 4;
  gix_sync_read_param.id_count = N_MOTORS;

  if(dxl.syncRead(gix_sync_read_param, gix_read_result)){
    for (int i=0; i<gix_sync_read_param.id_count; i++) {
      memcpy(&velocity_info.value[i], gix_read_result.xel[i].data, gix_read_result.xel[i].length);
    }
    ret = true;
  }

  return ret;
}

bool GIXMotorDriver::read_present_current(gix_current_info_t &current_info)
{
  bool ret = false;

  if (is_ready() == false) return false;

  gix_sync_read_param.addr = 126;
  gix_sync_read_param.length = 2;
  gix_sync_read_param.id_count = N_MOTORS;

  if(dxl.syncRead(gix_sync_read_param, gix_read_result)){
    for (int i=0; i<gix_sync_read_param.id_count; i++) {
      memcpy(&current_info.value[i], gix_read_result.xel[i].data, gix_read_result.xel[i].length);
    }
    ret = true;
  }

  return ret;
}

bool GIXMotorDriver::read_profile_acceleration(gix_accel_info_t &accel_info)
{
  bool ret = false;

  if (is_ready() == false) return false;

  gix_sync_read_param.addr = 108;
  gix_sync_read_param.length = 4;
  gix_sync_read_param.id_count = N_MOTORS;

  if(dxl.syncRead(gix_sync_read_param, gix_read_result)){
    for (int i=0; i<gix_sync_read_param.id_count; i++) {
      memcpy(&accel_info.value[i], gix_read_result.xel[i].data, gix_read_result.xel[i].length);
    }
    ret = true;
  }

  return ret;
}

bool GIXMotorDriver::read_profile_velocity(gix_accel_info_t &accel_info)
{
  bool ret = false;

  if (is_ready() == false) return false;

  gix_sync_read_param.addr = 112;
  gix_sync_read_param.length = 4;
  gix_sync_read_param.id_count = N_MOTORS;

  if(dxl.syncRead(gix_sync_read_param, gix_read_result)){
    for (int i=0; i<gix_sync_read_param.id_count; i++) {
      memcpy(&accel_info.value[i], gix_read_result.xel[i].data, gix_read_result.xel[i].length);
    }
    ret = true;
  }

  return ret;
}

bool GIXMotorDriver::read_goal_current(gix_current_info_t &current_info)
{
  bool ret = false;

  if (is_ready() == false) return false;

  gix_sync_read_param.addr = 102;
  gix_sync_read_param.length = 2;
  gix_sync_read_param.id_count = N_MOTORS;

  if(dxl.syncRead(gix_sync_read_param, gix_read_result)){
    for (int i=0; i<gix_sync_read_param.id_count; i++) {
      memcpy(&current_info.value[i], gix_read_result.xel[i].data, gix_read_result.xel[i].length);
    }
    ret = true;
  }

  return ret;
}

bool GIXMotorDriver::write_goal_position_joint(gix_position_info_t &position_info)
{
  bool ret = false;

  if (is_ready() == false) return false;

  gix_sync_write_param.addr = 116;
  gix_sync_write_param.length = 4;
  gix_sync_write_param.id_count = N_MOTORS;

  for (int i=0; i<gix_sync_write_param.id_count; i++) {
    memcpy(gix_sync_write_param.xel[i].data, &position_info.value[i], gix_sync_write_param.length);
  }

  if(dxl.syncWrite(gix_sync_write_param)){
    ret = true;
  }

  return ret;
}

bool GIXMotorDriver::write_profile_acceleration_joint(gix_accel_info_t &accel_info)
{
  bool ret = false;

  if (is_ready() == false) return false;

  gix_sync_write_param.addr = 108;
  gix_sync_write_param.length = 4;
  gix_sync_write_param.id_count = N_MOTORS;

  for (int i=0; i<gix_sync_write_param.id_count; i++) {
    memcpy(gix_sync_write_param.xel[i].data, &accel_info.value[i], gix_sync_write_param.length);
  }

  if(dxl.syncWrite(gix_sync_write_param)){
    ret = true;
  }

  return ret;
}

bool GIXMotorDriver::write_profile_velocity_joint(gix_accel_info_t &accel_info)
{
  bool ret = false;

  if (is_ready() == false) return false;

  gix_sync_write_param.addr = 112;
  gix_sync_write_param.length = 4;
  gix_sync_write_param.id_count = N_MOTORS;

  for (int i=0; i<gix_sync_write_param.id_count; i++) {
    memcpy(gix_sync_write_param.xel[i].data, &accel_info.value[i], gix_sync_write_param.length);
  }

  if(dxl.syncWrite(gix_sync_write_param)){
    ret = true;
  }

  return ret;
}

bool GIXMotorDriver::write_goal_current_joint(gix_current_info_t &current_info)
{
  bool ret = false;

  if (is_ready() == false) return false;

  gix_sync_write_param.addr = 102;
  gix_sync_write_param.length = 2;
  gix_sync_write_param.id_count = N_MOTORS;

  for (int i=0; i<gix_sync_write_param.id_count; i++) {
    memcpy(gix_sync_write_param.xel[i].data, &current_info.value[i], gix_sync_write_param.length);
  }

  if(dxl.syncWrite(gix_sync_write_param)){
    ret = true;
  }

  return ret;
}

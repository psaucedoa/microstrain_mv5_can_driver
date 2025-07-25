/*
 * Copyright 2024 Construction Engineering Research Laboratory (CERL)
 * Engineer Reseach and Development Center (ERDC)
 * U.S. Army Corps of Engineers
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
 */

#include "microstrain_mv5_can_driver/microstrain_mv5_can_driver.hpp"

#include <memory>
#include <string>

using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
namespace rlc = rclcpp_lifecycle;

namespace ros2_j1939
{

MicrostrainMV5CanDriver::MicrostrainMV5CanDriver(const rclcpp::NodeOptions & OPTIONS)
: rclcpp_lifecycle::LifecycleNode("microstrain_mv5_can_driver", OPTIONS)
{}

MicrostrainMV5CanDriver::~MicrostrainMV5CanDriver() {}

LNI::CallbackReturn MicrostrainMV5CanDriver::on_configure(const rlc::State & state)
{
  // (void)state;

  LNI::on_configure(state);

  // params
  dbw_dbc_file_ = this->declare_parameter<std::string>("dbw_dbc_file", "");
  frame_id_ = this->declare_parameter<std::string>("frame_id", "");
  sensor_name_ = this->declare_parameter<std::string>("sensor_name", "");
  device_ID_ = this->declare_parameter<uint8_t>("device_ID", 0);
  sub_topic_can_ = this->declare_parameter<std::string>("can_sub_topic", "");
  pub_topic_can_ = this->declare_parameter<std::string>("pub_topic_can", "");
  pub_topic_imu_ = this->declare_parameter<std::string>("pub_topic_imu","");

  this->rename_service_ = this->create_service<j1939_msgs::srv::ImuRename>(
    "microstrain/" + sensor_name_ + "/rename", std::bind(
      &MicrostrainMV5CanDriver::txRename, this,
      std::placeholders::_1,
      std::placeholders::_2,
      std::placeholders::_3
    )
  );

  this->tare_service_ = this->create_service<j1939_msgs::srv::ImuTareOrientation>(
    "microstrain/" + sensor_name_ + "/tare", std::bind(
      &MicrostrainMV5CanDriver::txTareOrientation, this,
      std::placeholders::_1,
      std::placeholders::_2,
      std::placeholders::_3
    )
  );

  this->set_orientation_service_ = this->create_service<j1939_msgs::srv::ImuSetOrientation>(
    "microstrain/" + sensor_name_ + "/set_orientation", std::bind(
      &MicrostrainMV5CanDriver::txSetOrientation, this,
      std::placeholders::_1,
      std::placeholders::_2,
      std::placeholders::_3
    )
  );

  this->reset_attitude_service_ = this->create_service<j1939_msgs::srv::ImuResetAttitude>(
    "microstrain/" + sensor_name_ + "/reset_attitude", std::bind(
      &MicrostrainMV5CanDriver::txResetAttitude, this,
      std::placeholders::_1,
      std::placeholders::_2,
      std::placeholders::_3
    )
  );

  this->set_data_rate_service_ = this->create_service<j1939_msgs::srv::ImuSetDataRate>(
    "microstrain/" + sensor_name_ + "/set_data_rate", std::bind(
      &MicrostrainMV5CanDriver::txSetDataRate, this,
      std::placeholders::_1,
      std::placeholders::_2,
      std::placeholders::_3
    )
  );

  this->dbw_dbc_db_ = NewEagle::DbcBuilder().NewDbc(dbw_dbc_file_);

  RCLCPP_INFO(this->get_logger(), "dbw_dbc_file: %s", dbw_dbc_file_.c_str());
  RCLCPP_INFO(this->get_logger(), "frame_id: %s", frame_id_.c_str());
  RCLCPP_INFO(this->get_logger(), "sensor_name: %s", sensor_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "device_id: %d", device_ID_);
  RCLCPP_INFO(this->get_logger(), "sub_topic_can: %s", sub_topic_can_.c_str());
  RCLCPP_INFO(this->get_logger(), "pub_topic_can: %s", pub_topic_can_.c_str());
  RCLCPP_INFO(this->get_logger(), "pub_topic_imu: %s", pub_topic_imu_.c_str());


  try
  {
    this->setupDatabase();

    // setup subscribers
    this->sub_can_ = this->create_subscription<can_msgs::msg::Frame>(
        this->sub_topic_can_, 500, std::bind(&MicrostrainMV5CanDriver::rxFrame, this,
        std::placeholders::_1));

    // setup publishers
    this->configurePublishers();
  }
  catch (const std::exception & e) 
  {
    RCLCPP_ERROR(this->get_logger(), "Error w/ on_configure: %s", e.what());
    return LNI::CallbackReturn::FAILURE;
  }

  RCLCPP_DEBUG(this->get_logger(), "Successfully Configured!");

  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn MicrostrainMV5CanDriver::on_activate(const rlc::State & state)
{
  LNI::on_activate(state);
  // when driver activates, configrue the device
  this->activatePublishers();
  txGetName();
  // rclcpp::sleep_for(50ms);
  // txPause(pause_time_);

  RCLCPP_DEBUG(this->get_logger(), "Microstrain activated.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn MicrostrainMV5CanDriver::on_deactivate(const rlc::State & state)
{
  // (void)state;

  LNI::on_deactivate(state);

  deactivatePublishers();

  RCLCPP_DEBUG(this->get_logger(), "Microstrain deactivated.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn MicrostrainMV5CanDriver::on_cleanup(const rlc::State & state)
{
  // (void)state;

  LNI::on_cleanup(state);

  RCLCPP_DEBUG(this->get_logger(), "Microstrain cleaned up.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn MicrostrainMV5CanDriver::on_shutdown(const rlc::State & state)
{
  // (void)state;

  LNI::on_shutdown(state);

  RCLCPP_DEBUG(this->get_logger(), "Microstrain shutting down.");
  return LNI::CallbackReturn::SUCCESS;
}

// CANUSB COMMS FUNCTIONS //
void MicrostrainMV5CanDriver::rxFrame(const can_msgs::msg::Frame::SharedPtr MSG)
{
  if (!MSG->is_rtr && !MSG->is_error && (device_ID_ == (MSG->id & 0x000000FFu))) {
    switch (MSG->id & 0xFFFFFF00u) {
      case ID_SLOPE:
        rxSlopeFrame(MSG);
        break;
      case ID_ACCEL:
        rxAccelFrame(MSG);
        break;
      case ID_ANGULAR_RATE:
        rxAngularRateFrame(MSG);
        break;
      case ID_NAME:
        // RCLCPP_INFO(this->get_logger(), "GOT NAME");
        rxGetName(MSG);
      default:
        break;
    }
  }
}

// BEGIN MANAGEMENT FUNCTIONS //
void MicrostrainMV5CanDriver::setupDatabase()
{
  dbw_dbc_db_ = NewEagle::DbcBuilder().NewDbc(dbw_dbc_file_);
  dbc_name_msg_map_ = * dbw_dbc_db_.GetMessages();

  for (auto [key, value] : dbc_name_msg_map_)
  {
    // strip id of priority and source address info
    uint32_t stripped_id = value.GetId() & 0x00FFFF00u;
    dbc_id_msg_map_[stripped_id] = value;
  }
}

void MicrostrainMV5CanDriver::configurePublishers()
{
  // pub_topic_imu_ = pub_topic_imu_ + sensor_name_;
  pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>(pub_topic_imu_, 20);
  pub_can_ = this->create_publisher<can_msgs::msg::Frame>(pub_topic_can_, 20);
}

void MicrostrainMV5CanDriver::activatePublishers()
{
  pub_imu_->on_activate();
  pub_can_->on_activate();
}

void MicrostrainMV5CanDriver::deactivatePublishers()
{
  pub_imu_->on_deactivate();
  pub_can_->on_deactivate();
}

void MicrostrainMV5CanDriver::rxGetName(const can_msgs::msg::Frame::SharedPtr MSG)
{
  this->device_name_ = MSG->data;

  for (int i = 0; i < 8; i++)
  {
    RCLCPP_INFO(this->get_logger(), "[rxGetName] NAME DATA: %d", this->device_name_[i]);
  }

}

void MicrostrainMV5CanDriver::txGetName()
{
  // stuff data (see MV5-AR j1939 user manual for more info)
  // std::array<uint8_t, 8UL> data_out = {0x00u, 0xB1u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00};
  std::array<uint8_t, 8UL> data_out = {0x00u, 0xEEu, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00};
  can_msgs::msg::Frame frame_out;
  uint32_t j1939_id = 0x18EA0000u;
  // RCLCPP_INFO(this->get_logger(), "ID: %d", j1939_id);
  j1939_id = j1939_id | (device_ID_ << 8);
  // RCLCPP_INFO(this->get_logger(), "ID: %d", j1939_id);

  frame_out.header.stamp = this->now();
  frame_out.header.frame_id = "txGetName";
  frame_out.id = j1939_id;
  frame_out.is_rtr = false;
  frame_out.is_extended = true;
  frame_out.is_error = false;
  frame_out.dlc = 3;
  frame_out.data = data_out;

  try {
    pub_can_->publish(frame_out);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "COULD NOT PUBLISH FRAME: %s", e.what());
  }
}

// ADDRESS MANAGEMENT FUNCTIONS //
void MicrostrainMV5CanDriver::createDataArray(
  const std::vector<uint16_t> data_in, const std::vector<uint16_t> data_lengths, 
  std::array<uint8_t, 8UL> &data_out)
{
  uint64_t data_concatenated = 0;
  uint64_t data_mask = 0x00000000000000FF;
  int size = data_in.size();

  for (int i = 0; i < size; i++)
  {
    data_concatenated = data_concatenated << data_lengths[size - 1 - i];
    data_concatenated += data_in[size - 1 - i];
  }

  for (int i = 0; i < 8; i++)
  {
    data_out[i] = (data_mask & data_concatenated >> 8*i);
  }
}

void MicrostrainMV5CanDriver::txRename(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<j1939_msgs::srv::ImuRename::Request> request, 
  const std::shared_ptr<j1939_msgs::srv::ImuRename::Response> response 
)
{
  (void)request_header;
  response->success = true;
  response->message = "Received IMU " + std::to_string(this->device_ID_) + 
    " rename request. Renaming to " + std::to_string(request->new_name) + "...";

  if( (request->new_name < 1) || (request->new_name > 253) )
  {
    response->success = false;
    response->message = "Valid name range is [1, 253]. \nName not set. \nPlease try again.";
    return;
  }

  std::array<uint8_t, 8UL> BAM_data_out = {0x20u, 0x09u, 0x00u, 0x02u, 0xFFu, 0xD8u, 0xFEu, 0x00u};
  can_msgs::msg::Frame BAM_frame_out;
  uint32_t j1939_id = 0x1CECFF00u;
  BAM_frame_out.header.stamp = this->now();
  BAM_frame_out.header.frame_id = "BAM_Rename_command";
  BAM_frame_out.id = j1939_id;
  BAM_frame_out.is_rtr = false;
  BAM_frame_out.is_extended = true;
  BAM_frame_out.is_error = false;
  BAM_frame_out.dlc = 8;
  BAM_frame_out.data = BAM_data_out;

  // for (int i = 0; i < 8; i++)
  // {
    // RCLCPP_INFO(this->get_logger(), "NAME DATA: %d", this->device_name_[i]);
  // }

  std::array<uint8_t, 8UL> name_data_out_1 = {
    0x01u, 
    this->device_name_[0], 
    this->device_name_[1], 
    this->device_name_[2], 
    this->device_name_[3], 
    this->device_name_[4], 
    this->device_name_[5], 
    this->device_name_[6]
  };

  // for (int i = 0; i < 8; i++)
  // {
    // RCLCPP_INFO(this->get_logger(), "NAME DATA OUT: %d", name_data_out_1[i]);
  // }

  can_msgs::msg::Frame name_frame_out_1;
  j1939_id = 0x1CEBFF00u;
  name_frame_out_1.header.stamp = this->now();
  name_frame_out_1.header.frame_id = "BAM_Rename_command";
  name_frame_out_1.id = j1939_id;
  name_frame_out_1.is_rtr = false;
  name_frame_out_1.is_extended = true;
  name_frame_out_1.is_error = false;
  name_frame_out_1.dlc = 8;
  name_frame_out_1.data = name_data_out_1;

  std::array<uint8_t, 8UL> name_data_out_2 = {
    0x02u, 
    this->device_name_[7], 
    static_cast<uint8_t>(request->new_name), 
    0xFF, 
    0xFF, 
    0xFF, 
    0xFF, 
    0xFF
  };
  
  can_msgs::msg::Frame name_frame_out_2;
  j1939_id = 0x1CEBFF00u;
  name_frame_out_2.header.stamp = this->now();
  name_frame_out_2.header.frame_id = "BAM_Rename_command";
  name_frame_out_2.id = j1939_id;
  name_frame_out_2.is_rtr = false;
  name_frame_out_2.is_extended = true;
  name_frame_out_2.is_error = false;
  name_frame_out_2.dlc = 8;
  name_frame_out_2.data = name_data_out_2;

  txLock(false);
  rclcpp::sleep_for(100ms);
  pub_can_->publish(BAM_frame_out);
  rclcpp::sleep_for(100ms);
  pub_can_->publish(name_frame_out_1);
  rclcpp::sleep_for(100ms);
  pub_can_->publish(name_frame_out_2);
  rclcpp::sleep_for(100ms);

  //update device id
  this->device_ID_ = request->new_name;
}

void MicrostrainMV5CanDriver::txPause(uint16_t time_ms)
{
  const uint8_t hi_byte = (time_ms >> 8) & 0xFF;
  const uint8_t lo_byte = time_ms & 0xFF;

  std::array<uint8_t, 8UL> data_out = {0x3fu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, hi_byte, lo_byte, 0xFFu};
  can_msgs::msg::Frame frame_out;
  uint32_t j1939_id = 0x18DF0099u;
  j1939_id = j1939_id | (device_ID_ << 8);

  frame_out.header.stamp = this->now();
  frame_out.header.frame_id = "can";
  frame_out.id = j1939_id;
  frame_out.is_rtr = false;
  frame_out.is_extended = true;
  frame_out.is_error = false;
  frame_out.dlc = 8;
  frame_out.data = data_out;

  try {
    pub_can_->publish(frame_out);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "COULD NOT PUBLISH FRAME: %s", e.what());
  }
}

void MicrostrainMV5CanDriver::txLock(const bool state)
{
  // make sure the true/false is a uint8
  uint8_t lock_state = static_cast<uint8_t>(state);

  // stuff data (see MV5-AR j1939 user manual for more info)
  std::array<uint8_t, 8UL> data_out = {0x50u, 0x49u, 0x4Du, 0x32u, 0x4Eu, 0x41u, 0x43u, 0x00};
  // std::array<uint8_t, 8UL> data_out = {0x43u, 0x41u, 0x4Eu, 0x32u, 0x4Du, 0x49u, 0x50, 0x01u};
  can_msgs::msg::Frame frame_out;
  uint32_t j1939_id = 0x18C0FF00u;

  frame_out.header.stamp = this->now();
  frame_out.header.frame_id = "txLock";
  frame_out.id = j1939_id;
  frame_out.is_rtr = false;
  frame_out.is_extended = true;
  frame_out.is_error = false;
  frame_out.dlc = 8;
  frame_out.data = data_out;

  try {
    pub_can_->publish(frame_out);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "COULD NOT PUBLISH FRAME: %s", e.what());
  }
}

void MicrostrainMV5CanDriver::txTareOrientation(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<j1939_msgs::srv::ImuTareOrientation::Request> request,
  const std::shared_ptr<j1939_msgs::srv::ImuTareOrientation::Response> response
)
{
  // successfully received the request
  (void)request_header;
  response->success = true;
  response->message = "Received IMU" + std::to_string(this->device_ID_) + " tare request...";

  // early return checks for correct data ranges
  if( (request->yaw_offset_deg < 0) || (request->yaw_offset_deg > 360) )
  {
    response->success = false;
    response->message = "Valid Yaw range is [0, 360]. Please try again.";
    return;
  }
  if( (request->roll_granularity_deg < 0) || (request->roll_granularity_deg > 90) )
  {
    response->success = false;
    response->message = "Valid Roll granularity range is [0, 90]. Please try again.";
    return;
  }
  if(request->pitch_granularity_deg < 0 || request->pitch_granularity_deg > 90)
  {
    response->success = false;
    response->message = "Valid Pitch granularity range is [0, 90]. Please try again.";
    return;
  }

  // break up the yaw into 2 bytes
  const uint16_t yaw = request->yaw_offset_deg * 10;
  const uint8_t hi_yaw_byte = (yaw >> 8) & 0xFFu;
  const uint8_t lo_yaw_byte = yaw & 0xFFu;

  // format the data for ouput
  std::array<uint8_t, 8UL> data_out = {
    static_cast<uint8_t>(request->roll_granularity_deg),
    static_cast<uint8_t>(request->pitch_granularity_deg),
    hi_yaw_byte,
    lo_yaw_byte,
    0x00u,
    0x00u,
    0x00u,
    0x00u
  };

  // stuffing can frame
  can_msgs::msg::Frame frame_out;
  uint32_t j1939_id = 0x18B50099u;
  j1939_id = j1939_id | (device_ID_ << 8);

  frame_out.header.stamp = this->now();
  frame_out.header.frame_id = "can";
  frame_out.id = j1939_id;
  frame_out.is_rtr = false;
  frame_out.is_extended = true;
  frame_out.is_error = false;
  frame_out.dlc = 4;
  frame_out.data = data_out;

  // publish
  try {
    pub_can_->publish(frame_out);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "COULD NOT PUBLISH FRAME: %s", e.what());
  }
}

void MicrostrainMV5CanDriver::txSetOrientation(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<j1939_msgs::srv::ImuSetOrientation::Request> request,
  const std::shared_ptr<j1939_msgs::srv::ImuSetOrientation::Response> response
)
{
  (void)request_header;
  // set up some early returns to check for correct data range
  if( (request->x_rotation_offset < -180) || (request->x_rotation_offset > 180) )
  {
    response->success = false;
    response->message = "Valid X-axis rotation offset range is [-180, 180]. Please try again.";
    return;
  }
  if( (request->y_rotation_offset < -180) || (request->y_rotation_offset > 180) )
  {
    response->success = false;
    response->message = "Valid Y-axis rotation offset range is [-180, 180]. Please try again.";
    return;
  }
  if( (request->z_rotation_offset < -180) || (request->z_rotation_offset > 180) )
  {
    response->success = false;
    response->message = "Valid Z-axis rotation offset range is [-180, 180]. Please try again.";
    return;
  }

  // the can message has 0.1 deg resolution, so we multiply by ten
  const uint16_t x_rotation_int = static_cast<int16_t>((request->x_rotation_offset+180) * 10);
  const uint16_t y_rotation_int = static_cast<int16_t>((request->y_rotation_offset+180) * 10);
  const uint16_t z_rotation_int = static_cast<int16_t>((request->z_rotation_offset+180) * 10);

  // then we split each axis into two bytes
  const uint8_t hi_x_byte = (x_rotation_int >> 8) & 0xFFu;
  const uint8_t hi_y_byte = (y_rotation_int >> 8) & 0xFFu;
  const uint8_t hi_z_byte = (z_rotation_int >> 8) & 0xFFu;

  const uint8_t lo_x_byte = x_rotation_int & 0xFFu;
  const uint8_t lo_y_byte = y_rotation_int & 0xFFu;
  const uint8_t lo_z_byte = z_rotation_int & 0xFFu;

  // and we stuff the data out array
  std::array<uint8_t, 8UL> data_out = {hi_x_byte, lo_x_byte, hi_y_byte, lo_y_byte,
    hi_z_byte, lo_z_byte, 0x00u, 0x00u};

  // and we stuff the can frame
  can_msgs::msg::Frame frame_out;
  uint32_t j1939_id = 0x18B60099u;
  j1939_id = j1939_id | (device_ID_ << 8);

  frame_out.header.stamp = this->now();
  frame_out.header.frame_id = "can";
  frame_out.id = j1939_id;
  frame_out.is_rtr = false;
  frame_out.is_extended = true;
  frame_out.is_error = false;
  frame_out.dlc = 6;
  frame_out.data = data_out;

  // and we publish the can frame
  try {
    pub_can_->publish(frame_out);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "COULD NOT PUBLISH FRAME: %s", e.what());
  }
}

void MicrostrainMV5CanDriver::txResetAttitude(const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<j1939_msgs::srv::ImuResetAttitude::Request> request, 
  const std::shared_ptr<j1939_msgs::srv::ImuResetAttitude::Response> response
)
{
  (void)request_header;
  // early return for if false
  if(request->value == false)
  {
    response->success = false;
    response->message = "Reset attitude false. So this doesn't do anything lmao.";
    return;
  }

  // according to manual, no data fields for this PGN
  std::array<uint8_t, 8UL> data_out = {0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u};

  // stuff teh can frame
  can_msgs::msg::Frame frame_out;
  uint32_t j1939_id = 0x18B40099u;
  j1939_id = j1939_id | (device_ID_ << 8);

  frame_out.header.stamp = this->now();
  frame_out.header.frame_id = "can";
  frame_out.id = j1939_id;
  frame_out.is_rtr = false;
  frame_out.is_extended = true;
  frame_out.is_error = false;
  frame_out.dlc = 8;
  frame_out.data = data_out;

  // publish
  try {
    pub_can_->publish(frame_out);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "COULD NOT PUBLISH FRAME: %s", e.what());
  }
}

void MicrostrainMV5CanDriver::txSetDataRate(const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<j1939_msgs::srv::ImuSetDataRate::Request> request, 
  const std::shared_ptr<j1939_msgs::srv::ImuSetDataRate::Response> response
)
{
  (void)request_header;
  // early return for if false
  if((request->hz < 1) || (request->hz > 250))
  {
    response->success = false;
    response->message = "Valid Hz range is [1, 250]. Please try again.";
    return;
  }

  // the devices want the ms between messages, so convert from hz to ms gap or w/e that's called i forgot
  const uint16_t ms_gap = round(1000 / request->hz);

  // we split into two bytes
  const uint8_t hi_ms_byte = (ms_gap >> 8) & 0xFFu;
  const uint8_t lo_ms_byte = ms_gap & 0xFFu;

  // do this for all three of angular rate, acceleration, and slope PGNs
  // and we stuff the data out arrays. First for Angular Rate
  const std::array<uint8_t, 8UL> data_out_2a = {0x2A, 0xF0, 0x00, lo_ms_byte,
    hi_ms_byte, 0x00u, 0x00u, 0x00u
  };

  const std::array<uint8_t, 8UL> data_out_2d = {0x2d, 0xF0, 0x00, lo_ms_byte,
    hi_ms_byte, 0x00u, 0x00u, 0x00u
  };

  const std::array<uint8_t, 8UL> data_out_29 = {0x29, 0xF0, 0x00, lo_ms_byte,
    hi_ms_byte, 0x00u, 0x00u, 0x00u
  };

  // stuff the can frame
  can_msgs::msg::Frame frame_out_2a;
  uint32_t j1939_id = 0x18B20099u;
  j1939_id = j1939_id | (device_ID_ << 8);
  frame_out_2a.header.stamp = this->now();
  frame_out_2a.header.frame_id = "IMU Hz setting out";
  frame_out_2a.id = j1939_id;
  frame_out_2a.is_rtr = false;
  frame_out_2a.is_extended = true;
  frame_out_2a.is_error = false;
  frame_out_2a.dlc = 5;
  frame_out_2a.data = data_out_2a;
  pub_can_->publish(frame_out_2a);
  rclcpp::sleep_for(100ms);

  // stuff the can frame
  can_msgs::msg::Frame frame_out_2d;
  frame_out_2d.header.stamp = this->now();
  frame_out_2d.header.frame_id = "IMU Hz setting out";
  frame_out_2d.id = j1939_id;
  frame_out_2d.is_rtr = false;
  frame_out_2d.is_extended = true;
  frame_out_2d.is_error = false;
  frame_out_2d.dlc = 5;
  frame_out_2d.data = data_out_2d;
  pub_can_->publish(frame_out_2d);
  rclcpp::sleep_for(100ms);

  // stuff the can frame
  can_msgs::msg::Frame frame_out_29;
  frame_out_29.header.stamp = this->now();
  frame_out_29.header.frame_id = "IMU Hz setting out";
  frame_out_29.id = j1939_id;
  frame_out_29.is_rtr = false;
  frame_out_29.is_extended = true;
  frame_out_29.is_error = false;
  frame_out_29.dlc = 5;
  frame_out_29.data = data_out_29;
  pub_can_->publish(frame_out_29);

  response->success = true;
  response->message = "Data rate message sent.";



}

void MicrostrainMV5CanDriver::rxSlopeFrame(const can_msgs::msg::Frame::SharedPtr MSG)
{
  NewEagle::DbcMessage * message = dbw_dbc_db_.GetMessage("SlopeSensor2");

  // if minimum datalength is met
  if (MSG->dlc >= message->GetDlc()) {
    message->SetFrame(MSG);
    // populating header - doing this once at the first received frame
    imu_msg_out_.header.stamp = MSG->header.stamp;
    imu_msg_out_.header.frame_id = frame_id_;

    double roll_deg = static_cast<double>(message->GetSignal("RollAngle")->GetResult());
    double pitch_deg = static_cast<double>(message->GetSignal("PitchAngle")->GetResult());
    double yaw_deg = 0.0;

    double roll = roll_deg * M_PI / 180.0;
    double pitch = pitch_deg * M_PI / 180.0;
    double yaw = yaw_deg * M_PI / 180.0;

    // Create a quaternion from roll, pitch, and yaw
    tf2::Quaternion quaternion;
    quaternion.setRPY(roll, pitch, yaw);

    // Set the orientation as quaternion
    imu_msg_out_.orientation = tf2::toMsg(quaternion);
  }
}

void MicrostrainMV5CanDriver::rxAccelFrame(const can_msgs::msg::Frame::SharedPtr MSG)
{
  // get message by name from the dbc database
  NewEagle::DbcMessage * message = this->dbw_dbc_db_.GetMessage("AccelerationSensor");
  // if minimum datalength is met
  if (MSG->dlc >= message->GetDlc()) {
    // set frame
    message->SetFrame(MSG);

    // populating data fields
    imu_msg_out_.linear_acceleration.x =
      static_cast<double>(message->GetSignal("LongitudinalAcceleration")->GetResult());
    imu_msg_out_.linear_acceleration.y =
      static_cast<double>(message->GetSignal("LateralAcceleration")->GetResult());
    imu_msg_out_.linear_acceleration.z =
      static_cast<double>(message->GetSignal("VerticalAcceleration")->GetResult());
  }
}

void MicrostrainMV5CanDriver::rxAngularRateFrame(const can_msgs::msg::Frame::SharedPtr MSG)
{
  NewEagle::DbcMessage * message = dbw_dbc_db_.GetMessage("AngularRate");

  // if minimum datalength is met
  if (MSG->dlc >= message->GetDlc()) {
    message->SetFrame(MSG);
    // TODO(arturo): add some thresholding? -> pub w/ warn if too big?

    double angular_velocity_x_deg_s_ = static_cast<double>(message->GetSignal("YawRate")->GetResult());
    double angular_velocity_y_deg_s_ = static_cast<double>(message->GetSignal("PitchRate")->GetResult());
    double angular_velocity_z_deg_s_ = static_cast<double>(message->GetSignal("RollRate")->GetResult());

    // convert to rad/s
    double angular_velocity_x_rad_s_ = angular_velocity_x_deg_s_ * M_PI / 180.0;
    double angular_velocity_y_rad_s_ = angular_velocity_y_deg_s_ * M_PI / 180.0;
    double angular_velocity_z_rad_s_ = angular_velocity_z_deg_s_ * M_PI / 180.0;

    // populating data fields
    imu_msg_out_.angular_velocity.x = angular_velocity_x_rad_s_;
    imu_msg_out_.angular_velocity.y = angular_velocity_y_rad_s_;
    imu_msg_out_.angular_velocity.z = angular_velocity_z_rad_s_;

    pub_imu_->publish(imu_msg_out_);
  }

}

}  // end namespace ros2_j1939

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_j1939::MicrostrainMV5CanDriver)

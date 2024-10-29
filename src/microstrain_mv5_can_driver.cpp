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
{
  // params
  dbw_dbc_file_ = this->declare_parameter<std::string>("dbw_dbc_file", "");
  frame_id_ = this->declare_parameter<std::string>("frame_id", "");
  sensor_name_ = this->declare_parameter<std::string>("sensor_name", "");
  device_ID_ = this->declare_parameter<uint8_t>("device_ID", 0);
  sub_topic_can_ = this->declare_parameter<std::string>("can_sub_topic", "");
  pub_topic_can_ = this->declare_parameter<std::string>("pub_topic_can", "");

  // pause_time_ = this->declare_parameter<uint16_t>("pause_time", 1000);  // turn into a service

  tare_ = this->declare_parameter<bool>("tare", false);  // turn into a service?
  roll_granularity_deg_ = this->declare_parameter<uint8_t>("roll_granularity_deg", 5);
  pitch_granularity_deg_ = this->declare_parameter<uint8_t>("pitch_granularity_deg", 5);
  yaw_direction_deg_ = this->declare_parameter<float>("yaw_direction_deg", 0.0);

  set_orientation_ = this->declare_parameter<bool>("set_orientation", false);
  x_rotation_ = this->declare_parameter<float>("x_rotation", 0.0);
  y_rotation_ = this->declare_parameter<float>("y_rotation", 0.0);
  z_rotation_ = this->declare_parameter<float>("z_rotation", 0.0);

  reset_attitude_ = this->declare_parameter<bool>("reset_attitude", false);  // service?

  // set_new_source_address_ = this->declare_parameter<bool>("set_new_source_address", false);  // serv
  // new_source_address_ = this->declare_parameter<uint8_t>("new_source_address", 0);  // serv

  this->dbw_dbc_db_ = NewEagle::DbcBuilder().NewDbc(dbw_dbc_file_);

  RCLCPP_INFO(this->get_logger(), "dbw_dbc_file: %s", dbw_dbc_file_.c_str());
  RCLCPP_INFO(this->get_logger(), "frame_id: %s", frame_id_.c_str());
  RCLCPP_INFO(this->get_logger(), "sensor_name: %s", sensor_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "device_id: %d", device_ID_);
  RCLCPP_INFO(this->get_logger(), "sub_topic_can: %s", sub_topic_can_.c_str());
  RCLCPP_INFO(this->get_logger(), "pub_topic_can: %s", pub_topic_can_.c_str());
  RCLCPP_INFO(this->get_logger(), "pub_topic_imu: %s", pub_topic_imu_.c_str());

  RCLCPP_INFO(this->get_logger(), "reset_attitude: %s", reset_attitude_ ? "true" : "false");

  if (tare_ && !reset_attitude_) {
    RCLCPP_INFO(this->get_logger(), "tare: %s", tare_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "roll_granularity_deg: %d", roll_granularity_deg_);
    RCLCPP_INFO(this->get_logger(), "pitch_granularity_deg: %d", pitch_granularity_deg_);
    RCLCPP_INFO(this->get_logger(), "yaw_direction_deg: %f", yaw_direction_deg_);
  }

  if (set_orientation_ && !reset_attitude_) {
    RCLCPP_INFO(this->get_logger(), "set_orientation: %s", set_orientation_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "x_rotation: %f", x_rotation_);
    RCLCPP_INFO(this->get_logger(), "y_rotation: %f", y_rotation_);
    RCLCPP_INFO(this->get_logger(), "z_rotation: %f", z_rotation_);
  }

  // if (set_new_source_address_) {
  //   RCLCPP_INFO(
  //     this->get_logger(), "set_new_source_address: %s", set_new_source_address_ ? "true" : "false");
  //   RCLCPP_INFO(this->get_logger(), "new_source_address: %d", new_source_address_);
  // }
}

MicrostrainMV5CanDriver::~MicrostrainMV5CanDriver() {}

// void MicrostrainMV5CanDriver::componentTimerCallback() {}

LNI::CallbackReturn MicrostrainMV5CanDriver::on_configure(const rlc::State & state)
{
  // (void)state;

  LNI::on_configure(state);

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

  // txPause(pause_time_);

  if (reset_attitude_) {
    txResetAttitude();
  } else if (tare_) {
    txTareOrientation(roll_granularity_deg_, pitch_granularity_deg_, yaw_direction_deg_);
  } else if (set_orientation_) {
    txSetOrientation(x_rotation_, y_rotation_, z_rotation_);
  }

  // if (set_new_source_address_) {
  //   txRename(device_name_, new_source_address_);
  // }

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
  pub_topic_imu_ = pub_topic_imu_ + sensor_name_;
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

void MicrostrainMV5CanDriver::generateAddressClaimAttackMsg(
  can_msgs::msg::Frame::SharedPtr MSG, const std::vector<uint32_t> source_addresses)
{
  // we go through each address given in the list
  for(uint32_t address : source_addresses)
  {
    // we add the source address (target of the claim attack) to a 'name declaration' message
    address += 0x18EEFF00;
    // by sending this message with only 0s, our name takes priority, 
    // and the competing device stops publishing   
    std::array<uint8_t, 8UL> claim_data = {
      0x00u, 0x00u, 0x00u, 0x00u, 0x00, 0x00, 0x00, 0x00u
      };

    // then we just stuff the can frame with all our data
    MSG->header.stamp = this->now();
    MSG->header.frame_id = "can";
    MSG->id = address;
    MSG->is_rtr = false;
    MSG->is_extended = true;
    MSG->is_error = false;
    MSG->dlc = 8;
    MSG->data = claim_data;
  }
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

void MicrostrainMV5CanDriver::txTareOrientation(
  const uint8_t roll_granularity_deg, const uint8_t pitch_granularity_deg,
  const float yaw_direction_deg)
{
  const uint16_t yaw = static_cast<uint16_t>(yaw_direction_deg * 10);
  const uint8_t hi_yaw_byte = (yaw >> 8) & 0xFFu;
  const uint8_t lo_yaw_byte = yaw & 0xFFu;

  std::array<uint8_t, 8UL> data_out = {roll_granularity_deg,
    pitch_granularity_deg,
    hi_yaw_byte,
    lo_yaw_byte,
    0x00u,
    0x00u,
    0x00u,
    0x00u};

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

  try {
    pub_can_->publish(frame_out);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "COULD NOT PUBLISH FRAME: %s", e.what());
  }
}

void MicrostrainMV5CanDriver::txSetOrientation(
  const float x_rotation, const float y_rotation, const float z_rotation)
{
  const uint16_t x_rotation_int = static_cast<uint16_t>(x_rotation * 10);
  const uint16_t y_rotation_int = static_cast<uint16_t>(y_rotation * 10);
  const uint16_t z_rotation_int = static_cast<uint16_t>(z_rotation * 10);

  const uint8_t hi_x_byte = (x_rotation_int >> 8) & 0xFFu;
  const uint8_t hi_y_byte = (y_rotation_int >> 8) & 0xFFu;
  const uint8_t hi_z_byte = (z_rotation_int >> 8) & 0xFFu;

  const uint8_t lo_x_byte = x_rotation_int & 0xFFu;
  const uint8_t lo_y_byte = y_rotation_int & 0xFFu;
  const uint8_t lo_z_byte = z_rotation_int & 0xFFu;

  std::array<uint8_t, 8UL> data_out = {hi_x_byte, lo_x_byte, hi_y_byte, lo_y_byte,
    hi_z_byte, lo_z_byte, 0x00u, 0x00u};

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

  try {
    pub_can_->publish(frame_out);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "COULD NOT PUBLISH FRAME: %s", e.what());
  }
}

void MicrostrainMV5CanDriver::txResetAttitude()
{
  std::array<uint8_t, 8UL> data_out = {0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u};

  can_msgs::msg::Frame frame_out;
  uint32_t j1939_id = 0x18B40099u;
  j1939_id = j1939_id | (device_ID_ << 8);

  frame_out.header.stamp = this->now();
  frame_out.header.frame_id = "can";
  frame_out.id = j1939_id;
  frame_out.is_rtr = false;
  frame_out.is_extended = true;
  frame_out.is_error = false;
  frame_out.dlc = 0;
  frame_out.data = data_out;

  try {
    pub_can_->publish(frame_out);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "COULD NOT PUBLISH FRAME: %s", e.what());
  }
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

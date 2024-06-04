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

#include "microstrain_mv5_can_driver/microstrain_mv5_can_driver_node.hpp"

#include <string>

namespace rlc = rclcpp_lifecycle;
using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;

namespace drivers
{

namespace microstrain
{

MicrostrainMV5CanDriverNode::MicrostrainMV5CanDriverNode(const rclcpp::NodeOptions & OPTIONS)
: generic_can_driver::GenericCanDriverNode(OPTIONS)
{
  // params
  dbw_dbc_file_ = this->declare_parameter<std::string>("dbw_dbc_file", "");
  frame_id_ = this->declare_parameter<std::string>("frame_id", "");
  sensor_name_ = this->declare_parameter<std::string>("sensor_name", "");
  device_ID_ = this->declare_parameter<uint8_t>("device_ID", 0);
  sub_topic_can_ = this->declare_parameter<std::string>("can_sub_topic", "");
  pub_topic_can_ = this->declare_parameter<std::string>("pub_topic_can", "");

  pause_time_ = this->declare_parameter<uint16_t>("pause_time", 1000);

  tare_ = this->declare_parameter<bool>("tare", false);
  roll_granularity_deg_ = this->declare_parameter<uint8_t>("roll_granularity_deg", 5);
  pitch_granularity_deg_ = this->declare_parameter<uint8_t>("pitch_granularity_deg", 5);
  yaw_direction_deg_ = this->declare_parameter<float>("yaw_direction_deg", 0.0);

  set_orientation_ = this->declare_parameter<bool>("set_orientation", false);
  x_rotation_ = this->declare_parameter<float>("x_rotation", 0.0);
  y_rotation_ = this->declare_parameter<float>("y_rotation", 0.0);
  z_rotation_ = this->declare_parameter<float>("z_rotation", 0.0);

  reset_attitude_ = this->declare_parameter<bool>("reset_attitude", false);

  set_new_source_address_ = this->declare_parameter<bool>("set_new_source_address", false);
  new_source_address_ = this->declare_parameter<uint8_t>("new_source_address", 0);

  dbw_dbc_db = NewEagle::DbcBuilder().NewDbc(dbw_dbc_file_);

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

  if (set_new_source_address_) {
    RCLCPP_INFO(
      this->get_logger(), "set_new_source_address: %s", set_new_source_address_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "new_source_address: %d", new_source_address_);
  }
}

MicrostrainMV5CanDriverNode::~MicrostrainMV5CanDriverNode() {}

void MicrostrainMV5CanDriverNode::componentTimerCallback() {}

LNI::CallbackReturn MicrostrainMV5CanDriverNode::on_configure(const rlc::State & state)
{
  // (void)state;

  LNI::on_configure(state);

  try {
    // subscribers
    sub_can_ = this->create_subscription<can_msgs::msg::Frame>(
      sub_topic_can_, 500,
      std::bind(&MicrostrainMV5CanDriverNode::rxFrame, this, std::placeholders::_1));

    // publishers
    pub_topic_imu_ = pub_topic_imu_ + sensor_name_;
    pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>(pub_topic_imu_, 20);
    pub_can_ = this->create_publisher<can_msgs::msg::Frame>(pub_topic_can_, 20);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Error w/ on_configure: %s", e.what());
    return LNI::CallbackReturn::FAILURE;
  }

  RCLCPP_DEBUG(this->get_logger(), "Successfully Configured!");

  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn MicrostrainMV5CanDriverNode::on_activate(const rlc::State & state)
{
  LNI::on_activate(state);
  // when driver activates, configrue the device
  pub_imu_->on_activate();
  pub_can_->on_activate();

  txPause(pause_time_);

  if (reset_attitude_) {
    txResetAttitude();
  } else if (tare_) {
    txTareOrientation(roll_granularity_deg_, pitch_granularity_deg_, yaw_direction_deg_);
  } else if (set_orientation_) {
    txSetOrientation(x_rotation_, y_rotation_, z_rotation_);
  }

  if (set_new_source_address_) {
    txRename(device_name_, new_source_address_);
  }

  RCLCPP_DEBUG(this->get_logger(), "Microstrain activated.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn MicrostrainMV5CanDriverNode::on_deactivate(const rlc::State & state)
{
  // (void)state;

  LNI::on_deactivate(state);

  pub_imu_->on_deactivate();

  RCLCPP_DEBUG(this->get_logger(), "Microstrain deactivated.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn MicrostrainMV5CanDriverNode::on_cleanup(const rlc::State & state)
{
  // (void)state;

  LNI::on_cleanup(state);

  RCLCPP_DEBUG(this->get_logger(), "Microstrain cleaned up.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn MicrostrainMV5CanDriverNode::on_shutdown(const rlc::State & state)
{
  // (void)state;

  LNI::on_shutdown(state);

  RCLCPP_DEBUG(this->get_logger(), "Microstrain shutting down.");
  return LNI::CallbackReturn::SUCCESS;
}

void MicrostrainMV5CanDriverNode::txPause(uint16_t time_ms)
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

void MicrostrainMV5CanDriverNode::txTareOrientation(
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

void MicrostrainMV5CanDriverNode::txSetOrientation(
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

void MicrostrainMV5CanDriverNode::txResetAttitude()
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

void MicrostrainMV5CanDriverNode::rxFrame(const can_msgs::msg::Frame::SharedPtr MSG)
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

void MicrostrainMV5CanDriverNode::rxSlopeFrame(const can_msgs::msg::Frame::SharedPtr MSG)
{
  NewEagle::DbcMessage * message = dbw_dbc_db.GetMessage("SlopeSensor2");

  // if minimum datalength is met
  if (MSG->dlc >= message->GetDlc()) {
    message->SetFrame(MSG);
    // populating header - doing this once at the first received frame
    imu_msg_out_.header.stamp = MSG->header.stamp;
    imu_msg_out_.header.frame_id = frame_id_;

    // quaternion. fix w/ correct field
    imu_msg_out_.orientation.x = static_cast<double>(message->GetSignal("RollAngle")->GetResult());
    imu_msg_out_.orientation.y = static_cast<double>(message->GetSignal("PitchAngle")->GetResult());
  }
}

void MicrostrainMV5CanDriverNode::rxAccelFrame(const can_msgs::msg::Frame::SharedPtr MSG)
{
  // get message by name from the dbc database
  NewEagle::DbcMessage * message = dbw_dbc_db.GetMessage("AccelerationSensor");
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

void MicrostrainMV5CanDriverNode::rxAngularRateFrame(const can_msgs::msg::Frame::SharedPtr MSG)
{
  NewEagle::DbcMessage * message = dbw_dbc_db.GetMessage("AngularRate");

  // if minimum datalength is met
  if (MSG->dlc >= message->GetDlc()) {
    message->SetFrame(MSG);
    // TODO(arturo): add some thresholding? -> pub w/ warn if too big?

    imu_msg_out_.angular_velocity.x =
      static_cast<double>(message->GetSignal("YawRate")->GetResult());
    imu_msg_out_.angular_velocity.y =
      static_cast<double>(message->GetSignal("RollRate")->GetResult());
    imu_msg_out_.angular_velocity.z =
      static_cast<double>(message->GetSignal("PitchRate")->GetResult());

    pub_imu_->publish(imu_msg_out_);
  }
}

}  // end namespace microstrain
}  // end namespace drivers

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(drivers::microstrain::MicrostrainMV5CanDriverNode)

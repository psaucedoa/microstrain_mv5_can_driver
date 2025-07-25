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

#ifndef MICROSTRAIN_MV5_CAN_DRIVER__MICROSTRAIN_MV5_CAN_DRIVER_HPP_
#define MICROSTRAIN_MV5_CAN_DRIVER__MICROSTRAIN_MV5_CAN_DRIVER_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "can_msgs/msg/frame.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "j1939_msgs/srv/imu_rename.hpp"
#include "j1939_msgs/srv/imu_reset_attitude.hpp"
#include "j1939_msgs/srv/imu_set_data_rate.hpp"
#include "j1939_msgs/srv/imu_set_orientation.hpp"
#include "j1939_msgs/srv/imu_tare_orientation.hpp"

#include "can_dbc_parser/Dbc.hpp"
#include "can_dbc_parser/DbcBuilder.hpp"
#include "can_dbc_parser/DbcMessage.hpp"
#include "can_dbc_parser/DbcSignal.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

using namespace std::chrono_literals;
using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
namespace rlc = rclcpp_lifecycle;

// TODO(Arturo): There is a way to restart the address claim procedure with a provided new source
// address. This means it could be possible to automate the setup of a new CAN device? Start new
// node -> param for new_device_setup -> iterate through source add to find add
namespace ros2_j1939
{

class MicrostrainMV5CanDriver : public rlc::LifecycleNode
{
public:
  /**
   * @brief Device reference manual:
   * https://www.microstrain.com/sites/default/files/mv5-ar_user_manual_8500-0091_1.pdf
  */
  explicit MicrostrainMV5CanDriver(const rclcpp::NodeOptions & OPTIONS);

  ~MicrostrainMV5CanDriver();

  // These are the can IDs for acceleration, angular rate, and slope can frames, minus device ID
  enum CANIDs
  {
    ID_ACCEL = 0x8F02D00,
    ID_SLOPE = 0xCF02900,
    ID_ANGULAR_RATE = 0xCF02A00,
    ID_NAME = 0x18EEFF00
  };

  /**
   * @brief Configures the driver. Sets up params, creates publishers.
  */
  LNI::CallbackReturn on_configure(const rlc::State & state);

  /**
   * @brief Activates the driver. Activates publishers, configures device.
  */
  LNI::CallbackReturn on_activate(const rlc::State & state);

  /**
   * @brief Deactivates the driver. Deactivates publisher.
  */
  LNI::CallbackReturn on_deactivate(const rlc::State & state);

  /**
   * @brief Performs Cleanup on the driver node. Resets to "as-new" state
  */
  LNI::CallbackReturn on_cleanup(const rlc::State & state);

  /**
   * @brief Shutsdown the driver.
  */
  LNI::CallbackReturn on_shutdown(const rlc::State & state);

  // DATABASE MANAGEMENT FUNCTIONS //
  /**
   * @brief Instantiates a NewEagle dbc database object using the dbc file specified in params.
   * Strips the priority and source address id from the message ID in the DBC.
   * These stripped IDs are stored as keys in a map with the NewEagle DBC messages as values.
   */
  void setupDatabase();

  /**
   * @brief Checks the messages in the DBC and creates a publisher for each one
   * 
   * The publishers are of type "j1939_msgs::msg::CanData" with a topic name folling a
   * "sensor_name/key_message" pattern
   */
  void configurePublishers();

  /**
   * Goes through the configured publishers and activates them
   */
  void activatePublishers();

  /**
   * Goes through the configured publishers and activates them
   */
  void deactivatePublishers();

  /**
   * @brief gets the J1939 NAME of the IMU
   */
  void txGetName();

  /**
   * @brief updates the J1939 NAME of the IMU
   */
  void rxGetName(const can_msgs::msg::Frame::SharedPtr MSG);

  void createDataArray(
    const std::vector<uint16_t> data_in, 
    const std::vector<uint16_t> data_lengths, 
    std::array<uint8_t, 8UL> &data_out
  );

  /**
   * @brief Receives the first frame from a group of three containing IMU data. This frame contains
   * linear acceleration data. The header stamp and frame id are set here.
   * It is stuffed into class member imu_msg_out_
   * @param MSG Standard ROS2 can frame message. J1939
  */
  void rxAccelFrame(const can_msgs::msg::Frame::SharedPtr MSG);

  /**
   * @brief Receives the second frame from a group of three containing IMU data. This frame contains
   * angular rate data. It is stuffed into class member imu_msg_out_
   * @param MSG Standard ROS2 can frame message. J1939
  */
  void rxAngularRateFrame(const can_msgs::msg::Frame::SharedPtr MSG);

  /**
   * @brief Receives the third and final frame from a group of thee. THis frame contains slope data.
   * It is stuffed into class member imu_msg_out_ and published
   * @param MSG Standard ROS2 can frame message. J1939
  */
  void rxSlopeFrame(const can_msgs::msg::Frame::SharedPtr MSG);

  /**
   * @brief Parses incoming CAN frames.
   * 
   * 1. Checks if incoming frame is valid and has a matching device ID (as set in params)
   * 
   * 2. Passes can frame to a local constant
   * 
   * 3. Checks if the message exists in the dbc
   * 
   * 4. Stuffs a CanData value-key message
   * 
   * 5. Publishes that message on the message topic
   */
  void rxFrame(const can_msgs::msg::Frame::SharedPtr MSG);

  /**
   * @brief changes the configuration lock state of the IMU.
   * 
   * @param state The state of the configuration lock
   * False = 0 = unlocked
   * True = 1 = Locked 
   */
  void txLock(const bool state);

  /**
   * @brief renames the device with the given source address
  */
  void txRename(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<j1939_msgs::srv::ImuRename::Request> request, 
    const std::shared_ptr<j1939_msgs::srv::ImuRename::Response> response 
  );

  /**
   * @brief tares the sensor at the current orientation, snapping it to the nearest multiple of
   * the granularity parameters. A 5 degree granularity means tare at 14.68 deg zeros at 15 deg.
   * A granularity of 0 tares at the current position, no rounding.
   * @param roll_granularity_deg Granularity on the roll axis. 1 byte, 1 deg resolution, 0 deg
   * offset, 0 to 90 deg range
   * @param pitch_granularity_deg Granularity on the pitch axis. 1 byte, 1 deg resolution, 0 deg
   * offset, 0 to 90 deg range
   * @param yaw_direction_deg Treated as an offset to current absolute yaw. 1 byte, 1 deg
   * resolution, 0 deg offset, 0 to 360 deg range.
  */
  void txTareOrientation(const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<j1939_msgs::srv::ImuTareOrientation::Request> request, 
    const std::shared_ptr<j1939_msgs::srv::ImuTareOrientation::Response> response 
  );

  /**
   * @brief Sets specific sensor orientation using offsets. 2 bytes per axis, 0.1 deg resolution,
   * -180 deg offset, -180 to 180 deg range
   * @param x_rotation Device x-axis rotation in degrees
   * @param y_rotation Device y-axis rotation in degrees
   * @param z_rotation Device z-axis rotation in degrees
  */
  void txSetOrientation(const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<j1939_msgs::srv::ImuSetOrientation::Request> request, 
    const std::shared_ptr<j1939_msgs::srv::ImuSetOrientation::Response> response 
  );

  /**
   * @brief Resets the attitude of the device through a PGN
  */
  void txResetAttitude(const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<j1939_msgs::srv::ImuResetAttitude::Request> request, 
    const std::shared_ptr<j1939_msgs::srv::ImuResetAttitude::Response> response 
  );

  void txSetDataRate(const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<j1939_msgs::srv::ImuSetDataRate::Request> request, 
    const std::shared_ptr<j1939_msgs::srv::ImuSetDataRate::Response> response 
  );

  /**
   * @brief Force device to pause transmission for specified time. 2 Bytes.
   * @param time_ms Time to silence device, in ms
  */
  void txPause(uint16_t time_ms);

  sensor_msgs::msg::Imu imu_msg_out_;  // buffers incoming IMU data, which is then published

  // publishers
  std::shared_ptr<rlc::LifecyclePublisher<sensor_msgs::msg::Imu>> pub_imu_;  // IMU msg publisher

  // params
  std::string dbw_dbc_file_;  // set in launch file. Files such as MV5.dbc
  std::string frame_id_;      // such as: base, etc.
  std::string sensor_name_;   // such as: /Imu/microstrain/joint_1_L, or w/e
  uint8_t device_ID_;         // such as 226
  NewEagle::Dbc dbw_dbc_db_;   // new eagle dbc database

  std::map<uint32_t , NewEagle::DbcMessage> dbc_id_msg_map_;
  std::map<std::string , NewEagle::DbcMessage> dbc_name_msg_map_;
  // std::map<std::string, std::shared_ptr<rlc::LifecyclePublisher<
  //   j1939_msgs::msg::CanData>>> publishers_;
  std::string device_ID_str_;
  std::string sub_topic_can_;
  std::string pub_topic_can_;
  std::string pub_topic_imu_;

  std::shared_ptr<rlc::LifecyclePublisher<can_msgs::msg::Frame>> pub_can_;  // IMU msg publisher
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr sub_can_;
  rclcpp::Service<j1939_msgs::srv::ImuRename>::SharedPtr rename_service_;
  rclcpp::Service<j1939_msgs::srv::ImuTareOrientation>::SharedPtr tare_service_;
  rclcpp::Service<j1939_msgs::srv::ImuSetOrientation>::SharedPtr set_orientation_service_;
  rclcpp::Service<j1939_msgs::srv::ImuResetAttitude>::SharedPtr reset_attitude_service_;
  rclcpp::Service<j1939_msgs::srv::ImuSetDataRate>::SharedPtr set_data_rate_service_;


  std::array<uint8_t, 8UL> device_name_ {0,0,0,0,0,0,0,0};
  // bool set_new_source_address_;
  // uint8_t new_source_address_;
};

}  // end namespace ros2_j1939

#endif  // MICROSTRAIN_MV5_CAN_DRIVER__MICROSTRAIN_MV5_CAN_DRIVER_NODE_HPP_

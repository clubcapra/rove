// Copyright (c) 2024, capra
// Copyright (c) 2024, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROVE_TRACKS__CHAINED_CONTROLLER_HPP_
#define ROVE_TRACKS__CHAINED_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "chained_controller_parameters.hpp"
#include "rove_tracks/visibility_control.h"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_srvs/srv/set_bool.hpp"

// TODO(anyone): Replace with controller specific messages
#include "control_msgs/msg/joint_controller_state.hpp"
#include "control_msgs/msg/joint_jog.hpp"

namespace rove_tracks
{

enum track_cmd_id : int
{
  TRK_CMD_FRONT_VELOCITY = 0,
  TRK_CMD_BACK_VELOCITY,
  TRK_CMD_FRONT_EFFORT,
  TRK_CMD_BACK_EFFORT,
  TRK_CMD_COUNT,
  TRK_CMD_TRACK_VELOCITY = TRK_CMD_COUNT,
  TRK_CMD_TOTAL_COUNT,
  TRK_CMD_NONE = -1,
};

enum track_state_id : int
{
  TRK_ST_FRONT_POSITION = 0,
  TRK_ST_BACK_POSITION,
  TRK_ST_FRONT_VELOCITY,
  TRK_ST_BACK_VELOCITY,
  TRK_ST_FRONT_EFFORT,
  TRK_ST_BACK_EFFORT,
  TRK_ST_COUNT,
  TRK_ST_TRACK_VELOCITY = TRK_ST_COUNT,
  TRK_ST_TOTAL_COUNT,
  TRK_ST_NONE = -1,
};

// name constants for command interfaces
static constexpr size_t CMD_MY_ITFS = TRK_CMD_COUNT;

// name constants for state interfaces
static constexpr size_t STATE_MY_ITFS = TRK_ST_COUNT;

class ChainedController : public controller_interface::ChainableControllerInterface
{
public:
  ROVE_TRACKS__VISIBILITY_PUBLIC
  ChainedController();

  ROVE_TRACKS__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  ROVE_TRACKS__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  ROVE_TRACKS__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  ROVE_TRACKS__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  ROVE_TRACKS__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  ROVE_TRACKS__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  ROVE_TRACKS__VISIBILITY_PUBLIC
  controller_interface::return_type update_reference_from_subscribers() override;

  ROVE_TRACKS__VISIBILITY_PUBLIC
  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // TODO(anyone): replace the state and command message types
  using ControllerReferenceMsg = control_msgs::msg::JointJog;
  using ControllerModeSrvType = std_srvs::srv::SetBool;
  using ControllerStateMsg = control_msgs::msg::JointControllerState;

protected:
  std::shared_ptr<rove_tracks::ParamListener> param_listener_;
  rove_tracks::Params params_;

  std::string front_joint_;
  std::string back_joint_;
  std::string track_joint_;
  int direction_ = 0;
  int coasting_direction_ = 0;
  const double inrush_torque_ = 0;
  bool caught_up_ = false;

  // Command subscribers and Controller State publisher
  rclcpp::Subscription<ControllerReferenceMsg>::SharedPtr ref_subscriber_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerReferenceMsg>> input_ref_;

  using ControllerStatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;

  rclcpp::Publisher<ControllerStateMsg>::SharedPtr s_publisher_;
  std::unique_ptr<ControllerStatePublisher> state_publisher_;

  // override methods from ChainableControllerInterface
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  bool on_set_chained_mode(bool chained_mode) override;

private:
  // callback for topic interface
  ROVE_TRACKS__VISIBILITY_LOCAL
  void reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg);

  rclcpp::Logger logger() const;
};

}  // namespace rove_tracks

#endif  // ROVE_TRACKS__CHAINED_CONTROLLER_HPP_

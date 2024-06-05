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

#include "rove_tracks/chained_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include "controller_interface/helpers.hpp"

namespace
{  // utility

// TODO(destogl): remove this when merged upstream
// Changed services history QoS to keep all so we don't lose any client service calls
static constexpr rmw_qos_profile_t rmw_qos_profile_services_hist_keep_all = {
  RMW_QOS_POLICY_HISTORY_KEEP_ALL,
  1,  // message queue depth
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false};

using ControllerReferenceMsg = rove_tracks::ChainedController::ControllerReferenceMsg;

// called from RT control loop
void reset_controller_reference_msg(
  const std::shared_ptr<ControllerReferenceMsg> & msg, const std::vector<std::string> & joint_names)
{
  msg->joint_names = joint_names;
  msg->displacements.resize(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
  msg->velocities.resize(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
  msg->duration = std::numeric_limits<double>::quiet_NaN();
}

}  // namespace

namespace rove_tracks
{
ChainedController::ChainedController() : controller_interface::ChainableControllerInterface() {}

controller_interface::CallbackReturn ChainedController::on_init()
{
  try
  {
    param_listener_ = std::make_shared<rove_tracks::ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    
    fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ChainedController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();

  front_joint_ = params_.front_joint;
  back_joint_ = params_.back_joint;
  track_joint_ = params_.track_joint_name;

  // // topics QoS
  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  // // Reference Subscriber
  ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
    "~/reference", subscribers_qos,
    std::bind(&ChainedController::reference_callback, this, std::placeholders::_1));

  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
  reset_controller_reference_msg(msg, {params_.track_joint_name});
  input_ref_.writeFromNonRT(msg);

  try
  {
    // State publisher
    s_publisher_ =
      get_node()->create_publisher<ControllerStateMsg>("~/state", rclcpp::SystemDefaultsQoS());
    state_publisher_ = std::make_unique<ControllerStatePublisher>(s_publisher_);
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // TODO(anyone): Reserve memory in state publisher depending on the message type
  state_publisher_->lock();
  state_publisher_->msg_.header.frame_id = params_.track_joint_name;
  state_publisher_->unlock();

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration ChainedController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // command_interfaces_config.names.reserve(TRK_CMD_TOTAL_COUNT);
  command_interfaces_config.names.reserve(TRK_CMD_COUNT);
  // command_interfaces_config.names.push_back(front_joint_ + "/" + hardware_interface::HW_IF_POSITION);
  // command_interfaces_config.names.push_back(back_joint_ + "/" + hardware_interface::HW_IF_POSITION);
  command_interfaces_config.names.push_back(front_joint_ + "/" + hardware_interface::HW_IF_VELOCITY);
  command_interfaces_config.names.push_back(back_joint_ + "/" + hardware_interface::HW_IF_VELOCITY);
  command_interfaces_config.names.push_back(front_joint_ + "/" + hardware_interface::HW_IF_EFFORT);
  command_interfaces_config.names.push_back(back_joint_ + "/" + hardware_interface::HW_IF_EFFORT);
  // command_interfaces_config.names.push_back(track_joint_ + "/" + hardware_interface::HW_IF_VELOCITY);
  // command_interfaces_config.names.push_back(track_joint_ + "/" + hardware_interface::HW_IF_VELOCITY);

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration ChainedController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // state_interfaces_config.names.reserve(TRK_ST_TOTAL_COUNT);
  state_interfaces_config.names.reserve(TRK_ST_COUNT);
  state_interfaces_config.names.push_back(front_joint_ + "/" + hardware_interface::HW_IF_POSITION);
  state_interfaces_config.names.push_back(back_joint_ + "/" + hardware_interface::HW_IF_POSITION);
  state_interfaces_config.names.push_back(front_joint_ + "/" + hardware_interface::HW_IF_VELOCITY);
  state_interfaces_config.names.push_back(back_joint_ + "/" + hardware_interface::HW_IF_VELOCITY);
  state_interfaces_config.names.push_back(front_joint_ + "/" + hardware_interface::HW_IF_EFFORT);
  state_interfaces_config.names.push_back(back_joint_ + "/" + hardware_interface::HW_IF_EFFORT);
  // state_interfaces_config.names.push_back(track_joint_ + "/" + hardware_interface::HW_IF_VELOCITY);
  // for (const auto & joint : state_joints_)
  // {
  //   state_interfaces_config.names.push_back(joint + "/" + params_.interface_name);
  // }

  return state_interfaces_config;
}

void ChainedController::reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg)
{
  if (msg->joint_names.size() == 1)
  {
    input_ref_.writeFromNonRT(msg);
  }
  else
  {
    RCLCPP_ERROR(
      logger(),
      "Received %zu , but expected %zu joints in command. Ignoring message.",
      msg->joint_names.size(), 1);
  }
}


rclcpp::Logger ChainedController::logger() const
{
  return get_node()->get_logger();
}

std::vector<hardware_interface::CommandInterface> ChainedController::on_export_reference_interfaces()
{
  // reference_interfaces_.resize(state_joints_.size(), std::numeric_limits<double>::quiet_NaN());
  reference_interfaces_.resize(TRK_CMD_TOTAL_COUNT - TRK_CMD_COUNT, std::numeric_limits<double>::quiet_NaN());
  

  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  reference_interfaces.reserve(reference_interfaces_.size());
  RCLCPP_INFO(logger(), "Interface count: %d", reference_interfaces_.size());

  reference_interfaces.push_back(hardware_interface::CommandInterface(get_node()->get_name(), track_joint_ + "/" + hardware_interface::HW_IF_VELOCITY, 
    &reference_interfaces_[0]));
  RCLCPP_INFO(logger(), "Interface exported: %d", reference_interfaces.size());

  // for (size_t i = 0; i < reference_interfaces_.size(); ++i)
  // {
  //   reference_interfaces.push_back(hardware_interface::CommandInterface(
  //     get_node()->get_name(), state_joints_[i] + "/" + params_.interface_name,
  //     &reference_interfaces_[i]));
  // }

  return reference_interfaces;
}

bool ChainedController::on_set_chained_mode(bool chained_mode)
{
  RCLCPP_INFO(logger(), "Set chained mode to: %d", chained_mode);
  // Always accept switch to/from chained mode
  return true || chained_mode;
}

controller_interface::CallbackReturn ChainedController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Set default value in command
  reset_controller_reference_msg(*(input_ref_.readFromRT()), {track_joint_});

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ChainedController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): depending on number of interfaces, use definitions, e.g., `CMD_MY_ITFS`,
  // instead of a loop
  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    command_interfaces_[i].set_value(std::numeric_limits<double>::quiet_NaN());
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ChainedController::update_reference_from_subscribers()
{
  auto current_ref = input_ref_.readFromRT();

  // TODO(anyone): depending on number of interfaces, use definitions, e.g., `CMD_MY_ITFS`,
  // instead of a loop
  for (size_t i = 0; i < reference_interfaces_.size(); ++i)
  {
    if (!std::isnan((*current_ref)->displacements[i]))
    {
      reference_interfaces_[i] = (*current_ref)->displacements[i];

      (*current_ref)->displacements[i] = std::numeric_limits<double>::quiet_NaN();
    }
  }
  return controller_interface::return_type::OK;
}

controller_interface::return_type ChainedController::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  static rclcpp::Clock clk{};
  if (!std::isnan(reference_interfaces_[0]))
  {
    RCLCPP_INFO_STREAM_THROTTLE(logger(), clk, 1000, "Ref value: " << reference_interfaces_[0]);
    direction_ = reference_interfaces_[0] == 0 ? 0 : (
      reference_interfaces_[0] < 0 ? -1 : 1
    );
  }

  track_cmd_id leading_cmd = track_cmd_id::TRK_CMD_NONE;
  track_cmd_id trailing_cmd = track_cmd_id::TRK_CMD_NONE;
  track_state_id leading_st_velocity = track_state_id::TRK_ST_NONE;
  track_state_id leading_st_effort = track_state_id::TRK_ST_NONE;

  switch (direction_)
  {
  case -1:
    leading_cmd = track_cmd_id::TRK_CMD_BACK_VELOCITY;
    trailing_cmd = track_cmd_id::TRK_CMD_FRONT_EFFORT;
    leading_st_velocity = track_state_id::TRK_ST_BACK_VELOCITY;
    leading_st_effort = track_state_id::TRK_ST_BACK_EFFORT;
    break;
  case 0:
    if (coasting_direction_ < 0)
    {
      leading_cmd = track_cmd_id::TRK_CMD_BACK_VELOCITY;
      trailing_cmd = track_cmd_id::TRK_CMD_FRONT_EFFORT;
      leading_st_velocity = track_state_id::TRK_ST_BACK_VELOCITY;
      leading_st_effort = track_state_id::TRK_ST_BACK_EFFORT;
    }
    if (coasting_direction_ > 0)
    {
      leading_cmd = track_cmd_id::TRK_CMD_FRONT_VELOCITY;
      trailing_cmd = track_cmd_id::TRK_CMD_BACK_EFFORT;
      leading_st_velocity = track_state_id::TRK_ST_FRONT_VELOCITY;
      leading_st_effort = track_state_id::TRK_ST_FRONT_EFFORT;
    }
    break;
  case 1:
    leading_cmd = track_cmd_id::TRK_CMD_FRONT_VELOCITY;
    trailing_cmd = track_cmd_id::TRK_CMD_BACK_EFFORT;
    leading_st_velocity = track_state_id::TRK_ST_FRONT_VELOCITY;
    leading_st_effort = track_state_id::TRK_ST_FRONT_EFFORT;
    break;
  default:
    leading_cmd = track_cmd_id::TRK_CMD_NONE;
    trailing_cmd = track_cmd_id::TRK_CMD_NONE;
    leading_st_velocity = track_state_id::TRK_ST_NONE;
    leading_st_effort = track_state_id::TRK_ST_NONE;
    break;
  }

  if (!std::isnan(reference_interfaces_[0]))
  {
    command_interfaces_[leading_cmd].set_value(reference_interfaces_[0]);
    reference_interfaces_[0] = std::numeric_limits<double>::quiet_NaN();
  }

  double leading_vel = 0;
  double leading_torque = 0;
  if (leading_st_velocity != track_state_id::TRK_ST_NONE)
  {
    leading_vel = state_interfaces_[leading_st_velocity].get_value();
    coasting_direction_ = /*TODO close to*/leading_vel == 0 ? 0 : (
      leading_vel > 0 ? 1 : -1
    );
  }

  if (leading_st_effort != track_state_id::TRK_ST_NONE)
  {
    leading_torque = state_interfaces_[leading_st_effort].get_value();
  }

  if (coasting_direction_ < direction_ || coasting_direction_ > direction_) caught_up_ = false;
  if (abs(leading_torque) > inrush_torque_) caught_up_ = true;

  double torque = caught_up_ ? leading_torque : (std::min(inrush_torque_, abs(leading_torque)) * coasting_direction_);

  if (coasting_direction_ != 0)
  {
    command_interfaces_[trailing_cmd].set_value(torque);
  }
  RCLCPP_INFO_STREAM_THROTTLE(logger(), clk, 1000, "Ref value: " << reference_interfaces_[0] <<
    " Direction: " << direction_ <<
    " Leading vel: " << leading_vel <<
    " Leading torque: " << leading_torque <<
    " Coasting direction: " << coasting_direction_);

  return controller_interface::return_type::OK;
}

} // namespace rove_tracks

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  rove_tracks::ChainedController, controller_interface::ChainableControllerInterface)

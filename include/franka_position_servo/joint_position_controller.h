/***************************************************************************

*
* @package: franka_position_servo
* @metapackage: franka_ros_interface
* @author: Saif Sidhik <sxs1412@bham.ac.uk>
*

**************************************************************************/

/***************************************************************************
* Copyright (c) 2019-2020, Saif Sidhik.
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
**************************************************************************/
#pragma once

#include <array>
#include <string>
#include <vector>

#include <dynamic_reconfigure/server.h>
#include <franka_position_servo/joint_controller_paramsConfig.h>

#include <franka_position_servo/JointCommand.h>
// #include <franka_position_servo/JointControllerStates.h>
#include <franka_position_servo/JointLimits.h>
#include <franka_position_servo/RobotState.h>

#include <mutex>
#include <franka_hw/trigger_rate.h>
#include <realtime_tools/realtime_publisher.h>

#include <controller_interface/multi_interface_controller.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <franka/robot_state.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Dense>


namespace franka_position_servo {

class PositionJointPositionController : public controller_interface::MultiInterfaceController<
                                           hardware_interface::PositionJointInterface, 
                                           franka_hw::FrankaStateInterface,
                                           franka_hw::FrankaModelInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  hardware_interface::PositionJointInterface* position_joint_interface_;
  std::vector<hardware_interface::JointHandle> position_joint_handles_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;

  std::array<double, 7> initial_pos_{};
  std::array<double, 7> prev_pos_{};
  std::array<double, 7> pos_d_target_{};
  std::array<double, 7> pos_d_;

  // joint_cmd subscriber
  ros::Subscriber desired_joints_subscriber_;

  double filter_joint_pos_{0.3};
  double target_filter_joint_pos_{0.1};
  double filter_factor_{0.01};

  double param_change_filter_{0.005};

  Eigen::Vector3d position_d_;
  Eigen::Quaterniond orientation_d_;



  franka_position_servo::JointLimits joint_limits_;
  
  // Dynamic reconfigure
  std::unique_ptr< dynamic_reconfigure::Server<franka_position_servo::joint_controller_paramsConfig> > dynamic_server_joint_controller_params_;
  ros::NodeHandle dynamic_reconfigure_joint_controller_params_node_;

  franka_hw::TriggerRate trigger_publish_, trigger_publish_force_, trigger_publish_pose_;
//   realtime_tools::RealtimePublisher<franka_position_servo::JointControllerStates> publisher_controller_states_;
  realtime_tools::RealtimePublisher<franka_position_servo::RobotState> publisher_robot_states_;
  realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped> publisher_forces_;
  realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped> publisher_eefpose_;

  bool checkPositionLimits(std::vector<double> positions);


  void jointControllerParamCallback(franka_position_servo::joint_controller_paramsConfig& config,
                               uint32_t level);
  void jointPosCmdCallback(const franka_position_servo::JointCommandConstPtr& msg);
};

}  // namespace franka_position_servo

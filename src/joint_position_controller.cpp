
#include <franka_position_servo/joint_position_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace franka_position_servo {

bool PositionJointPositionController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {

  desired_joints_subscriber_ = node_handle.subscribe(
      "/motion_controller/arm/joint_commands", 20, &PositionJointPositionController::jointPosCmdCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());


  auto* model_interface = robot_hardware->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM("JointPositionController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle("panda_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "JointPositionController: Exception getting model handle from interface: " << ex.what());
    return false;
  }

  position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();
  if (position_joint_interface_ == nullptr) {
    ROS_ERROR(
        "PositionJointPositionController: Error getting position joint interface from hardware!");
    return false;
  }
  if (!node_handle.getParam("/robot_config/joint_names", joint_limits_.joint_names)) {
    ROS_ERROR("PositionJointPositionController: Could not parse joint names");
  }
  if (joint_limits_.joint_names.size() != 7) {
    ROS_ERROR_STREAM("PositionJointPositionController: Wrong number of joint names, got "
                     << joint_limits_.joint_names.size() << " instead of 7 names!");
    return false;
  }
  std::map<std::string, double> pos_limit_lower_map;
  std::map<std::string, double> pos_limit_upper_map;
  if (!node_handle.getParam("/robot_config/joint_config/joint_position_limit/lower", pos_limit_lower_map) ) {
  ROS_ERROR(
      "LOWER PositionJointPositionController: Joint limits parameters not provided, aborting "
      "controller init!");
  return false;
      }
  if (!node_handle.getParam("/robot_config/joint_config/joint_position_limit/upper", pos_limit_upper_map) ) {
  ROS_ERROR(
      "UPPER PositionJointPositionController: Joint limits parameters not provided, aborting "
      "controller init!");
  return false;
      }

  for (size_t i = 0; i < joint_limits_.joint_names.size(); ++i){
    if (pos_limit_lower_map.find(joint_limits_.joint_names[i]) != pos_limit_lower_map.end())
      {
        joint_limits_.position_lower.push_back(pos_limit_lower_map[joint_limits_.joint_names[i]]);
      }
      else
      {
        ROS_ERROR("PositionJointPositionController: Unable to find lower position limit values for joint %s...",
                       joint_limits_.joint_names[i].c_str());
      }
    if (pos_limit_upper_map.find(joint_limits_.joint_names[i]) != pos_limit_upper_map.end())
      {
        joint_limits_.position_upper.push_back(pos_limit_upper_map[joint_limits_.joint_names[i]]);
      }
      else
      {
        ROS_ERROR("PositionJointPositionController: Unable to find upper position limit  values for joint %s...",
                       joint_limits_.joint_names[i].c_str());
      }
  }   

  position_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      position_joint_handles_[i] = position_joint_interface_->getHandle(joint_limits_.joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "PositionJointPositionController: Exception getting joint handles: " << e.what());
      return false;
    }
  }

  double controller_state_publish_rate(30.0);
  if (!node_handle.getParam("controller_state_publish_rate", controller_state_publish_rate)) {
    ROS_INFO_STREAM("PositionJointPositionController: Did not find controller_state_publish_rate. Using default "
                    << controller_state_publish_rate << " [Hz].");
  }
  trigger_publish_ = franka_hw::TriggerRate(controller_state_publish_rate);


  double force_publish_rate(500.0);
  trigger_publish_force_ = franka_hw::TriggerRate(force_publish_rate);

  trigger_publish_pose_ = franka_hw::TriggerRate(force_publish_rate);

  auto* state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("joint_position_controller: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle("panda_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "joint_position_controller: Exception getting state handle from interface: " << ex.what());
    return false;
  }



  dynamic_reconfigure_joint_controller_params_node_ =
      ros::NodeHandle("/position_joint_position_controller/arm/controller_parameters_config");

  dynamic_server_joint_controller_params_ = std::make_unique<
      dynamic_reconfigure::Server<franka_position_servo::joint_controller_paramsConfig>>(
      dynamic_reconfigure_joint_controller_params_node_);

  dynamic_server_joint_controller_params_->setCallback(
      boost::bind(&PositionJointPositionController::jointControllerParamCallback, this, _1, _2));

//   publisher_controller_states_.init(node_handle, "/motion_controller/arm/joint_controller_states", 1);

//   {
//     std::lock_guard<realtime_tools::RealtimePublisher<franka_position_servo::JointControllerStates> > lock(
//         publisher_controller_states_);
//     publisher_controller_states_.msg_.controller_name = "position_joint_position_controller";
//     publisher_controller_states_.msg_.names.resize(joint_limits_.joint_names.size());
//     publisher_controller_states_.msg_.joint_controller_states.resize(joint_limits_.joint_names.size());

//   }

  std::array<double, 42> O_Jac_EE =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  publisher_robot_states_.init(node_handle, "/franka_state_controller/robot_state", 1);

  {
    std::lock_guard<realtime_tools::RealtimePublisher<franka_position_servo::RobotState> > lock(
        publisher_robot_states_);
    for (size_t i = 0; i < O_Jac_EE.size(); i++){
        publisher_robot_states_.msg_.O_Jac_EE[i] = O_Jac_EE[i];
    }

  }

  publisher_forces_.init(node_handle, "/franka_state_controller/forces_est", 1);
  {
    std::lock_guard<realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped> > lock(
        publisher_forces_);
        publisher_forces_.msg_.wrench.force.x = 0.;
        publisher_forces_.msg_.wrench.force.y = 0.;
        publisher_forces_.msg_.wrench.force.z = 0.;

        publisher_forces_.msg_.wrench.torque.x = 0.;
        publisher_forces_.msg_.wrench.torque.y = 0.;
        publisher_forces_.msg_.wrench.torque.z = 0.;

  }


  publisher_eefpose_.init(node_handle, "/franka_state_controller/eef_pose", 1);
  {
    std::lock_guard<realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped> > lock(
        publisher_eefpose_);
        publisher_eefpose_.msg_.pose.position.x = 0.;
        publisher_eefpose_.msg_.pose.position.y = 0.;
        publisher_eefpose_.msg_.pose.position.z = 0.;

        publisher_eefpose_.msg_.pose.orientation.x = 0.;
        publisher_eefpose_.msg_.pose.orientation.y = 0.;
        publisher_eefpose_.msg_.pose.orientation.z = 0.;
        publisher_eefpose_.msg_.pose.orientation.w = 0.;

  }



  return true;
}

void PositionJointPositionController::starting(const ros::Time& /* time */) {
  for (size_t i = 0; i < 7; ++i) {
    initial_pos_[i] = position_joint_handles_[i].getPosition();
  }
  pos_d_ = initial_pos_;
  prev_pos_ = initial_pos_;
  pos_d_target_ = initial_pos_;

  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 6> force_meas_array = robot_state.O_F_ext_hat_K;

//   std::cout << force_meas_array << std::endl;
}

void PositionJointPositionController::update(const ros::Time& time,
                                            const ros::Duration& period) {
  for (size_t i = 0; i < 7; ++i) {
    position_joint_handles_[i].setCommand(pos_d_[i]);
  }
  double filter_val = filter_joint_pos_ * filter_factor_;
  for (size_t i = 0; i < 7; ++i) {
    prev_pos_[i] = position_joint_handles_[i].getPosition();
    pos_d_[i] = filter_val * pos_d_target_[i] + (1.0 - filter_val) * pos_d_[i];
  }

  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 6> force_meas_array = robot_state.O_F_ext_hat_K;

//   if (trigger_publish_() && publisher_controller_states_.trylock()) {
//     for (size_t i = 0; i < 7; ++i){

//       publisher_controller_states_.msg_.joint_controller_states[i].set_point = pos_d_target_[i];
//       publisher_controller_states_.msg_.joint_controller_states[i].process_value = pos_d_[i];
//       publisher_controller_states_.msg_.joint_controller_states[i].time_step = period.toSec();

//       publisher_controller_states_.msg_.joint_controller_states[i].header.stamp = time;

//     }

//     publisher_controller_states_.unlockAndPublish();        
//   }



  Eigen::Affine3d curr_transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  position_d_ = curr_transform.translation();
  orientation_d_ = Eigen::Quaterniond(curr_transform.linear());

  std::array<double, 42> O_Jac_EE =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  if (trigger_publish_() && publisher_robot_states_.trylock())
  {
      for (size_t i = 0; i < O_Jac_EE.size(); i++){
          publisher_robot_states_.msg_.O_Jac_EE[i] = O_Jac_EE[i];
      }
          publisher_robot_states_.unlockAndPublish();        
  }



  if (trigger_publish_force_() && publisher_forces_.trylock() )
  {
        publisher_forces_.msg_.header.stamp = ros::Time::now();
        publisher_forces_.msg_.wrench.force.x = force_meas_array[0];
        publisher_forces_.msg_.wrench.force.y = force_meas_array[1];
        publisher_forces_.msg_.wrench.force.z = force_meas_array[2];

        publisher_forces_.msg_.wrench.torque.x = force_meas_array[3];
        publisher_forces_.msg_.wrench.torque.y = force_meas_array[4];
        publisher_forces_.msg_.wrench.torque.z = force_meas_array[5];

        publisher_forces_.unlockAndPublish();        

  }

  if (trigger_publish_pose_() && publisher_eefpose_.trylock() )
  {
        publisher_eefpose_.msg_.header.stamp = ros::Time::now();
        publisher_eefpose_.msg_.pose.position.x = position_d_[0];
        publisher_eefpose_.msg_.pose.position.y = position_d_[1];
        publisher_eefpose_.msg_.pose.position.z = position_d_[2];

        publisher_eefpose_.msg_.pose.orientation.x = orientation_d_.x();
        publisher_eefpose_.msg_.pose.orientation.y = orientation_d_.y();
        publisher_eefpose_.msg_.pose.orientation.z = orientation_d_.z();
        publisher_eefpose_.msg_.pose.orientation.w = orientation_d_.w();

        
        publisher_eefpose_.unlockAndPublish();        

  }

  // update parameters changed online either through dynamic reconfigure or through the interactive
  // target by filtering
  filter_joint_pos_ = param_change_filter_ * target_filter_joint_pos_ + (1.0 - param_change_filter_) * filter_joint_pos_;

}

bool PositionJointPositionController::checkPositionLimits(std::vector<double> positions)
{
  // bool retval = true;
  for (size_t i = 0;  i < 7; ++i){
    if (!((positions[i] <= joint_limits_.position_upper[i]) && (positions[i] >= joint_limits_.position_lower[i]))){
      return true;
    }
  }

  return false;
}

void PositionJointPositionController::jointPosCmdCallback(const franka_position_servo::JointCommandConstPtr& msg) {

    if (msg->mode == franka_position_servo::JointCommand::POSITION_MODE){
      if (msg->position.size() != 7) {
        ROS_ERROR_STREAM(
            "PositionJointPositionController: Published Commands are not of size 7");
        pos_d_ = prev_pos_;
        pos_d_target_ = prev_pos_;
      }
      else if (checkPositionLimits(msg->position)) {
         ROS_ERROR_STREAM(
            "PositionJointPositionController: Commanded positions are beyond allowed position limits.");
        pos_d_ = prev_pos_;
        pos_d_target_ = prev_pos_;

      }
      else
      {
        std::copy_n(msg->position.begin(), 7, pos_d_target_.begin());
      }
      
    }
    // else ROS_ERROR_STREAM("PositionJointPositionController: Published Command msg are not of JointCommand::POSITION_MODE! Dropping message");
}

void PositionJointPositionController::jointControllerParamCallback(franka_position_servo::joint_controller_paramsConfig& config,
                               uint32_t level){
  target_filter_joint_pos_ = config.position_joint_delta_filter;
}

}  // namespace franka_position_servo

PLUGINLIB_EXPORT_CLASS(franka_position_servo::PositionJointPositionController,
                       controller_interface::ControllerBase)

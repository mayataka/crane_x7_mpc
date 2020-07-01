#include "mpc_controller.hpp"

#include "pluginlib/class_list_macros.h"


namespace cranex7mpc {

MPCController::MPCController() 
  : urdf_path_("/home/sotaro/catkin_ws/src/crane_x7_mpc/crane_x7_mpc/IDOCP/examples/cranex7/crane_x7_description/urdf/crane_x7.urdf"),
    robot_(urdf_path_),
    cost_(robot_),
    constraints_(robot_),
    mpc_(robot_, &cost_, &constraints_, T_, N_, num_proc_),
    dimq_(robot_.dimq()),
    dimv_(robot_.dimv()),
    q_(Eigen::VectorXd::Zero(robot_.dimq())),
    v_(Eigen::VectorXd::Zero(robot_.dimq())),
    u_(Eigen::VectorXd::Zero(robot_.dimq())),
    q_ref_(Eigen::VectorXd::Zero(robot_.dimq())) {
  robot_.set_joint_damping(Eigen::VectorXd::Constant(robot_.dimq(), 1.0e-06));
}


bool MPCController::init(hardware_interface::EffortJointInterface* hardware, 
                         ros::NodeHandle &node_handle) {
  std::vector<std::string> joint_names = {
      "crane_x7_shoulder_fixed_part_pan_joint",
      "crane_x7_shoulder_revolute_part_tilt_joint",
      "crane_x7_upper_arm_revolute_part_twist_joint",
      "crane_x7_upper_arm_revolute_part_rotate_joint",
      "crane_x7_lower_arm_fixed_part_joint",
      "crane_x7_lower_arm_revolute_part_joint",
      "crane_x7_wrist_joint"};
  if (!joint_effort_handlers_.empty()) {
    return false;
  }
  for (const auto& joint_name : joint_names) {
    joint_effort_handlers_.push_back(hardware->getHandle(joint_name));
  }
  service_server_ = node_handle.advertiseService(
      "/crane_x7/mpc_nodelet/set_goal_configuration", 
      &cranex7mpc::MPCController::setGoalConfiguration, this);
  return true;
}


void MPCController::starting(const ros::Time& time) {
  q_ref_ = Eigen::VectorXd::Zero(robot_.dimq());
  cost_.set_q_ref(q_ref_);
  ROS_INFO("start MPC controller!!");
}


void MPCController::stopping(const ros::Time& time) {
  ROS_INFO("stop MPC controller!!");
}


bool MPCController::setGoalConfiguration(
    crane_x7_mpc::SetGoalConfiguration::Request& request, 
    crane_x7_mpc::SetGoalConfiguration::Response& response) {
  ROS_INFO("set goal configuration!!");
  for (int i=0; i<dimq_; ++i) {
    q_ref_.coeffRef(i) = request.goal_configuration[i];
  }
  cost_.set_q_ref(q_ref_);
  response.success = true;
  return true;
}


void MPCController::update(const ros::Time& time, const ros::Duration& period) {
  for (int i=0; i<dimq_; ++i) {
    q_.coeffRef(i) = joint_effort_handlers_[i].getPosition();
    v_.coeffRef(i) = joint_effort_handlers_[i].getVelocity();
  }
  const double t = time.toSec();
  ROS_INFO("KKT error = %lf", mpc_.KKTError(t, q_, v_));
  mpc_.updateSolution(t, q_, v_);
  mpc_.getControlInput(u_);
  for (int i=0; i<dimv_; ++i) {
    joint_effort_handlers_[i].setCommand(u_.coeff(i));
  }
}

} // namespace crane_x7_mpc 


PLUGINLIB_EXPORT_CLASS(cranex7mpc::MPCController, 
                       controller_interface::ControllerBase)
#include "mpc_nodelet.hpp"

#include "pluginlib/class_list_macros.h"


namespace cranex7mpc {

MPCNodelet::MPCNodelet() 
  : urdf_path_("/home/sotaro/ros1_ws/src/crane_x7_mpc/crane_x7_mpc/IDOCP/examples/cranex7/crane_x7_description/urdf/crane_x7.urdf"),
    robot_(urdf_path_),
    cost_(robot_),
    constraints_(robot_),
    mpc_(robot_, &cost_, &constraints_, T_, N_, num_proc_),
    dimq_(robot_.dimq()),
    dimv_(robot_.dimv()),
    q_(Eigen::VectorXd::Zero(robot_.dimq())),
    v_(Eigen::VectorXd::Zero(robot_.dimq())),
    u_(Eigen::VectorXd::Zero(robot_.dimq())) {
}


void MPCNodelet::onInit() {
}


bool MPCNodelet::init(hardware_interface::EffortJointInterface* hardware, 
                      ros::NodeHandle &node_handler) {
  ROS_INFO("init controller!!");
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
  return true;
}


void MPCNodelet::starting(const ros::Time& time) {
}


void MPCNodelet::stopping(const ros::Time& time) {
}


void MPCNodelet::update(const ros::Time& time, const ros::Duration& period) {
  for (int i=0; i<dimq_; ++i) {
    q_.coeffRef(i) = joint_effort_handlers_[i].getPosition();
    v_.coeffRef(i) = joint_effort_handlers_[i].getVelocity();
  }
  const double t = time.toSec();
  mpc_.updateSolution(t, q_, v_);
  mpc_.getControlInput(u_);
  for (int i=0; i<dimv_; ++i) {
    joint_effort_handlers_[i].setCommand(u_[i]);
  }
}

} // namespace crane_x7_mpc 


PLUGINLIB_EXPORT_CLASS(cranex7mpc::MPCNodelet, nodelet::Nodelet)
PLUGINLIB_EXPORT_CLASS(
    cranex7mpc::MPCNodelet, 
    controller_interface::ControllerBase)
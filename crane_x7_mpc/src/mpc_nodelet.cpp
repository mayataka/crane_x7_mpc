#include "mpc_nodelet.hpp"

#include "pluginlib/class_list_macros.h"


namespace cranex7mpc {
MPCNodelet::MPCNodelet() {
  ROS_INFO("SampleNodeletClass Constructor");
}

void MPCNodelet::onInit()
{
    NODELET_INFO("SampleNodeletClass - %s", __FUNCTION__);
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
  ROS_INFO("start controller!!");
}

void MPCNodelet::stopping(const ros::Time& time) {
  ROS_INFO("stop controller!!");
}

void MPCNodelet::update(const ros::Time& time, const ros::Duration& period) {
  // for (int i=0; i<dimq_; ++i) {
  //   q_.coeffRef(i) = joint_effort_handlers_[i].getPosition();
  //   v_.coeffRef(i) = joint_effort_handlers_[i].getVelocity();
  // }
  double q_[7], v_[7];
  for (int i=0; i<7; ++i) {
    q_[i] = joint_effort_handlers_[i].getPosition();
    v_[i] = joint_effort_handlers_[i].getVelocity();
    joint_effort_handlers_[i].setCommand(4);
  }
  // ROS_INFO("q =  [%lf %lf %lf %lf %lf %lf %lf]", q_[0], q_[1], q_[2], q_[3], q_[4], q_[5], q_[6]);
  // ROS_INFO("v =  [%lf %lf %lf %lf %lf %lf %lf]", v_[0], v_[1], v_[2], v_[3], v_[4], v_[5], v_[6]);
}

} // namespace crane_x7_mpc 


PLUGINLIB_EXPORT_CLASS(cranex7mpc::MPCNodelet, nodelet::Nodelet)
PLUGINLIB_EXPORT_CLASS(
    cranex7mpc::MPCNodelet, 
    // controller_interface::Controller<hardware_interface::EffortJointInterface>)
    controller_interface::ControllerBase)
// PLUGINLIB_DECLARE_CLASS(crane_x7_mpc, cranex7mpc::MPCNodelet, cranex7mpc::MPCNodelet, controller_interface::ControllerBase)
// PLUGINLIB_DECLARE_CLASS(package_name, PositionController, controller_ns::PositionController, controller_interface::ControllerBase);
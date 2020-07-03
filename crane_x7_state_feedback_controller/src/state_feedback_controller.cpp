#include "state_feedback_controller.hpp"

#include "pluginlib/class_list_macros.h"


namespace cranex7mpc {

StateFeedbackController::StateFeedbackController(const unsigned int dimq, 
                                                 const unsigned int dimv)
  : dimq_(dimq),
    dimv_(dimv),
    q_stn_(Eigen::VectorXd::Zero(dimq)),
    v_stn_(Eigen::VectorXd::Zero(dimv)),
    u_stn_(Eigen::VectorXd::Zero(dimv)),
    q_(Eigen::VectorXd::Zero(dimq)),
    v_(Eigen::VectorXd::Zero(dimv)),
    u_(Eigen::VectorXd::Zero(dimv)),
    Kq_(Eigen::MatrixXd::Zero(dimv, dimv)),
    Kv_(Eigen::MatrixXd::Zero(dimv, dimv)) {
  control_input_policy_subscriber_ = node_handle_.subscribe(
      "/crane_x7/mpc_nodelet/control_input_policy", 10, 
      &cranex7mpc::StateFeedbackController::subscribeControlInputPolicy, this);
}


bool StateFeedbackController::init(
    hardware_interface::EffortJointInterface* hardware, 
    ros::NodeHandle &node_handle) {
  std::vector<std::string> joint_names = {
      "crane_x7_shoulder_fixed_part_pan_joint",
      "crane_x7_shoulder_revolute_part_tilt_joint",
      "crane_x7_upper_arm_revolute_part_twist_joint",
      "crane_x7_upper_arm_revolute_part_rotate_joint",
      "crane_x7_lower_arm_fixed_part_joint",
      "crane_x7_lower_arm_revolute_part_joint",
      "crane_x7_wrist_joint"};
  if (!joint_handlers_.empty()) {
    return false;
  }
  for (const auto& joint_name : joint_names) {
    joint_handlers_.push_back(hardware->getHandle(joint_name));
  }
  return true;
}


void StateFeedbackController::starting(const ros::Time& time) {
}


void StateFeedbackController::stopping(const ros::Time& time) {
}


void StateFeedbackController::update(const ros::Time& time, 
                                     const ros::Duration& period) { 
  for (int i=0; i<dimq_; ++i) {
    q_.coeffRef(i) = joint_handlers_[i].getPosition();
    v_.coeffRef(i) = joint_handlers_[i].getVelocity();
  }
  u_ = u_stn_; 
  u_.alias() += Kq_ * (q_-q_stn_);
  u_.alias() += Kv_ * (v_-v_stn_);
  for (int i=0; i<dimv_; ++i) {
    joint_handlers_[i].setCommand(u_.coeff(i));
  }
}


void StateFeedbackController::subscribeControlInputPolicy(
    const sensor_msgs::JointState& joint_state_msg) {
  q_stn_ = ;
  v_stn_ = ;
  u_stn_ = ;
  Kq_ = ;
  Kv_ = ;
}

} // namespace crane_x7_mpc 


PLUGINLIB_EXPORT_CLASS(cranex7mpc::StateFeedbackController, 
                       controller_interface::ControllerBase)
#include "state_feedback_controller.hpp"

#include "pluginlib/class_list_macros.h"


namespace crane_x7_mpc {

StateFeedbackController::StateFeedbackController()
  : q_stn_(),
    v_stn_(),
    u_stn_(),
    q_(),
    v_(),
    u_(),
    Kq_(),
    Kv_() {
  q_stn_.setZero();
  v_stn_.setZero();
  u_stn_.setZero();
  q_.setZero();
  v_.setZero();
  u_.setZero();
  Kq_.setZero();
  Kv_.setZero();
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
  control_input_policy_subscriber_ = node_handle.subscribe(
      "/crane_x7/crane_x7_mpc/control_input_policy", 10, 
      &crane_x7_mpc::StateFeedbackController::subscribeControlInputPolicy, this);
  return true;
}


void StateFeedbackController::starting(const ros::Time& time) {
}


void StateFeedbackController::stopping(const ros::Time& time) {
}


void StateFeedbackController::update(const ros::Time& time, 
                                     const ros::Duration& period) { 
  for (int i=0; i<kDimq; ++i) {
    q_.coeffRef(i) = joint_handlers_[i].getPosition();
    v_.coeffRef(i) = joint_handlers_[i].getVelocity();
  }
  u_ = u_stn_; 
  u_.noalias() += Kq_ * (q_-q_stn_);
  u_.noalias() += Kv_ * (v_-v_stn_);
  // u_.setZero();
  for (int i=0; i<kDimq; ++i) {
    joint_handlers_[i].setCommand(u_.coeff(i));
  }
}


void StateFeedbackController::subscribeControlInputPolicy(
    const crane_x7_msgs::ControlInputPolicy& policy) {
  q_stn_ = Eigen::Map<const Eigen::Matrix<double, kDimq, 1>>(&policy.q[0]);
  v_stn_ = Eigen::Map<const Eigen::Matrix<double, kDimq, 1>>(&policy.v[0]);
  u_stn_ = Eigen::Map<const Eigen::Matrix<double, kDimq, 1>>(&policy.u[0]);
  Kq_ = Eigen::Map<const Eigen::Matrix<double, kDimq, kDimq>>(&policy.Kq[0]);
  Kv_ = Eigen::Map<const Eigen::Matrix<double, kDimq, kDimq>>(&policy.Kv[0]);
}

} // namespace crane_x7_mpc 


PLUGINLIB_EXPORT_CLASS(crane_x7_mpc::StateFeedbackController, 
                       controller_interface::ControllerBase)
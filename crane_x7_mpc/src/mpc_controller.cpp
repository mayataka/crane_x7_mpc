#include "mpc_controller.hpp"

#include "pluginlib/class_list_macros.h"


namespace cranex7mpc {

MPCController::MPCController() 
  : urdf_path_("/home/sotaro/ros1_ws/src/crane_x7_mpc/crane_x7_mpc/IDOCP/examples/cranex7/crane_x7_description/urdf/crane_x7.urdf"),
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
  for (int i=0; i<robot_.dimv(); ++i) {
    joint_efforts_.data.push_back(0);
  }
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


void MPCController::onInit() {
  node_handle_ = getNodeHandle();
  service_server_ = node_handle_.advertiseService(
      "/crane_x7/mpc_nodelet/set_goal_configuration", 
      &cranex7mpc::MPCController::setGoalConfiguration, this);
  joint_state_subscriber_ = node_handle_.subscribe(
      "/crane_x7/joint_states", 10, 
      &MPCController::subscribeJointState, this);
  timer_ = node_handle_.createTimer(ros::Duration(0.001), &cranex7mpc::MPCController::updateControlInput, this);
  joint_efforts_publisher_ 
      = node_handle_.advertise<std_msgs::Float64MultiArray>("/mpc_arm_controller/command", 10);
}


void MPCController::subscribeJointState(
    const sensor_msgs::JointState& joint_state_msg) {
  q_.coeffRef(0) = joint_state_msg.position[3];
  q_.coeffRef(1) = joint_state_msg.position[4];
  q_.coeffRef(2) = joint_state_msg.position[6];
  q_.coeffRef(3) = joint_state_msg.position[5];
  q_.coeffRef(4) = joint_state_msg.position[1];
  q_.coeffRef(5) = joint_state_msg.position[2];
  q_.coeffRef(6) = joint_state_msg.position[7];
  v_.coeffRef(0) = joint_state_msg.velocity[3];
  v_.coeffRef(1) = joint_state_msg.velocity[4];
  v_.coeffRef(2) = joint_state_msg.velocity[6];
  v_.coeffRef(3) = joint_state_msg.velocity[5];
  v_.coeffRef(4) = joint_state_msg.velocity[1];
  v_.coeffRef(5) = joint_state_msg.velocity[2];
  v_.coeffRef(6) = joint_state_msg.velocity[7];
}


void MPCController::updateControlInput(const ros::TimerEvent& time_event) {
  // const double t = time_event.current_expected().toSec();
  const double t = 0;
  ROS_INFO("KKT error = %lf", mpc_.KKTError(t, q_, v_));
  mpc_.updateSolution(t, q_, v_);
  mpc_.getControlInput(u_);
  for (int i=0; i<dimv_; ++i) {
    joint_efforts_.data[i] = u_.coeff(i);
  }
  joint_efforts_publisher_.publish(joint_efforts_);
}

} // namespace crane_x7_mpc 


PLUGINLIB_EXPORT_CLASS(cranex7mpc::MPCController, nodelet::Nodelet)
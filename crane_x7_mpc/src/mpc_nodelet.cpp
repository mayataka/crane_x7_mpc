#include "mpc_nodelet.hpp"

#include "pluginlib/class_list_macros.h"


namespace cranex7mpc {

MPCNodelet::MPCNodelet() 
  : urdf_path_("/home/sotaro/catkin_ws/src/crane_x7_mpc/crane_x7_mpc/IDOCP/examples/cranex7/crane_x7_description/urdf/crane_x7.urdf"),
    robot_(urdf_path_),
    cost_(robot_),
    constraints_(robot_),
    mpc_(robot_, &cost_, &constraints_, T_, N_, num_proc_),
    dimq_(robot_.dimq()),
    dimv_(robot_.dimv()),
    q_(Eigen::VectorXd::Zero(robot_.dimq())),
    v_(Eigen::VectorXd::Zero(robot_.dimv())),
    u_(Eigen::VectorXd::Zero(robot_.dimv())),
    q_ref_(Eigen::VectorXd::Zero(robot_.dimq())), 
    Kq_(Eigen::MatrixXd::Zero(robot_.dimv(), robot_.dimv())),
    Kv_(Eigen::MatrixXd::Zero(robot_.dimv(), robot_.dimv())) {
  robot_.setJointDamping(Eigen::VectorXd::Constant(robot_.dimv(), 1.0e-06));
  cost_.set_a_weight(Eigen::VectorXd::Constant(robot_.dimv(), 0.1));
  mpc_ = idocp::MPC(robot_, &cost_, &constraints_, T_, N_, num_proc_);
}


bool MPCNodelet::setGoalConfiguration(
    crane_x7_msgs::SetGoalConfiguration::Request& request, 
    crane_x7_msgs::SetGoalConfiguration::Response& response) {
  ROS_INFO("set goal configuration!!");
  for (int i=0; i<dimq_; ++i) {
    q_ref_.coeffRef(i) = request.goal_configuration[i];
  }
  cost_.set_q_ref(q_ref_);
  response.success = true;
  return true;
}


void MPCNodelet::onInit() {
  node_handle_ = getNodeHandle();
  service_server_ = node_handle_.advertiseService(
      "/crane_x7/mpc_nodelet/set_goal_configuration", 
      &cranex7mpc::MPCNodelet::setGoalConfiguration, this);
  joint_state_subscriber_ = node_handle_.subscribe(
      "/crane_x7/joint_states", 10, 
      &cranex7mpc::MPCNodelet::subscribeJointState, this);
  timer_ = node_handle_.createTimer(
      ros::Duration(0.0025), &cranex7mpc::MPCNodelet::updateControlInputPolicy, 
      this);
  control_input_policy_publisher_
      = node_handle_.advertise<crane_x7_msgs::ControlInputPolicy>(
          "/crane_x7/mpc_nodelet/control_input_policy", 10);
}


void MPCNodelet::subscribeJointState(
    const sensor_msgs::JointState& joint_state) {
  q_.coeffRef(0) = joint_state.position[3];
  q_.coeffRef(1) = joint_state.position[4];
  q_.coeffRef(2) = joint_state.position[6];
  q_.coeffRef(3) = joint_state.position[5];
  q_.coeffRef(4) = joint_state.position[1];
  q_.coeffRef(5) = joint_state.position[2];
  q_.coeffRef(6) = joint_state.position[7];
  v_.coeffRef(0) = joint_state.velocity[3];
  v_.coeffRef(1) = joint_state.velocity[4];
  v_.coeffRef(2) = joint_state.velocity[6];
  v_.coeffRef(3) = joint_state.velocity[5];
  v_.coeffRef(4) = joint_state.velocity[1];
  v_.coeffRef(5) = joint_state.velocity[2];
  v_.coeffRef(6) = joint_state.velocity[7];
}


void MPCNodelet::updateControlInputPolicy(const ros::TimerEvent& time_event) {
  const double t = time_event.current_expected.toSec();
  ROS_INFO("KKT error = %lf", mpc_.KKTError(t, q_, v_));
  mpc_.updateSolution(t, q_, v_);
  mpc_.getControlInput(u_);
  mpc_.getStateFeedbackGain(Kq_, Kv_);
  Eigen::Map<Eigen::VectorXd>(&(policy_.q[0]), dimq_) = q_;
  Eigen::Map<Eigen::VectorXd>(&(policy_.v[0]), dimv_) = v_;
  Eigen::Map<Eigen::VectorXd>(&(policy_.u[0]), dimv_) = u_;
  Eigen::Map<Eigen::MatrixXd>(&(policy_.Kq[0]), dimv_, dimv_) = Kq_;
  Eigen::Map<Eigen::MatrixXd>(&(policy_.Kv[0]), dimv_, dimv_) = Kv_;
  control_input_policy_publisher_.publish(policy_);
}

} // namespace crane_x7_mpc 


PLUGINLIB_EXPORT_CLASS(cranex7mpc::MPCNodelet, nodelet::Nodelet)
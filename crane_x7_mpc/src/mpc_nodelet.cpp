#include "mpc_nodelet.hpp"

#include "pluginlib/class_list_macros.h"


namespace crane_x7_mpc {

MPCNodelet::MPCNodelet() 
  : robot_(),
    mpc_(),
    cost_(),
    joint_space_cost_(),
    constraints_(),
    joint_position_lower_limit_(),
    joint_position_upper_limit_(),
    joint_velocity_lower_limit_(),
    joint_velocity_upper_limit_(),
    joint_torques_lower_limit_(),
    joint_torques_upper_limit_(),
    horizon_length_(0),
    horizon_discretization_steps_(0),
    num_procs_(0),
    q_(),
    v_(),
    u_(Eigen::VectorXd::Zero(kDimq)),
    q_ref_(), 
    Kq_(Eigen::MatrixXd::Zero(kDimq, kDimq)),
    Kv_(Eigen::MatrixXd::Zero(kDimq, kDimq)) {
  q_.setZero();
  v_.setZero();
  u_.setZero();
  q_ref_.setZero();
}


bool MPCNodelet::setGoalConfiguration(
    crane_x7_msgs::SetGoalConfiguration::Request& request, 
    crane_x7_msgs::SetGoalConfiguration::Response& response) {
  ROS_INFO("set goal configuration!!");
  q_ref_ = Eigen::Map<Eigen::Matrix<double, kDimq, 1>>(&(request.goal_configuration[0]));
  joint_space_cost_->set_q_ref(q_ref_);
  response.success = true;
  return true;
}


void MPCNodelet::onInit() {
  node_handle_ = getNodeHandle();
  service_server_ = node_handle_.advertiseService(
      "/crane_x7/mpc_nodelet/set_goal_configuration", 
      &crane_x7_mpc::MPCNodelet::setGoalConfiguration, this);
  joint_state_subscriber_ = node_handle_.subscribe(
      "/crane_x7/joint_states", 10, 
      &crane_x7_mpc::MPCNodelet::subscribeJointState, this);
  timer_ = node_handle_.createTimer(
      ros::Duration(0.0025), 
      &crane_x7_mpc::MPCNodelet::updateControlInputPolicy, this);
  control_input_policy_publisher_
      = node_handle_.advertise<crane_x7_msgs::ControlInputPolicy>(
          "/crane_x7/mpc_nodelet/control_input_policy", 10);

  std::string path_to_urdf;
  getPrivateNodeHandle().getParam("path_to_urdf", path_to_urdf);
  robot_ = idocp::Robot(path_to_urdf);
  joint_space_cost_ = std::make_shared<idocp::JointSpaceCost>(robot_);
  joint_space_cost_->set_q_weight(Eigen::VectorXd::Constant(kDimq, 10));
  joint_space_cost_->set_v_weight(Eigen::VectorXd::Constant(kDimq, 1));
  joint_space_cost_->set_a_weight(Eigen::VectorXd::Constant(kDimq, 0.1));
  joint_space_cost_->set_qf_weight(Eigen::VectorXd::Constant(kDimq, 10));
  joint_space_cost_->set_vf_weight(Eigen::VectorXd::Constant(kDimq, 1));
  cost_ = std::make_shared<idocp::CostFunction>();
  cost_->push_back(joint_space_cost_);
  joint_position_lower_limit_ = std::make_shared<idocp::JointPositionLowerLimit>(robot_);
  joint_position_upper_limit_ = std::make_shared<idocp::JointPositionUpperLimit>(robot_);
  joint_velocity_lower_limit_ = std::make_shared<idocp::JointVelocityLowerLimit>(robot_);
  joint_velocity_upper_limit_ = std::make_shared<idocp::JointVelocityUpperLimit>(robot_);
  joint_torques_lower_limit_ = std::make_shared<idocp::JointTorquesLowerLimit>(robot_);
  joint_torques_upper_limit_ = std::make_shared<idocp::JointTorquesUpperLimit>(robot_);
  constraints_ = std::make_shared<idocp::Constraints>();
  constraints_->push_back(joint_position_lower_limit_);
  constraints_->push_back(joint_position_upper_limit_);
  constraints_->push_back(joint_velocity_lower_limit_);
  constraints_->push_back(joint_velocity_upper_limit_);
  constraints_->push_back(joint_torques_lower_limit_);
  constraints_->push_back(joint_torques_upper_limit_);
  horizon_length_ = 1;
  horizon_discretization_steps_ = 25;
  num_procs_ = 2;
  mpc_ = idocp::MPC<idocp::OCP>(robot_, cost_, constraints_, horizon_length_, 
                                horizon_discretization_steps_, num_procs_);
  mpc_.initializeSolution(0, q_, v_);
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
  const double KKT_error = mpc_.KKTError(t, q_, v_);
  ROS_INFO("KKT error = %lf", KKT_error);
  if (std::isnan(KKT_error)) {
    u_.setZero();
    Kq_.setZero();
    Kv_.setZero();
  }
  else {
    mpc_.updateSolution(t, q_, v_);
    mpc_.getControlInput(u_);
    mpc_.getStateFeedbackGain(Kq_, Kv_);
  }
  Eigen::Map<Eigen::Matrix<double, kDimq, 1>>(&(policy_.q[0])) = q_;
  Eigen::Map<Eigen::Matrix<double, kDimq, 1>>(&(policy_.v[0])) = v_;
  Eigen::Map<Eigen::Matrix<double, kDimq, 1>>(&(policy_.u[0])) = u_;
  Eigen::Map<Eigen::Matrix<double, kDimq, kDimq>>(&(policy_.Kq[0])) = Kq_;
  Eigen::Map<Eigen::Matrix<double, kDimq, kDimq>>(&(policy_.Kv[0])) = Kv_;
  control_input_policy_publisher_.publish(policy_);
}

} // namespace crane_x7_mpc 


PLUGINLIB_EXPORT_CLASS(crane_x7_mpc::MPCNodelet, nodelet::Nodelet)
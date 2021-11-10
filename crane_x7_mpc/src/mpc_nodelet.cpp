#include "mpc_nodelet.hpp"

#include "pluginlib/class_list_macros.h"

#include <chrono>


namespace crane_x7_mpc {

MPCNodelet::MPCNodelet() 
  : ocp_solver_(),
    N_(20), 
    nthreads_(4), 
    niter_(2),
    T_(0.5), 
    dt_(T_/N_),
    robot_(),
    end_effector_frame_(26),
    cost_(),
    config_cost_(),
    task_cost_3d_(),
    task_cost_6d_(),
    ref_3d_(),
    ref_6d_(),
    constraints_(),
    joint_position_lower_limit_(),
    joint_position_upper_limit_(),
    joint_velocity_lower_limit_(),
    joint_velocity_upper_limit_(),
    joint_torques_lower_limit_(),
    joint_torques_upper_limit_(),
    q_(),
    v_(),
    u_(Eigen::VectorXd::Zero(kDimq)),
    qj_ref_(), 
    Kq_(Eigen::MatrixXd::Zero(kDimq, kDimq)),
    Kv_(Eigen::MatrixXd::Zero(kDimq, kDimq)) {
  q_.setZero();
  v_.setZero();
  u_.setZero();
  qj_ref_.setZero();
}


bool MPCNodelet::setGoalConfiguration(
    crane_x7_msgs::SetGoalConfiguration::Request& request, 
    crane_x7_msgs::SetGoalConfiguration::Response& response) {
  ROS_INFO("set goal configuration!!");
  qj_ref_ = Eigen::Map<Eigen::Matrix<double, kDimq, 1>>(&(request.goal_configuration[0]));
  config_cost_->set_q_ref(qj_ref_);
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
      &crane_x7_mpc::MPCNodelet::updatePolicy, this);
  policy_publisher_ = node_handle_.advertise<crane_x7_msgs::ControlInputPolicy>(
          "/crane_x7/mpc_nodelet/control_input_policy", 10);
  position_command_publisher_ = node_handle_.advertise<crane_x7_msgs::JointPositionCommand>(
          "/crane_x7/mpc_nodelet/joint_position_command", 10);

  std::string path_to_urdf;
  getPrivateNodeHandle().getParam("path_to_urdf", path_to_urdf);
  robot_ = robotoc::Robot(path_to_urdf);
  create_cost();
  create_constraints();
  T_ = 0.5;
  N_ = 20;
  nthreads_ = 2;
  ocp_solver_ = robotoc::UnconstrOCPSolver(robot_, cost_, constraints_, 
                                           T_, N_, nthreads_);
  // init OCP solver
  const int num_iteration = 10;
  ocp_solver_.setSolution("q", q_);
  ocp_solver_.setSolution("v", v_);
  ocp_solver_.initConstraints();
  const auto t_cl = std::chrono::system_clock::now();
  const double t = 1e-06 * std::chrono::duration_cast<std::chrono::microseconds>(
      t_cl.time_since_epoch()).count();
  if (num_iteration > 0) {
    for (int i=0; i<num_iteration; ++i) {
      ocp_solver_.updateSolution(t, q_, v_);
    }
  }
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


void MPCNodelet::updatePolicy(const ros::TimerEvent& time_event) {
  const double t = time_event.current_expected.toSec();
  const double kkt_error = ocp_solver_.KKTError();
  ROS_INFO("KKT error = %lf", kkt_error);
  if (std::isnan(kkt_error)) {
    u_.setZero();
    Kq_.setZero();
    Kv_.setZero();
  }
  else {
    for (int i=0; i<niter_; ++i) {
      ocp_solver_.updateSolution(t, q_, v_);
    }
    u_ = ocp_solver_.getSolution(0).u;
    ocp_solver_.getStateFeedbackGain(0, Kq_, Kv_);
  }
  Eigen::Map<Eigen::Matrix<double, kDimq, 1>>(&(control_input_policy_.q[0])) = q_;
  Eigen::Map<Eigen::Matrix<double, kDimq, 1>>(&(control_input_policy_.v[0])) = v_;
  Eigen::Map<Eigen::Matrix<double, kDimq, 1>>(&(control_input_policy_.u[0])) = u_;
  Eigen::Map<Eigen::Matrix<double, kDimq, kDimq>>(&(control_input_policy_.Kq[0])) = Kq_;
  Eigen::Map<Eigen::Matrix<double, kDimq, kDimq>>(&(control_input_policy_.Kv[0])) = Kv_;
  policy_publisher_.publish(control_input_policy_);
  Eigen::Map<Eigen::Matrix<double, kDimq, 1>>(&(joint_position_command_.q[0])) 
      = ocp_solver_.getSolution(2).q;
  position_command_publisher_.publish(joint_position_command_);
}


void MPCNodelet::create_cost() {
  cost_ = std::make_shared<robotoc::CostFunction>();
  config_cost_ = std::make_shared<robotoc::ConfigurationSpaceCost>(robot_);
  const double q_weight = 0.001;
  const double v_weight = 0.01;
  const double a_weight = 0.001;
  const double u_weight = 0;
  const double qf_weight = 0.001;
  const double vf_weight = 0.01;
  config_cost_->set_q_weight(Eigen::VectorXd::Constant(robot_.dimv(), q_weight));
  config_cost_->set_v_weight(Eigen::VectorXd::Constant(robot_.dimv(), v_weight));
  config_cost_->set_a_weight(Eigen::VectorXd::Constant(robot_.dimv(), a_weight));
  config_cost_->set_u_weight(Eigen::VectorXd::Constant(robot_.dimv(), u_weight));
  config_cost_->set_qf_weight(Eigen::VectorXd::Constant(robot_.dimv(), qf_weight));
  config_cost_->set_vf_weight(Eigen::VectorXd::Constant(robot_.dimv(), vf_weight));
  ref_3d_ = std::make_shared<TimeVaryingTaskSpace3DRef>();
  task_cost_3d_ = std::make_shared<robotoc::TimeVaryingTaskSpace3DCost>(robot_, end_effector_frame_, ref_3d_);
  const double task_3d_q_weight = 10;
  const double task_3d_qf_weight = 10;
  task_cost_3d_->set_q_weight(Eigen::Vector3d::Constant(task_3d_q_weight));
  task_cost_3d_->set_qf_weight(Eigen::Vector3d::Constant(task_3d_qf_weight));
  const double task_6d_q_weight = 10;
  const double task_6d_qf_weight = 10;
  ref_6d_ = std::make_shared<TimeVaryingTaskSpace6DRef>();
  task_cost_6d_ = std::make_shared<robotoc::TimeVaryingTaskSpace6DCost>(robot_, end_effector_frame_, ref_6d_);
  task_cost_6d_->set_q_weight(Eigen::Vector3d::Constant(task_6d_q_weight), 
                              Eigen::Vector3d::Constant(task_6d_q_weight));
  task_cost_6d_->set_qf_weight(Eigen::Vector3d::Constant(task_6d_qf_weight), 
                               Eigen::Vector3d::Constant(task_6d_qf_weight));
  cost_->push_back(config_cost_);
  cost_->push_back(task_cost_3d_);
  cost_->push_back(task_cost_6d_);

  // ref_3d_->deactivate();
  // ref_6d_->deactivate();
  ref_3d_->activate();
}


void MPCNodelet::create_constraints() {
  constraints_                = std::make_shared<robotoc::Constraints>();
  joint_position_lower_limit_ = std::make_shared<robotoc::JointPositionLowerLimit>(robot_);
  joint_position_upper_limit_ = std::make_shared<robotoc::JointPositionUpperLimit>(robot_);
  joint_velocity_lower_limit_ = std::make_shared<robotoc::JointVelocityLowerLimit>(robot_);
  joint_velocity_upper_limit_ = std::make_shared<robotoc::JointVelocityUpperLimit>(robot_);
  joint_torques_lower_limit_  = std::make_shared<robotoc::JointTorquesLowerLimit>(robot_);
  joint_torques_upper_limit_  = std::make_shared<robotoc::JointTorquesUpperLimit>(robot_);
  constraints_->push_back(joint_position_lower_limit_);
  constraints_->push_back(joint_position_upper_limit_);
  constraints_->push_back(joint_velocity_lower_limit_);
  constraints_->push_back(joint_velocity_upper_limit_);
  constraints_->push_back(joint_torques_lower_limit_);
  constraints_->push_back(joint_torques_upper_limit_);
}

} // namespace crane_x7_mpc 


PLUGINLIB_EXPORT_CLASS(crane_x7_mpc::MPCNodelet, nodelet::Nodelet)
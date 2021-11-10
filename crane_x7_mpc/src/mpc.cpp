#include "mpc.hpp"


namespace crane_x7_mpc {

MPC::MPC() 
  : ocp_solver_(),
    N_(20), 
    nthreads_(2), 
    niter_(3),
    T_(0.5), 
    dt_(T_/N_),
    kkt_error_(0),
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



void MPC::init(const std::string& path_to_urdf, const double t, 
               const Eigen::VectorXd& q, const Eigen::VectorXd& v) {
  create_cost();
  create_constraints();
  robot_ = robotoc::Robot(path_to_urdf);
  T_ = 0.5;
  N_ = 20;
  nthreads_ = 2;
  ocp_solver_ = robotoc::UnconstrOCPSolver(robot_, cost_, constraints_, 
                                           T_, N_, nthreads_);
  // init OCP solver
  ocp_solver_.setSolution("q", q_);
  ocp_solver_.setSolution("v", v_);
  ocp_solver_.initConstraints();
}


void MPC::updatePolicy(const double t, const Eigen::VectorXd& q, 
                       const Eigen::VectorXd& v, const int niter) {
  const int num_iteration = 10;
  ocp_solver_.setSolution("q", q_);
  ocp_solver_.setSolution("v", v_);
  ocp_solver_.initConstraints();
  for (int i=0; i<niter; ++i) {
    ocp_solver_.updateSolution(t, q, v);
  }
  kkt_error_ = ocp_solver_.KKTError();
}


void MPC::getPolicy(Eigen::VectorXd& u, 
                    const Eigen::VectorXd& Kq, 
                    const Eigen::VectorXd& Kv) const {
  if (std::isnan(kkt_error_)) {
    u.setZero();
    Kq.setZero();
    Kv.setZero();
  }
  else {
    u = ocp_solver_.getSolution(0).u;
    ocp_solver_.getStateFeedbackGain(0, Kq, Kv);
  }
}


const Eigen::VectorXd& MPC::get_q_opt(const int stage) const {
  return ocp_solver_.getSolution(stage).q;
}


const Eigen::VectorXd& MPC::get_v_opt(const int stage) const {
  return ocp_solver_.getSolution(stage).v;
}


const Eigen::VectorXd& MPC::get_a_opt(const int stage) const {
  return ocp_solver_.getSolution(stage).a;
}


const Eigen::VectorXd& MPC::get_u_opt(const int stage) const {
  return ocp_solver_.getSolution(stage).u;
}


double MPC::KKTError() const {
  return kkt_error_;
}


void MPC::create_cost() {
  cost_ = std::make_shared<robotoc::CostFunction>();
  config_cost_ = std::make_shared<robotoc::ConfigurationSpaceCost>(robot_);
  const double q_weight = 0.01;
  const double v_weight = 0.1;
  const double a_weight = 0.01;
  const double u_weight = 0;
  const double qf_weight = 0.01;
  const double vf_weight = 0.1;
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
  const double task_6d_q_trans_weight = 10;
  const double task_6d_q_rot_weight = 1;
  const double task_6d_qf_trans_weight = 10;
  const double task_6d_qf_rot_weight = 1;
  ref_6d_ = std::make_shared<TimeVaryingTaskSpace6DRef>();
  task_cost_6d_ = std::make_shared<robotoc::TimeVaryingTaskSpace6DCost>(robot_, end_effector_frame_, ref_6d_);
  task_cost_6d_->set_q_weight(Eigen::Vector3d::Constant(task_6d_q_trans_weight), 
                              Eigen::Vector3d::Constant(task_6d_q_rot_weight));
  task_cost_6d_->set_qf_weight(Eigen::Vector3d::Constant(task_6d_qf_trans_weight), 
                               Eigen::Vector3d::Constant(task_6d_qf_rot_weight));
  cost_->push_back(config_cost_);
  cost_->push_back(task_cost_3d_);
  cost_->push_back(task_cost_6d_);

  ref_3d_->deactivate();
  ref_6d_->deactivate();

  ref_3d_->activate();
  // ref_6d_->activate(); // this doe not work well 
}


void MPC::create_constraints() {
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
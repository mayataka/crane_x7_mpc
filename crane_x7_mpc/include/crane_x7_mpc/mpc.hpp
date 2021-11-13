#ifndef CRANE_X7_MPC_MPC_HPP_
#define CRANE_X7_MPC_MPC_HPP_

#include <string>
#include <vector>
#include <memory>

#include "pinocchio/fwd.hpp" // To avoid bug in BOOST_MPL_LIMIT_LIST_SIZE

#include "Eigen/Core"

#include "robotoc/solver/unconstr_ocp_solver.hpp"
#include "robotoc/cost/cost_function.hpp"
#include "robotoc/cost/configuration_space_cost.hpp"
#include "robotoc/cost/time_varying_task_space_3d_cost.hpp"
#include "robotoc/cost/time_varying_task_space_6d_cost.hpp"
#include "robotoc/constraints/constraints.hpp"
#include "robotoc/constraints/joint_position_lower_limit.hpp"
#include "robotoc/constraints/joint_position_upper_limit.hpp"
#include "robotoc/constraints/joint_velocity_lower_limit.hpp"
#include "robotoc/constraints/joint_velocity_upper_limit.hpp"
#include "robotoc/constraints/joint_torques_lower_limit.hpp"
#include "robotoc/constraints/joint_torques_upper_limit.hpp"


namespace crane_x7_mpc {

class TimeVaryingTaskSpace3DRef final : public robotoc::TimeVaryingTaskSpace3DRefBase {
public:
  TimeVaryingTaskSpace3DRef() 
    : TimeVaryingTaskSpace3DRefBase() {
    pos0_ << 0.15, 0, 0.3;
    radius_ = 0.1;
    is_active_ = false;
  }

  ~TimeVaryingTaskSpace3DRef() {}

  void update_q_3d_ref(const double t, Eigen::VectorXd& q_3d_ref) const override {
    q_3d_ref = pos0_;
    q_3d_ref.coeffRef(1) += radius_ * sin(0.5*M_PI*t);
    q_3d_ref.coeffRef(2) += radius_ * cos(0.5*M_PI*t);
  }

  bool isActive(const double t) const override {
    (void)t;
    return is_active_;
  }

  void activate() {
    is_active_ = true;
  }

  void deactivate() {
    is_active_ = false;
  }

private:
  double radius_;
  Eigen::Vector3d pos0_;
  bool is_active_;
};


class TimeVaryingTaskSpace6DRef final : public robotoc::TimeVaryingTaskSpace6DRefBase {
public:
  TimeVaryingTaskSpace6DRef() 
    : TimeVaryingTaskSpace6DRefBase() {
    rotm_  <<  0, 0, 1, 
               0, 1, 0,
              -1, 0, 0;
    pos0_ << 0.15, 0, 0.3;
    radius_ = 0.1;
    is_active_ = false;
  }

  ~TimeVaryingTaskSpace6DRef() {}

  void update_SE3_ref(const double t, pinocchio::SE3& SE3_ref) const override {
    Eigen::Vector3d pos(pos0_);
    pos.coeffRef(1) += radius_ * sin(0.5*M_PI*t);
    pos.coeffRef(2) += radius_ * cos(0.5*M_PI*t);
    SE3_ref = pinocchio::SE3(rotm_, pos);
  }

  bool isActive(const double t) const override {
    (void)t;
    return is_active_;
  }

  void activate() {
    is_active_ = true;
  }

  void deactivate() {
    is_active_ = false;
  }

private:
  double radius_;
  Eigen::Matrix3d rotm_;
  Eigen::Vector3d pos0_;
  bool is_active_;
};


class MPC {
public:
  MPC();

  void init(const std::string& path_to_urdf, const double t, 
            const Eigen::VectorXd& q, const Eigen::VectorXd& v);

  void updatePolicy(const double t, const Eigen::VectorXd& q, 
                    const Eigen::VectorXd& v, const int niter);

  void getPolicy(Eigen::VectorXd& u, Eigen::MatrixXd& Kq, Eigen::MatrixXd& Kv) const;

  const Eigen::VectorXd& get_q_opt(const int stage) const;
  const Eigen::VectorXd& get_v_opt(const int stage) const;
  const Eigen::VectorXd& get_a_opt(const int stage) const;
  const Eigen::VectorXd& get_u_opt(const int stage) const;

  double KKTError() const;

private:
  // OCP solver 
  robotoc::UnconstrOCPSolver ocp_solver_;
  int N_, nthreads_, niter_; 
  double T_, barrier_, kkt_error_;
  robotoc::Robot robot_;
  // Cost function
  int end_effector_frame_;
  std::shared_ptr<robotoc::CostFunction> cost_;
  std::shared_ptr<robotoc::ConfigurationSpaceCost> config_cost_;
  std::shared_ptr<robotoc::TimeVaryingTaskSpace3DCost> task_cost_3d_;
  std::shared_ptr<robotoc::TimeVaryingTaskSpace6DCost> task_cost_6d_;
  std::shared_ptr<TimeVaryingTaskSpace3DRef> ref_3d_;
  std::shared_ptr<TimeVaryingTaskSpace6DRef> ref_6d_;
  Eigen::VectorXd q_ref_;
  // Constraints
  std::shared_ptr<robotoc::Constraints> constraints_;
  std::shared_ptr<robotoc::JointPositionLowerLimit> joint_position_lower_limit_;
  std::shared_ptr<robotoc::JointPositionUpperLimit> joint_position_upper_limit_;
  std::shared_ptr<robotoc::JointVelocityLowerLimit> joint_velocity_lower_limit_;
  std::shared_ptr<robotoc::JointVelocityUpperLimit> joint_velocity_upper_limit_;
  std::shared_ptr<robotoc::JointTorquesLowerLimit> joint_torques_lower_limit_;
  std::shared_ptr<robotoc::JointTorquesUpperLimit> joint_torques_upper_limit_;

  void create_cost();
  void create_constraints();
};

} // namespace crane_x7_mpc

#endif // CRANE_X7_MPC_MPC_HPP_ 
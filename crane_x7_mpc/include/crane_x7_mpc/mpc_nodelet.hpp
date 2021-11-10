#ifndef CRANE_X7_MPC_MPC_NODELET_HPP_
#define CRANE_X7_MPC_MPC_NODELET_HPP_

#include <string>
#include <vector>
#include <memory>

#include "pinocchio/fwd.hpp" // To avoid bug in BOOST_MPL_LIMIT_LIST_SIZE
#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"

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

#include "crane_x7_msgs/ControlInputPolicy.h"
#include "crane_x7_msgs/JointPositionCommand.h"
#include "crane_x7_msgs/SetGoalConfiguration.h"


namespace crane_x7_mpc {

class TimeVaryingTaskSpace3DRef final : public robotoc::TimeVaryingTaskSpace3DRefBase {
public:
  TimeVaryingTaskSpace3DRef() 
    : TimeVaryingTaskSpace3DRefBase() {
    pos0_ << 0.3, 0, 0.3;
    radius_ = 0.15;
    is_active_ = false;
  }

  ~TimeVaryingTaskSpace3DRef() {}

  void update_q_3d_ref(const double t, Eigen::VectorXd& q_3d_ref) const override {
    q_3d_ref = pos0_;
    q_3d_ref.coeffRef(1) += radius_ * sin(M_PI*t);
    q_3d_ref.coeffRef(2) += radius_ * cos(M_PI*t);
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
    pos0_ << 0.3, 0, 0.3;
    radius_ = 0.1;
    is_active_ = false;
  }

  ~TimeVaryingTaskSpace6DRef() {}

  void update_SE3_ref(const double t, pinocchio::SE3& SE3_ref) const override {
    Eigen::Vector3d pos(pos0_);
    pos.coeffRef(1) += radius_ * sin(M_PI*t);
    pos.coeffRef(2) += radius_ * cos(M_PI*t);
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


class MPCNodelet : public nodelet::Nodelet {
public:
  MPCNodelet();

  bool setGoalConfiguration(
      crane_x7_msgs::SetGoalConfiguration::Request& request, 
      crane_x7_msgs::SetGoalConfiguration::Response& response);

private:
  virtual void onInit() override;
  void subscribeJointState(const sensor_msgs::JointState& joint_state);
  void updatePolicy(const ros::TimerEvent& time_event);

  ros::NodeHandle node_handle_;
  ros::ServiceServer service_server_;
  ros::Subscriber joint_state_subscriber_;
  ros::Publisher policy_publisher_, position_command_publisher_;
  crane_x7_msgs::ControlInputPolicy control_input_policy_;
  crane_x7_msgs::JointPositionCommand joint_position_command_;
  ros::Timer timer_;

  // OCP solver 
  robotoc::UnconstrOCPSolver ocp_solver_;
  int N_, nthreads_, niter_; 
  double T_, dt_, barrier_;
  robotoc::Robot robot_;
  // Cost function
  int end_effector_frame_;
  std::shared_ptr<robotoc::CostFunction> cost_;
  std::shared_ptr<robotoc::ConfigurationSpaceCost> config_cost_;
  std::shared_ptr<robotoc::TimeVaryingTaskSpace3DCost> task_cost_3d_;
  std::shared_ptr<robotoc::TimeVaryingTaskSpace6DCost> task_cost_6d_;
  std::shared_ptr<TimeVaryingTaskSpace3DRef> ref_3d_;
  std::shared_ptr<TimeVaryingTaskSpace6DRef> ref_6d_;
  // Constraints
  std::shared_ptr<robotoc::Constraints> constraints_;
  std::shared_ptr<robotoc::JointPositionLowerLimit> joint_position_lower_limit_;
  std::shared_ptr<robotoc::JointPositionUpperLimit> joint_position_upper_limit_;
  std::shared_ptr<robotoc::JointVelocityLowerLimit> joint_velocity_lower_limit_;
  std::shared_ptr<robotoc::JointVelocityUpperLimit> joint_velocity_upper_limit_;
  std::shared_ptr<robotoc::JointTorquesLowerLimit> joint_torques_lower_limit_;
  std::shared_ptr<robotoc::JointTorquesUpperLimit> joint_torques_upper_limit_;

  static constexpr int kDimq = 7;
  Eigen::Matrix<double, kDimq, 1> q_, v_, qj_ref_;
  Eigen::VectorXd u_;
  Eigen::MatrixXd Kq_, Kv_;

  void create_cost();
  void create_constraints();
};

} // namespace crane_x7_mpc

#endif // CRANE_X7_MPC_MPC_NODELET_HPP_
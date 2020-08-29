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

#include "idocp/idocp.hpp"

#include "crane_x7_msgs/ControlInputPolicy.h"
#include "crane_x7_msgs/SetGoalConfiguration.h"


namespace crane_x7_mpc {

class MPCNodelet : public nodelet::Nodelet {
public:
  MPCNodelet();

  bool setGoalConfiguration(
      crane_x7_msgs::SetGoalConfiguration::Request& request, 
      crane_x7_msgs::SetGoalConfiguration::Response& response);

private:
  virtual void onInit() override;
  void subscribeJointState(const sensor_msgs::JointState& joint_state);
  void updateControlInputPolicy(const ros::TimerEvent& time_event);

  ros::NodeHandle node_handle_;
  ros::ServiceServer service_server_;
  ros::Subscriber joint_state_subscriber_;
  ros::Publisher control_input_policy_publisher_;
  crane_x7_msgs::ControlInputPolicy policy_;
  ros::Timer timer_;

  idocp::Robot robot_;
  idocp::MPC<idocp::OCP> mpc_;
  std::shared_ptr<idocp::CostFunction> cost_;
  std::shared_ptr<idocp::JointSpaceCost> joint_space_cost_;
  std::shared_ptr<idocp::Constraints> constraints_;
  std::shared_ptr<idocp::JointPositionLowerLimit> joint_position_lower_limit_;
  std::shared_ptr<idocp::JointPositionUpperLimit> joint_position_upper_limit_;
  std::shared_ptr<idocp::JointVelocityLowerLimit> joint_velocity_lower_limit_;
  std::shared_ptr<idocp::JointVelocityUpperLimit> joint_velocity_upper_limit_;
  std::shared_ptr<idocp::JointTorquesLowerLimit> joint_torques_lower_limit_;
  std::shared_ptr<idocp::JointTorquesUpperLimit> joint_torques_upper_limit_;
  double horizon_length_;
  int horizon_discretization_steps_, num_procs_;

  static constexpr int kDimq = 7;
  Eigen::Matrix<double, kDimq, 1> q_, v_, q_ref_;
  Eigen::VectorXd u_;
  Eigen::MatrixXd Kq_, Kv_;
};

} // namespace crane_x7_mpc


#endif // CRANE_X7_MPC_MPC_NODELET_HPP_
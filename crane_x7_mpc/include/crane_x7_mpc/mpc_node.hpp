#ifndef CRANE_X7_MPC_MPC_NODE_HPP_
#define CRANE_X7_MPC_MPC_NODE_HPP_

#include <string>

#include "pinocchio/fwd.hpp" // To avoid bug in BOOST_MPL_LIMIT_LIST_SIZE
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"

#include "Eigen/Core"

#include "mpc.hpp"

#include "crane_x7_msgs/ControlInputPolicy.h"


namespace crane_x7_mpc {

class MPCNode {
public:
  MPCNode(ros::NodeHandle& node_handle);

  void init(const std::string& path_to_urdf);

  double dt() const;

private:
  void subscribeJointState(const sensor_msgs::JointState& joint_state);
  void updatePolicy(const ros::TimerEvent& time_event);

  ros::NodeHandle node_handle_;
  ros::Subscriber joint_state_subscriber_;
  ros::Publisher control_input_policy_publisher_;
  crane_x7_msgs::ControlInputPolicy control_input_policy_;
  ros::Timer timer_;

  // MPC solver 
  MPC mpc_;
  int niter_;
  double dt_;
  Eigen::VectorXd q_, v_, qj_ref_, u_;
  Eigen::MatrixXd Kq_, Kv_;
};

} // namespace crane_x7_mpc

#endif // CRANE_X7_MPC_MPC_NODE_HPP_
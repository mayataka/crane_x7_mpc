#ifndef CRANE_X7_MPC_MPC_CONTROLLER_HPP_
#define CRANE_X7_MPC_MPC_CONTROLLER_HPP_

#include <string>
#include <vector>

#include "pinocchio/fwd.hpp" // To avoid bug in BOOST_MPL_LIMIT_LIST_SIZE
#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"

#include "Eigen/Core"

#include "ocp/mpc.hpp"
#include "robot/robot.hpp"
#include "manipulator/cost_function.hpp"
#include "manipulator/constraints.hpp"

#include "crane_x7_mpc/SetGoalConfiguration.h"


namespace cranex7mpc {

class MPCController : public nodelet::Nodelet {
public:
  MPCController();
  bool setGoalConfiguration(
      crane_x7_mpc::SetGoalConfiguration::Request& request, 
      crane_x7_mpc::SetGoalConfiguration::Response& response);

private:
  virtual void onInit() override;
  void subscribeJointState(const sensor_msgs::JointState& joint_state_msg);
  void updateControlInput(const ros::TimerEvent& time_event);

  ros::NodeHandle node_handle_;
  ros::ServiceServer service_server_;
  ros::Subscriber joint_state_subscriber_;
  ros::Publisher joint_efforts_publisher_;
  std_msgs::Float64MultiArray joint_efforts_;
  ros::Timer timer_;

  static constexpr double T_ = 1;
  static constexpr unsigned int N_ = 25;
  static constexpr unsigned int num_proc_ = 2;
  std::string urdf_path_;
  idocp::Robot robot_;
  idocp::manipulator::CostFunction cost_;
  idocp::manipulator::Constraints constraints_;
  idocp::MPC mpc_;
  unsigned int dimq_, dimv_;
  Eigen::VectorXd q_, v_, u_, q_ref_;
};

} // namespace cranex7mpc


#endif // CRANE_X7_MPC_MPC_CONTROLLER_HPP_
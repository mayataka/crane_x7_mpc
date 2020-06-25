#include CRANE_X7_MPC_MPC_NODELET_HPP_
#define CRANE_X7_MPC_MPC_NODELET_HPP_

#include <string>
#include <vector>

#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include "pluginlib/class_list_macros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"

#include "Eigen/Core"

#include "IDOCP/ocp/mpc.hpp"
#include "IDOCP/robot/robot.hpp"
#include "IDOCP/manipulator/cost_function.hpp"
#include "IDOCP/manipulator/constraints.hpp"


namespace cranex7mpc {

class MPCNodelet : public nodelet::Nodelet {
public:
  MPCNodelet();
  virtual void onInit();
  void timer_callback(const ros::TimerEvent&);

private:
  void reflect_jointstate();

  ros::NodeHandle public_node_handler_;
  ros::NodeHandle private_node_handler_;
  ros::Publisher joint_effort_publisher_;
  ros::Subscriber joint_state_subscriber_;
  ros::Timer timer_;

  static constexpr double T_ = 1;
  static constexpr unsigned int N_ = 25;
  static constexpr unsigned int num_proc_ = 2;
  std::string urdf_path_;
  idocp::Robot robot_;
  idocp::manipulator::CostFunction cost_;
  idocp::manipulator::Constraints constraints_;
  idocp::MPC mpc_;
  Eigen::VectorXd q_, v_, tau_;
};

} // namespace cranex7mpc

#endif // CRANE_X7_MPC_MPC_NODELET_HPP_
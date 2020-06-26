#ifndef CRANE_X7_MPC_MPC_NODELET_HPP_
#define CRANE_X7_MPC_MPC_NODELET_HPP_

#include <string>
#include <vector>

#include "pinocchio/fwd.hpp" // To avoid bug in BOOST_MPL_LIMIT_LIST_SIZE
#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include "controller_interface/controller.h"
#include "hardware_interface/joint_command_interface.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"

#include "Eigen/Core"

#include "ocp/mpc.hpp"
#include "robot/robot.hpp"
#include "manipulator/cost_function.hpp"
#include "manipulator/constraints.hpp"

#include "crane_x7_mpc/SetGoalConfiguration.h"


namespace cranex7mpc {

class MPCNodelet : public nodelet::Nodelet, 
                   public controller_interface::Controller
                          <hardware_interface::EffortJointInterface> {
public:
  MPCNodelet();
  bool init(hardware_interface::EffortJointInterface* hardware, 
            ros::NodeHandle &node_handler) override;
  void starting(const ros::Time& time) override;
  void stopping(const ros::Time& time) override;
  bool setGoalConfiguration(
      crane_x7_mpc::SetGoalConfiguration::Request& request, 
      crane_x7_mpc::SetGoalConfiguration::Response& response);

private:
  virtual void onInit() override;
  void update(const ros::Time& time, const ros::Duration& period) override;

  std::vector<hardware_interface::JointHandle> joint_effort_handlers_;

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

  ros::NodeHandle node_handle_;
  ros::ServiceServer service_server_;
};

} // namespace cranex7mpc


#endif // CRANE_X7_MPC_MPC_NODELET_HPP_
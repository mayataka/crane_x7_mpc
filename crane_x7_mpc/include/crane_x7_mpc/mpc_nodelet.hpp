#ifndef CRANE_X7_MPC_MPC_NODELET_HPP_
#define CRANE_X7_MPC_MPC_NODELET_HPP_

#include <string>
#include <vector>

#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include "controller_interface/controller.h"
#include "hardware_interface/joint_command_interface.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"


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

private:
  virtual void onInit();
  void update(const ros::Time& time, const ros::Duration& period) override;

  ros::NodeHandle node_handler_;
  ros::Subscriber joint_state_subscriber_;
  ros::Timer timer_;
  std::vector<hardware_interface::JointHandle> joint_effort_handlers_;
  std_msgs::Float64MultiArray joint_efforts_;

};
} // namespace cranex7mpc


#endif // CRANE_X7_MPC_MPC_NODELET_HPP_
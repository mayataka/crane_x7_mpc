#include "mpc_nodelet.hpp"

namespace cranex7mpc {

MPCNodelet::MPCNodelet 
  : nodelet::Nodelet(),
    urdf_path_(""),
    robot_(urdf_path_),
    cost_(robot_),
    constraints_(robot_),
    constraints_(robot_),
    mpc_(robot_, &cost_, &constraints_, T_, N_, num_proc_),
    q_(Eigen::VectorXd::Zero(robot_.dimq())),
    v_(Eigen::VectorXd::Zero(robot_.dimq())),
    tau_(Eigen::VectorXd::Zero(robot_.dimq())) {

ros::Subscriber sub = n.subscribe("/joint_states", 10, reflect_jointstate);
}


void MPCNodelet::onInit() {
  public_node_handler_ = getNodeHandle();
  private_node_handler_ = getPrivateNodeHandle();
  joint_effort_publisher_ = public_node_handler_.advertise<std_msgs::String>("chatter", 10);
  joint_state_subscriber_ = private_node_handler_.subscribe("/joint_states", 10);

  content_ = "hello";
  pnh_.getParam("content", content_);
  pub_ = nh_.advertise<std_msgs::String>("chatter", 10);
  timer_ = nh_.createTimer(ros::Duration(1.0), &plugin_nodelet_talker::timer_callback, this);
}

} // namespace cranex7mpc
#include "mpc_node.hpp"

#include "pluginlib/class_list_macros.h"


namespace crane_x7_mpc {

MPCNode::MPCNode(ros::NodeHandle& node_handle) 
  : node_handle_(node_handle),
    joint_state_subscriber_(
        node_handle.subscribe(
            "/crane_x7/joint_states", 10, 
            &crane_x7_mpc::MPCNode::subscribeJointState, this)),
    control_input_policy_publisher_(
        node_handle.advertise<crane_x7_msgs::ControlInputPolicy>(
          "/crane_x7/mpc_node/control_input_policy", 10)),
    timer_(
        node_handle.createTimer(
            ros::Duration(0.0025), 
            &crane_x7_mpc::MPCNode::updatePolicy, this)),
    mpc_(),
    niter_(3),
    dt_(0.0025),
    q_(Eigen::VectorXd::Zero(7)),
    v_(Eigen::VectorXd::Zero(7)),
    qj_ref_(Eigen::VectorXd::Zero(7)), 
    u_(Eigen::VectorXd::Zero(7)),
    Kq_(Eigen::MatrixXd::Zero(7, 7)),
    Kv_(Eigen::MatrixXd::Zero(7, 7)) {
}


void MPCNode::init(const std::string& path_to_urdf) {
  const double t = 0;
  mpc.init(path_to_urdf, t, q_, v_);
}


double MPCNode::dt() const {
  return dt_;
}


void MPCNode::subscribeJointState(const sensor_msgs::JointState& joint_state) {
  for (int i=0; i<7; ++i) {
    q_.coeffRef(i) = joint_state.position[i];
  }
  for (int i=0; i<7; ++i) {
    v_.coeffRef(i) = joint_state.velocity[i];
  }
}


void MPCNode::updatePolicy(const ros::TimerEvent& time_event) {
  const double t = time_event.current_expected.toSec();
  const Eigen::VectorXd q = q_;
  const Eigen::VectorXd v = v_;
  mpc.updatePolicy(t, q, v, niter_);
  mpc.getPolicy(u_, Kq_, Kv_);
  const double kkt_error = mpc_.KKTError();
  ROS_INFO("KKT error = %lf", kkt_error);
  const Eigen::VectorXd& a_opt = mpc_.get_a_opt(0);
  Eigen::Map<Eigen::Matrix<double, kDimq, 1>>(&(control_input_policy_.q[0])) = q + dt_ * v + (dt_*dt_) * a_opt;
  Eigen::Map<Eigen::Matrix<double, kDimq, 1>>(&(control_input_policy_.v[0])) = v + dt_ * a_opt;
  Eigen::Map<Eigen::Matrix<double, kDimq, 1>>(&(control_input_policy_.u[0])) = u_;
  Eigen::Map<Eigen::Matrix<double, kDimq, kDimq>>(&(control_input_policy_.Kq[0])) = Kq_;
  Eigen::Map<Eigen::Matrix<double, kDimq, kDimq>>(&(control_input_policy_.Kv[0])) = Kv_;
  policy_publisher_.publish(control_input_policy_);
}

} // namespace crane_x7_mpc 


int main(int argc, char * argv[]) {
  ros::init(argc, argv, "MPCNode");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate("~");
  std::string path_to_urdf;
  nhPrivate.getParam("path_to_urdf", path_to_urdf);
  crane_x7_mpc::MPCNode mpc_node(nh);
  mpc_node.init(path_to_urdf);
  ros::spin();
	// ros::Rate loop_rate(400);
	// while (ros::ok()) {
	// 	mpc_node.subscribeJointState();
	// 	ros::spinOnce();
	// 	loop_rate.sleep();
	// }
  return 0;
}
#include "mpc_node.hpp"

#include "pluginlib/class_list_macros.h"


namespace crane_x7_mpc {

MPCNode::MPCNode(ros::NodeHandle node_handle) 
  : mpc_(),
    niter_(3),
    dt_(0.0025),
    q_(Eigen::VectorXd::Zero(7)),
    v_(Eigen::VectorXd::Zero(7)),
    q_mpc_(Eigen::VectorXd::Zero(7)),
    v_mpc_(Eigen::VectorXd::Zero(7)),
    qj_ref_(Eigen::VectorXd::Zero(7)), 
    u_(Eigen::VectorXd::Zero(7)),
    Kq_(Eigen::MatrixXd::Zero(7, 7)),
    Kv_(Eigen::MatrixXd::Zero(7, 7)), 
    node_handle_(node_handle),
    joint_state_subscriber_(
        node_handle.subscribe(
            "/crane_x7/joint_states", 10, 
            &crane_x7_mpc::MPCNode::subscribeJointState, this)),
    control_input_policy_publisher_(
        node_handle.advertise<crane_x7_msgs::ControlInputPolicy>(
          "/crane_x7/crane_x7_mpc/control_input_policy", 10)),
    timer_(
        node_handle.createTimer(
            ros::Duration(dt_), 
            &crane_x7_mpc::MPCNode::updatePolicy, this)) {
}


void MPCNode::init(const std::string& path_to_urdf) {
  const double t = 0;
  mpc_.init(path_to_urdf, t, q_, v_);
}


double MPCNode::dt() const {
  return dt_;
}


void MPCNode::subscribeJointState(const sensor_msgs::JointState& joint_state) {
  // This is for the actual robot
  q_.coeffRef(0) = joint_state.position[3];
  q_.coeffRef(1) = joint_state.position[4];
  q_.coeffRef(2) = joint_state.position[6];
  q_.coeffRef(3) = joint_state.position[5];
  q_.coeffRef(4) = joint_state.position[1];
  q_.coeffRef(5) = joint_state.position[2];
  q_.coeffRef(6) = joint_state.position[7];
  v_.coeffRef(0) = joint_state.velocity[3];
  v_.coeffRef(1) = joint_state.velocity[4];
  v_.coeffRef(2) = joint_state.velocity[6];
  v_.coeffRef(3) = joint_state.velocity[5];
  v_.coeffRef(4) = joint_state.velocity[1];
  v_.coeffRef(5) = joint_state.velocity[2];
  v_.coeffRef(6) = joint_state.velocity[7];

  // // This is for Gazebo simulation
  // for (int i=0; i<7; ++i) {
  //   q_.coeffRef(i) = joint_state.position[i];
  // }
  // for (int i=0; i<7; ++i) {
  //   v_.coeffRef(i) = joint_state.velocity[i];
  // }
}


void MPCNode::updatePolicy(const ros::TimerEvent& time_event) {
  const double t = time_event.current_expected.toSec();
  q_mpc_ = q_;
  v_mpc_ = v_;
  mpc_.updatePolicy(t, q_mpc_, v_mpc_, niter_);
  mpc_.getPolicy(u_, Kq_, Kv_);
  const double kkt_error = mpc_.KKTError();
  ROS_INFO("KKT error = %lf", kkt_error);
  const Eigen::VectorXd& a_opt = mpc_.get_a_opt(0);
  q_mpc_ = q_;
  v_mpc_ = v_;
  Eigen::Map<Vector7d>(&(control_input_policy_.q[0])) = q_mpc_ + dt_ * v_mpc_ + (dt_*dt_) * a_opt;
  Eigen::Map<Vector7d>(&(control_input_policy_.v[0])) = v_mpc_ + dt_ * a_opt;
  Eigen::Map<Vector7d>(&(control_input_policy_.u[0])) = u_;
  Eigen::Map<Matrix7d>(&(control_input_policy_.Kq[0])) = Kq_;
  Eigen::Map<Matrix7d>(&(control_input_policy_.Kv[0])) = Kv_;
  control_input_policy_publisher_.publish(control_input_policy_);
}

} // namespace crane_x7_mpc 


int main(int argc, char * argv[]) {
  ros::init(argc, argv, "MPCNode");
  ros::NodeHandle nh_private("~");
  std::string path_to_urdf;
  nh_private.getParam("path_to_urdf", path_to_urdf);
  ros::NodeHandle nh;
  crane_x7_mpc::MPCNode mpc_node(nh);
  mpc_node.init(path_to_urdf);
  ros::spin();
  return 0;
}
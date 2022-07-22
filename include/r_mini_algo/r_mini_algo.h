#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

#ifndef R_MINI_KINEMATICS
#define R_MINI_KINEMATICS

namespace r_mini
{

/** @class r_mini::RobotKinematics
    @brief Access the forward and inverse kinematics, the 
    kinematic Jacobian and the Hessian of r_mini */
class RobotKinematics
{
public:
  RobotKinematics(ros::NodeHandle* nh, std::string joint_states_topic);
  RobotKinematics(ros::NodeHandle* nh);
  ~RobotKinematics();
  void getJacobian();
  //getHessian();
  //getDeterminant();
  //getForwardKinematics();
  //getInverseKinematics();
private:
  void jointStatesCallback_(const sensor_msgs::JointStateConstPtr &state);
  ros::NodeHandle nh_;
  ros::Subscriber joint_states_sub_;
  std::string joint_states_topic_;
  robot_model_loader::RobotModelLoader  robot_model_loader_;
  moveit::core::RobotModelPtr kinematic_model_;
  moveit::core::RobotStatePtr kinematic_state_;
  moveit::core::JointModelGroup* joint_model_group_;  
  std::vector<double> group_position_;
  Eigen::MatrixXd jacobian_;
  Eigen::Vector3d reference_point_position_;
  //getJacobian_();
  
  //getHessian_();
  //getDeterminant_();
  //printJacobian_();
  //printHessian_();
  //printDeterminant_();


};
} //end of namespace r_mini
#endif


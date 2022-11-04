#include "r_mini_algo/r_mini_algo.h"

namespace r_mini
{
RobotKinematics::RobotKinematics(ros::NodeHandle* nh, std::string joint_states_topic)
  : robot_model_loader_("robot_description")
  , reference_point_position_ (0.0, 0.0, 0.0)
  , joint_states_topic_(joint_states_topic)
  , nh_(*nh) 
{

  kinematic_model_ = robot_model_loader_.getModel();
  kinematic_state_.reset(new robot_state::RobotState(kinematic_model_));
  ROS_INFO_STREAM("INITIALIZE KINEMATIC STATE"); //[DELETE]
  joint_model_group_ = kinematic_model_->getJointModelGroup("arm");
  ROS_INFO_STREAM("INITIALIZE SUBSCRIBER"); //[DELETE]
  joint_states_sub_ = nh_.subscribe(joint_states_topic_, 1000, &RobotKinematics::jointStatesCallback_, this);
  ROS_INFO_STREAM("CONSTRUCTOR is DONE"); //[DELETE]

}
RobotKinematics::RobotKinematics(ros::NodeHandle* nh)
  : nh_(*nh)
  , robot_model_loader_("robot_description")
  , reference_point_position_ (0.0, 0.0, 0.0)
{
  kinematic_model_ = robot_model_loader_.getModel();
  kinematic_state_.reset(new robot_state::RobotState(kinematic_model_));
  joint_model_group_ = kinematic_model_->getJointModelGroup("arm");
  joint_states_sub_ = nh_.subscribe("joint_states", 1000, &RobotKinematics::jointStatesCallback_, this);
}

RobotKinematics::RobotKinematics()
  : nh_("~")
  , robot_model_loader_("robot_description")
  , reference_point_position_ (0.0, 0.0, 0.0)
{
  kinematic_model_ = robot_model_loader_.getModel();
  kinematic_state_.reset(new robot_state::RobotState(kinematic_model_));
  joint_model_group_ = kinematic_model_->getJointModelGroup("arm");
  joint_states_sub_ = nh_.subscribe("joint_states", 1000, &RobotKinematics::jointStatesCallback_, this);
}

RobotKinematics::~RobotKinematics() 
{
  ROS_INFO_STREAM("DESTRUCTOR IS CALLED");
}

void RobotKinematics::jointStatesCallback_(const sensor_msgs::JointStateConstPtr &state)
{
  ROS_INFO_STREAM("IN JOINT_STATE_CALLBACK"); //[DELETE]
  group_position_ = state -> position;
  ROS_INFO_STREAM("GETTING JACOBIAN"); //[DELETE]
  getJacobian();
}

void RobotKinematics::getJacobian()
{
  if ( group_position_.empty() )
  {
    ROS_INFO_STREAM("Waiting for joint_states messages ... ");
  }
  else
  {
    kinematic_state_ -> setJointGroupPositions(joint_model_group_, group_position_);
    kinematic_state_ -> getJacobian(joint_model_group_, kinematic_state_ -> getLinkModel(joint_model_group_ -> getLinkModelNames().back()), reference_point_position_, jacobian_);
    ROS_INFO_STREAM("Jacobian: \n" << jacobian_ << "\n");
    ROS_INFO_STREAM("Det(J): " << jacobian_.fullPivLu().determinant() << "\n");
  }
}
  
} //end of namespace r_mini

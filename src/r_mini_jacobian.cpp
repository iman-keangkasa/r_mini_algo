#include "r_mini_algo/r_mini_algo.h"
int main( int argc, char ** argv)
{
  ros::init(argc, argv, "r_mini_jacobian_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(0);
  spinner.start();

  r_mini::RobotKinematics r_jacobian(&nh,"joint_states");
  while(ros::ok()) {};
  //ros::spin();  
  return 0;
}

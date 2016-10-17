#include <ros/ros.h>
#include "JacobianSrv.hpp"
#include <string>

using namespace std;

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "jacobian_srv");
	ros::AsyncSpinner spinner(1);
  spinner.start();

  // Start the jacobian server
	JacobianSrv jacobian_srv = JacobianSrv("robot_description");
	ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("compute_jacobian", 
  		                                            &JacobianSrv::computeJacobian,
  		                                            &jacobian_srv);
  ROS_INFO("Jacobian server Ready");
  ros::spin();

  return 0;
}
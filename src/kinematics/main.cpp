#include <ros/ros.h>
#include "TracIKSolver.hpp"
#include <string>

using namespace std;

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "divided_ik_srv");
	ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle nh;

  	string urdf_param;
  	double timeout;
  
  	nh.param("timeout", timeout, 0.005);
  	nh.param("urdf_param", urdf_param, std::string("/human_description"));

  	TracIKSolver solver(argv[1], argv[2], urdf_param, timeout);
  	ros::ServiceServer div_ik_srv = nh.advertiseService(argv[3], &TracIKSolver::perform_ik, &solver);

  	ros::spin();

    return 0;
}
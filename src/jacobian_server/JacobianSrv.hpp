#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <string>
#include <map>
#include <vector>
#include "human_moveit_config/GetJacobian.h"
#include <iostream>

using namespace std;

class JacobianSrv{
private:
	map<string, const robot_state::JointModelGroup*> groups;
	robot_state::RobotStatePtr kinematic_state;

public:
	JacobianSrv(string param_description);
	~JacobianSrv();
	
	bool computeJacobian(human_moveit_config::GetJacobian::Request  &req,
		                 human_moveit_config::GetJacobian::Response &res);
};
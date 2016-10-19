#include "TracIKSolver.hpp"

TracIKSolver::TracIKSolver(const string& chain_start, const string& chain_end, const string& urdf_param, double timeout):
_tracik_solver(chain_start, chain_end, urdf_param, timeout, 1e-5){
	KDL::JntArray ll, ul; //lower joint limits, upper joint limits

  if(!(this->_tracik_solver.getKDLChain(this->_chain))) {
    ROS_ERROR("There was no valid KDL chain found");
    exit(EXIT_FAILURE);
  }

  if(!(this->_tracik_solver.getKDLLimits(ll,ul))) {
    ROS_ERROR("There were no valid KDL joint limits found");
    exit(EXIT_FAILURE);
  }

  if(!(this->_chain.getNrOfJoints() == ll.data.size())
     || !(this->_chain.getNrOfJoints() == ul.data.size())) {
      ROS_ERROR("Inconsistent joint limits found");
      exit(EXIT_FAILURE);
  }

  // Create Nominal chain configuration midway between all joint limits
  this->_nominal = new KDL::JntArray(this->_chain.getNrOfJoints());

  for (uint j=0; j < this->_nominal->data.size(); j++) {
    this->_nominal->operator()(j) = (ll(j)+ul(j))/2.0;
  }
}

TracIKSolver::~TracIKSolver(){
	delete this->_nominal;
}

KDL::JntArray TracIKSolver::JointState2JntArray(const sensor_msgs::JointState &js) {
  KDL::JntArray array(this->_chain.getNrOfJoints());
  for(uint joint=0; joint<js.position.size(); ++joint) {
      array(joint) = js.position[joint];
  }
  return array;
}

bool TracIKSolver::perform_ik(human_moveit_config::GetHumanIK::Request &request,
                              human_moveit_config::GetHumanIK::Response &response){

  int rc;
  KDL::JntArray result;
  for(uint segment=0; segment<this->_chain.getNrOfSegments(); ++segment) {
    KDL::Joint joint = this->_chain.getSegment(segment).getJoint();
    if(joint.getType()!= KDL::Joint::None)
        response.joint_state.name.push_back(joint.getName());
  }
  KDL::Frame end_effector_pose(KDL::Rotation::Quaternion(request.desired_poses[0].pose.orientation.x,
                                                         request.desired_poses[0].pose.orientation.y,
                                                         request.desired_poses[0].pose.orientation.z,
                                                         request.desired_poses[0].pose.orientation.w),
                               KDL::Vector(request.desired_poses[0].pose.position.x,
                                           request.desired_poses[0].pose.position.y,
                                           request.desired_poses[0].pose.position.z));

  this->_tracik_solver.setEpsilon(request.tolerance);
  bool seed_provided = request.seed.name.size() == this->_chain.getNrOfJoints();
  rc = this->_tracik_solver.CartToJnt(seed_provided? JointState2JntArray(request.seed): *(this->_nominal),
                                      end_effector_pose, result);

  for(uint joint=0; joint<this->_chain.getNrOfJoints(); ++joint) {
    response.joint_state.position.push_back(result(joint));
  }
  return (rc>=0);
}
#include <trac_ik/trac_ik.hpp>
#include <ros/ros.h>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <string>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include "human_moveit_config/GetHumanIK.h"

using namespace std;

class TracIKSolver {
private:
    TRAC_IK::TRAC_IK _tracik_solver;
    KDL::Chain _chain;
    KDL::JntArray *_nominal;
    
public:
    TracIKSolver(const string& chain_start, const string& chain_end, const string& urdf_param, double timeout);
    ~TracIKSolver();

    KDL::JntArray JointState2JntArray(const sensor_msgs::JointState &js);
    bool perform_ik(human_moveit_config::GetHumanIK::Request &request,
                    human_moveit_config::GetHumanIK::Response &response);
};

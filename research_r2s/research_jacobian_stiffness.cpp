#include <openrave-core.h>
#include <vector>
#include <cstring>
#include <sstream>
#include <utility>
#include <map>

#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>
#include <vector>
#include <Eigen/Dense>

using namespace OpenRAVE;
using namespace std;
using namespace Eigen;

vector<int> Getactivejoint(RobotBasePtr robot)
{
	vector<int> activejoint = {robot->GetJoint("Joint0")->GetDOFIndex(),robot->GetJoint("Joint1")->GetDOFIndex(),
	robot->GetJoint("Joint2")->GetDOFIndex(),robot->GetJoint("Joint3")->GetDOFIndex()};
	return activejoint;
}

vector<int> vector_arange(int dof)
{	
	vector<int> temp;
	for(int i=0; i<dof; i++)
	{
		temp.push_back(i);
	}
	return temp;
}

int main(int argc, char ** argv)
{

    string scenefilename = "/home/pengchang/openrave/test/research/env.xml";
    string viewername = "qtcoin"; // qtcoin

    RaveInitialize(true); // start openrave core
    EnvironmentBasePtr penv = RaveCreateEnvironment(); // create the main environment

    // set gravity
	geometry::RaveVector<double> gravity = {0,-1,0};  // gravity direction
	penv->GetPhysicsEngine()->SetGravity(gravity); 

    penv->Load(scenefilename); // load the scene
 
    vector<RobotBasePtr> vrobots;
    penv->GetRobots(vrobots);
    RobotBasePtr probot = vrobots.at(0); //get robot probot

    // get the manipulator
   	auto manip = probot->GetManipulator("cable_manip"); // if you want to use the manipulator class

	// get gravity
	geometry::RaveVector<double> vgravity = penv->GetPhysicsEngine()->GetGravity();
	

	//get the active joint index in a vector
	vector<int> activejoint = Getactivejoint(probot);

	//load the joint position velocity acceleration
	double record1 = 0.84107; // need more data
	double record2 = 0.193146;
	double record3 = -0.159449;
	double record4 = -0.519068;
	double joint1 = record1;
	double joint2 = record2-record1;
	double joint3 = record3-record2;
	double joint4 = record4-record3;
	vector<double> Position = {joint1,joint2,joint3,joint4};
	vector<double> Velocity = {0.0,0.0,0.0,0.0};
	vector<double> Acceleration = {0.0,0.0,0.0,0.0};

	//load external force
	geometry::RaveVector<double> force = {0.0,-1.96,0.0}; // ForceTorqueMap asks for geometry::RaveVector // should I change -1.96 to 1.96? // are these refer to world frame or link frame?
	geometry::RaveVector<double> torque = {0.0,0.0,0.0};
	std::pair<geometry::RaveVector<double>,geometry::RaveVector<double>> external_force (force,torque);
		//creating external force map
	KinBody::ForceTorqueMap externalforce_map; // std::map<int, std::pair<geometry::RaveVector<double>,geometry::RaveVector<double>>> externalforce_map; (also work)
		//adding the force to link_4
	externalforce_map[4] = external_force; //externalforce_map.insert(std::pair<int, std::pair<geometry::RaveVector<double>,geometry::RaveVector<double>>>(4,external_force)); (also work)

	//set the joint values
	probot->SetActiveDOFs(std::vector<int> {0, 1, 2, 3});
	probot->SetDOFValues(Position, 1, activejoint);
	probot->SetDOFVelocities(Velocity, 1, activejoint);

    // get the transform of the ee
    std::vector<OpenRAVE::Transform> transforms;
   	probot->GetLinkTransformations(transforms);
   	//OpenRAVE::Transform ee_pose = probot->GetLink("Arm3")->GetTransform(); // not working
   	//transforms[4] is for the end effector
   	cout << transforms[4].trans.x << " " << transforms[4].trans.y << " " << transforms[4].trans.z << endl;
   	cout << transforms[4].rot.x << " " << transforms[4].rot.y << " " << transforms[4].rot.z << " " << transforms[4].rot.w << endl;
   	

	//calculate the jacobian
	boost::multi_array<double,2> jacobian_translation{boost::extents[3][4]};
	// probot->CalculateJacobian(4,transforms[4].trans,jacobian_translation);
	manip->CalculateJacobian(jacobian_translation);
    cout << "displaying jacobian_translation" << endl;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++)
            cout << jacobian_translation[i][j] << " ";
        cout << endl;
    }
    cout << endl;

    boost::multi_array<double,2> jacobian_rotation{boost::extents[3][4]};
    // probot->CalculateRotationJacobian(4,transforms[4].rot,jacobian_rotation);
    manip->CalculateRotationJacobian(jacobian_rotation);
	cout << "displaying jacobian_rotation" << endl;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++)
            cout << jacobian_rotation[i][j] << " ";
        cout << endl;
    }
    cout << endl;

    boost::multi_array<double,2> jacobian{boost::extents[6][4]};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++){
        	jacobian[i][j] = jacobian_translation[i][j];
        }
    }
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++){
        	jacobian[i+3][j] = jacobian_rotation[i][j];
        }
    }
    cout << "displaying jacobian" << endl;
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 4; j++)
            cout << jacobian[i][j] << " ";
        cout << endl;
    }
    cout << endl;
    //jacobian transpose
	boost::multi_array<double,2> jacobian_transpose{boost::extents[4][6]};
	for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 6; j++)
        {
        	jacobian_transpose[i][j] = jacobian[j][i];
        }
    }

    //calculate j_transpose*external_force
    vector<double> external;
    external.push_back(jacobian_transpose[0][0]*force[0]+jacobian_transpose[0][1]*force[1]+jacobian_transpose[0][2]*force[2] + jacobian_transpose[0][3]*torque[0]+jacobian_transpose[0][4]*torque[1]+jacobian_transpose[0][5]*torque[2]);
    external.push_back(jacobian_transpose[1][0]*force[0]+jacobian_transpose[1][1]*force[1]+jacobian_transpose[1][2]*force[2] + jacobian_transpose[1][3]*torque[0]+jacobian_transpose[1][4]*torque[1]+jacobian_transpose[1][5]*torque[2]);
    external.push_back(jacobian_transpose[2][0]*force[0]+jacobian_transpose[2][1]*force[1]+jacobian_transpose[2][2]*force[2] + jacobian_transpose[2][3]*torque[0]+jacobian_transpose[2][4]*torque[1]+jacobian_transpose[2][5]*torque[2]);
    external.push_back(jacobian_transpose[3][0]*force[0]+jacobian_transpose[3][1]*force[1]+jacobian_transpose[3][2]*force[2] + jacobian_transpose[3][3]*torque[0]+jacobian_transpose[3][4]*torque[1]+jacobian_transpose[3][5]*torque[2]);


	//calculate the inverse dynamics
	vector<double> Torque_gravity_no_external;
	probot->ComputeInverseDynamics(Torque_gravity_no_external, Acceleration); // dont add external force, will add it manually
		// display inverse dynamics
		// for (size_t i=0; i<Torque_gravity.size(); i++){
		// 	std::cout << Torque_gravity.size() <<std::endl;
		// 	std::cout << Torque_gravity[i] << std::endl;
		// }

	//update the inverse dynamic with external force
	vector<double> Torque_gravity;
	for(size_t i=0; i<Torque_gravity_no_external.size();i++){
		Torque_gravity.push_back(Torque_gravity_no_external[i]-external[i]);	
	}
	
	cout << "Torque: " << endl;
	for (auto torque:Torque_gravity) {
		cout << torque << endl;
	}

	//get stiffness and damping 
	vector<double> Stiffness;
	// thought the first joint should refer to the y axis (upwards)
	Stiffness.push_back(-(double)Torque_gravity[0]/(double)(Position[0]-1.5708));
	for (size_t i=1; i<Torque_gravity.size(); i++){
		Stiffness.push_back(-(double)Torque_gravity[i]/(double)Position[i]);
	}
		// display stiffness
		// for (int i=0; i<4; i++){
		// 	std::cout << Stiffness[i] << std::endl;
		// }
	cout << "Stiffness: " << endl;
	for (auto stiff:Stiffness) {
		cout << stiff <<endl;
	}

// // Stiffness: 
// 0.710039
// 0.675703
// 0.906265
// 0.557738

	//set the viewer
	ViewerBasePtr viewer = RaveCreateViewer(penv,viewername);
	penv->Add(viewer);
	bool showgui = true;
	viewer->main(showgui);

	return 0;
}
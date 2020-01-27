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
	//load the joint position velocity acceleration
	double time1 = 1.6473;
	double time2 = 1.7393;
	double time3 = 1.7666;
	vector<double> position1 = {1.1598, 0.783581, 0.562371, 0.290721};
 	vector<double> position2 = {1.14072, 0.754111, 0.51993, 0.238344};
	vector<double> position3 = {1.13855, 0.739258, 0.505685, 0.220465};

	double vel11 = (position2[0]-position1[0])/(time2-time1);
	double vel12 = (position2[1]-position1[1])/(time2-time1);
	double vel13 = (position2[2]-position1[2])/(time2-time1);
	double vel14 = (position2[3]-position1[3])/(time2-time1); 
	vector<double> velocity1 = {vel11,vel12,vel13,vel14};

	double vel21 = (position3[0]-position2[0])/(time3-time2);
	double vel22 = (position3[1]-position2[1])/(time3-time2);
	double vel23 = (position3[2]-position2[2])/(time3-time2);
	double vel24 = (position3[3]-position2[3])/(time3-time2);
	vector<double> velocity2 = {vel21,vel22,vel23,vel24};

	double accel1 = (vel21-vel11)/(time2-time1);
	double accel2 = (vel22-vel12)/(time2-time1);
	double accel3 = (vel23-vel13)/(time2-time1);
	double accel4 = (vel24-vel14)/(time2-time1);
	vector<double> acceleration = {accel1,accel2,accel3,accel4};

	for (auto accelvalue:acceleration) {
		cout << accelvalue << endl;
	}

	vector<double> Position = {position1[0],(position1[1]-position1[0]),(position1[2]-position1[1]),(position1[3]-position1[2])};
	vector<double> Velocity = {vel11,vel12,vel13,vel14};
	vector<double> Acceleration = {accel1,accel2,accel3,accel4};

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

	//get damping 


// // Stiffness: 
// 0.710039
// 0.675703
// 0.906265
// 0.557738

	vector<double> Stiffness = {0.710039, 0.675703, 0.906265, 0.557738};
	vector<double> Damping;
	
	Damping.push_back((Torque_gravity[0]+Stiffness[0]*(Position[0]-1.5708))/Velocity[0]);
	for (size_t i=1; i<Torque_gravity.size(); i++){
		Damping.push_back((Torque_gravity[i]+Stiffness[i]*Position[i])/Velocity[i]);
	}

	cout << "Damping: " << endl;
	for (auto damp:Damping) {
		cout << damp <<endl;
	}

// // Damping:
// 7.02696
// 6.31788
// 3.5445
// 1.75954

	//set the viewer
	ViewerBasePtr viewer = RaveCreateViewer(penv,viewername);
	penv->Add(viewer);
	bool showgui = true;
	viewer->main(showgui);

	return 0;
}
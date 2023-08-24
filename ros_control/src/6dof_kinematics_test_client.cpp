/**
*
* ROS Simulation Online Course
* @uthor Min Htet Han (ROM Robotics)
* 
**/


#include <ros/ros.h>
#include <ros_control/Arm_trajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <vector>
#include <math.h>
#include <string>
#include <gazebo_msgs/GetJointProperties.h>
#include <gazebo_msgs/GetModelState.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <eigen3/Eigen/Dense>
using namespace Eigen;

// Inverse Kinematics Function
std::vector<double> Arm_InverseKinematics(geometry_msgs::Point gripper_pos) {
	double L_1 = 0.4548;//0.1548;
	double L_2 = 0.3302;
	double L_3 = 0.381;
	double d_z = 0.1524;//(4.5in + 1.5in) 0.0762
	double P_x;
	double P_y;
	double P_z;
	double theta_one, theta_two, theta_three, theta_three_two, theta_four, theta_five;
	double alpha, beta, omega, gamma, rho; // angle_values
	double a, b, c;



	MatrixXd desire_pose(3,1);
	desire_pose(0,0) = gripper_pos.x;
	desire_pose(1,0) = gripper_pos.y;
	desire_pose(2,0) = gripper_pos.z;
	ROS_INFO("desire pose is %f", gripper_pos.z);

	MatrixXd wrist_center_pose(3,1);
	MatrixXd desire_rotation(3,3);
	desire_rotation(0,0) = 0.0;  //Gripper_down_pose(0.1)=1,(1.0)=1,(2,2)=-1
	desire_rotation(0,1) = 1.0;  //Gripper belong to Fixed frame (0.0)=1,(1,1)=1,(2,2)=1
	desire_rotation(0,2) = 0.0;   //1 Gripper_Parallel_to_Ground_Plane (0,2)=1,(1,0)=1,(2,1)=1
	desire_rotation(1,0) = 1.0;  
	desire_rotation(1,1) = 0.0;  //1
	desire_rotation(1,2) = 0.0;
	desire_rotation(2,0) = 0.0;  //-1
	desire_rotation(2,1) = 0.0;
	desire_rotation(2,2) = -1.0; 

	MatrixXd desire_rotation_multiplier(3,1);
	desire_rotation_multiplier(0,0) = 0.0;
	desire_rotation_multiplier(1,0) = 0.0;
	desire_rotation_multiplier(2,0) = 1.0;

	MatrixXd rotation_3_6(3,3);


	wrist_center_pose = desire_pose - d_z*(desire_rotation * desire_rotation_multiplier);

	P_x = wrist_center_pose(0,0);
	P_y = wrist_center_pose(1,0);
	P_z = wrist_center_pose(2,0);
	ROS_INFO("Pz is %f",wrist_center_pose(2,0));
	
	

	std::vector<double> motor_joints;
	motor_joints.resize(8); 

	// Calculate theta_one, theta_two, theta_three based on wrist_center_pose(3x1)
	theta_one = atan2(P_y,P_x);
	ROS_INFO("theta_one is %f", theta_one);
	c = sqrt(pow(P_x, 2.0) + pow(P_y, 2.0));
	b = sqrt(pow(c, 2.0) + pow(P_z, 2.0));
	alpha = acos(c/b);
	beta = (M_PI/2) - alpha;
	a = sqrt((L_1*L_1) + (b*b) - 2.0*L_1*b*cos(beta));
	rho =  acos(( (L_2*L_2) + (L_3*L_3) - (a*a)) / (2.0*L_2*L_3));
	ROS_INFO("rho is %f", rho);
	theta_three = M_PI - rho;
	omega = acos(((a*a) + (L_1*L_1) - (b*b)) / (2.0*L_1*a));
	gamma = acos(((L_2*L_2) + (a*a) - (L_3*L_3)) / (2.0*L_2*a));
	theta_two = M_PI - (omega + gamma);

	MatrixXd rotation_0_3(3,3);
	rotation_0_3(0,0) = (cos(theta_one)*cos(theta_two)*cos(theta_three))-((cos(theta_one)*sin(theta_two)*sin(theta_three)));
	rotation_0_3(0,1) = -sin(theta_one);
	rotation_0_3(0,2) = (cos(theta_one)*cos(theta_two)*sin(theta_three))+(cos(theta_one)*cos(theta_three)*sin(theta_two));
	rotation_0_3(1,0) = (sin(theta_one)*cos(theta_two)*cos(theta_three))-(sin(theta_one)*sin(theta_two)*sin(theta_three));
	rotation_0_3(1,1) = cos(theta_one);
	rotation_0_3(1,2) = (sin(theta_one)*sin(theta_three)*cos(theta_two))+(sin(theta_one)*sin(theta_two)*cos(theta_three));
	rotation_0_3(2,0) = (-sin(theta_two)*cos(theta_three))-(cos(theta_two)*sin(theta_three));
	rotation_0_3(2,1) = 0.0;
	rotation_0_3(2,2) = (-sin(theta_two)*sin(theta_three))+(cos(theta_two)*cos(theta_three));

	MatrixXd rotation_0_3_t(3,3);
	rotation_0_3_t = rotation_0_3.transpose(); 
	rotation_3_6 = rotation_0_3_t * desire_rotation; 
	// theta_four = acos(rotation_3_6(2,2));

	theta_four = atan2f(sqrt(1-pow(rotation_3_6(2,2),2)) , rotation_3_6(2,2));
	//theta_three_two = acos(rotation_3_6(0,2)/sin(theta_four));
	//theta_five = asin(rotation_3_6(2,1)/sin(theta_four));
	ROS_INFO("theta_four is %f", theta_four);

	theta_three_two = atan2(rotation_3_6(1,2), rotation_3_6(0,2));
	theta_five = atan2(rotation_3_6(2,1), -rotation_3_6(2,0));

	motor_joints[0] = theta_one;
	motor_joints[1] = theta_two;
	motor_joints[2] = theta_three;
	motor_joints[3] = theta_three_two;
	motor_joints[4] = theta_four;
	motor_joints[5] = theta_five;
	motor_joints[6] = 0.0;
	motor_joints[7] = 0.0;
	return motor_joints;

}

// callback to get "result" message from action server
void done_CallBack(const actionlib::SimpleClientGoalState& state,
		const ros_control::Arm_trajectoryResultConstPtr& result) {
	ROS_INFO("Server responded with state [%s]", state.toString().c_str());
}

int main(int argc,char** argv){
	ros::init(argc, argv, "six_dof_client_node");
	ros::NodeHandle nh;

	// initialize an action client
	actionlib::SimpleActionClient<ros_control::Arm_trajectoryAction> action_client("Arm_trajectory_action", true);
	// try to connecto the action server
	bool server_exist = action_client.waitForServer(ros::Duration(5.0));
	ros::Duration sleep1s(1);
	if(!server_exist) {
		ROS_WARN("could not connect to server!!!");
		bool server_exist = action_client.waitForServer(ros::Duration(1.0));
		sleep1s.sleep();
	}
	ROS_INFO("connected to action server");

	ros_control::Arm_trajectoryGoal goal;

	// instantiate goal message

	trajectory_msgs::JointTrajectory trajectory;
	trajectory_msgs::JointTrajectoryPoint trajectory_points;

	// joint_names
	trajectory.joint_names.resize(8);
	trajectory.joint_names[0] = "joint_one";
	trajectory.joint_names[1] = "joint_two";
	trajectory.joint_names[2] = "joint_three";
	trajectory.joint_names[3] = "joint_three_two";
	trajectory.joint_names[4] = "joint_four";
	trajectory.joint_names[5] = "joint_five";
	trajectory.joint_names[6] = "gripper_one_joint";
	trajectory.joint_names[7] = "gripper_two_joint";
	// positions and velocities field
	trajectory_points.positions.resize(8);

	// initialize a service client to get joint positions(feed_back)
	ros::ServiceClient get_joint_state_client = nh.serviceClient<gazebo_msgs::GetJointProperties>(
		"/gazebo/get_joint_properties");
	gazebo_msgs::GetJointProperties get_joint_state_srv_msg;
	// initialize a service client to get model state
	ros::ServiceClient get_model_state_client = nh.serviceClient<gazebo_msgs::GetModelState>(
		"/gazebo/get_model_state");
	gazebo_msgs::GetModelState get_model_state_srv_msg;

	double dt = 1.0;
	// timer for each task
	double time = 5.0;
	double time_2 = 0.250;
	// time delay between every task
	double time_delay = 2.0;

	std::vector<double> start_joints; // start joints for each position 
	std::vector<double> end_joints;
	bool finish_before_timeout;
	start_joints.resize(8);
	end_joints.resize(8);

	ROS_INFO("Move to the home position!!!");

	// get the current_joint positions
	std::vector<double> current_joints;
	current_joints.resize(8);
	for (int i=0; i<8; i++) {
		get_joint_state_srv_msg.request.joint_name = trajectory.joint_names[i];
		get_joint_state_client.call(get_joint_state_srv_msg);
		current_joints[i] = get_joint_state_srv_msg.response.position[0];
	}

	// define current_joints as start_joints
	start_joints = current_joints;

	// define the home position
	std::vector<double> home_pose_joints;
	home_pose_joints.resize(8);
	home_pose_joints[0] = -0.785; 
	home_pose_joints[1] = 0.618; 
	home_pose_joints[2] = 1.1409; 
	home_pose_joints[3] = 0; 
	home_pose_joints[4] = 1.382; 
	home_pose_joints[5] = 0.785;
	home_pose_joints[6] = 0.0;
	home_pose_joints[7] = 0.0; 
	// assign the safe joints to end joints
	end_joints = home_pose_joints;
	// prepare goal message
	trajectory.points.clear();
	for (int i=0; i<time+1; i++) { 
	// for 8 joint in Arm
		for (int j=0; j<8; j++) {
			trajectory_points.positions[j] = end_joints[j];
		}
		trajectory_points.time_from_start = ros::Duration((double)i);
		trajectory.points.push_back(trajectory_points);
	}
	
	// copy this trajectory into our action goal
	goal.trajectory = trajectory;
	// send out the goal
	action_client.sendGoal(goal, &done_CallBack);

	finish_before_timeout = action_client.waitForResult(ros::Duration(time+2.0));
	if (!finish_before_timeout) {
		ROS_WARN("I cann't reach Home!!!(timeout)");
		return 0;
	}
	else {
		ROS_INFO("At Home Position!!!");
	}
	// Wait before next step
	ros::Duration(time_delay).sleep();

	// Line Interpolation
	for(double i=0.0; i<=1.0; i=i+0.02){
	// home pose x = 0.4, y = -0.4, z = 0.5
	double x0 = 0.4;
	double x1 = 0.4;
	double y0 = -0.4;
	double y1 = 0.4;


	// Kinematics for desire position!!!
	geometry_msgs::Point desire_position;  // float64 x, float64 y, float64 z
	// desire position
	desire_position.x = 0.0;//x0*(1-i) + x1*(i);//0.40;
	desire_position.y = 0.4;//y0*(1-i) + y1*(i);
	desire_position.z = 0.50;
	ROS_INFO("x and y is %f and %f", desire_position.x, desire_position.y);
	std::vector<double> desire_start_joints;
	
	desire_start_joints.resize(8);
	// calculate robot joints
	desire_start_joints = Arm_InverseKinematics(desire_position);

	// assign the start joints and end joints
	start_joints = home_pose_joints;
	end_joints = desire_start_joints;

	// goal message
	trajectory.points.clear();
	for (double i=0; i<time_2; i=i+0.1) { 
	// for 8 joint in Arm
		for (int j=0; j<8; j++) {
			trajectory_points.positions[j] = end_joints[j];
		}
		trajectory_points.time_from_start = ros::Duration((double)i);
		trajectory.points.push_back(trajectory_points);
	}

	// copy this trajectory into our action goal
	goal.trajectory = trajectory;
	// send out the goal
	action_client.sendGoal(goal, &done_CallBack);

	finish_before_timeout = action_client.waitForResult(ros::Duration(time_2 + 2.0));
	if (!finish_before_timeout) {
		ROS_WARN("I cann't reach desire!!!(timeout)");
		return 0;
	}
	else {
		ROS_INFO("At desire Position!!!");
	}
}
	



	return 0;

}

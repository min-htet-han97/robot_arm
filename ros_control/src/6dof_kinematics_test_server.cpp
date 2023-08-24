/**
*
* ROS Simulation Online Course
* @uthor Min Htet Han (ROM Robotics)
* 
**/


#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <ros_control/Arm_trajectoryAction.h>
#include <std_msgs/Float64.h>
#include <vector>
#include <string>
const double dt = 0.005;//0.005; // resolution for interpolating the trajectory
const double dt_min = 0.05; //0.1 time step


class TrajectoryActionServer {
public:
	TrajectoryActionServer(ros::NodeHandle* nodehandle); //constructor
	~TrajectoryActionServer(void) {}
private:
	void send_joint_commands_(std::vector<double> cmd_jnts);
	void execute_CallBack(const actionlib::SimpleActionServer<ros_control::Arm_trajectoryAction>::GoalConstPtr& goal);
	


	ros::NodeHandle nh_; // nodehandle
	
	std::vector<ros::Publisher> pos_cmd_publisher_;  // initialization will be in execute_CallBack()
	actionlib::SimpleActionServer<ros_control::Arm_trajectoryAction> actionServer_;
	

	// message types for the action
	ros_control::Arm_trajectoryActionGoal goal_; // goal message
	ros_control::Arm_trajectoryActionResult result_; // result message
	ros_control::Arm_trajectoryActionFeedback feedback_; // feedback message, not used

};

TrajectoryActionServer::TrajectoryActionServer(ros::NodeHandle* nodehandle):
	nh_(*nodehandle), 
	actionServer_(nh_, "Arm_trajectory_action", boost::bind(&TrajectoryActionServer::execute_CallBack, this, _1), false)
	
{
	ROS_INFO("In constructor of TrajectoryActionServer...");
	actionServer_.start(); // Start Arm_trajectory_action Server
	
}

void TrajectoryActionServer::send_joint_commands_(std::vector<double> cmd_jnts) {
	int npublishers = pos_cmd_publisher_.size();
	if (cmd_jnts.size() == npublishers) { // dimension of pos_cmd and publisher is same
		for (int i=0; i<npublishers; i++) {
			std_msgs::Float64 cmd_msg;
			cmd_msg.data = cmd_jnts[i];
			pos_cmd_publisher_[i].publish(cmd_msg);
		}
	}
	else
		ROS_WARN("joint commanders and publishers are not consistent!");
}


void TrajectoryActionServer::execute_CallBack(const actionlib::SimpleActionServer<ros_control::Arm_trajectoryAction>::GoalConstPtr& goal) {
	ROS_INFO("in execute_CallBack...");
	
	trajectory_msgs::JointTrajectory trajectory = goal -> trajectory;
	int no_of_joints = trajectory.joint_names.size(); // get the number of joints
	int no_of_points = trajectory.points.size(); // get the number of points



	ROS_INFO("trajectory message commands %d joint(s) and %d", no_of_joints, no_of_points);
	//initialize command publishers for each joints
	pos_cmd_publisher_.resize(no_of_joints);
	for (int i=0; i<no_of_joints; i++) {
		pos_cmd_publisher_[i] = nh_.advertise<std_msgs::Float64>("/six_dof_arm/" + trajectory.joint_names[i] + "_position_controller/command", 1, true);
	}
	
	std::vector<double> cmd_jnts; // interpolated joints to be published
	std::vector<double> motor_joints;

	cmd_jnts.resize(no_of_joints);

	// time control parameters
	double t_final = trajectory.points[no_of_points - 1].time_from_start.toSec();
		// the first point should be the current point in the gazebo(push back loop in client)
		// so the first time_from_start should be 0.0
	double t_previous = trajectory.points[0].time_from_start.toSec();
	double t_next = trajectory.points[1].time_from_start.toSec();
	double t_stream = t_previous; // initializee to t_previous
	

	// check the validality of time range
	double t_range = t_next - t_previous;
	if (t_range < dt_min) {
		ROS_WARN("time step invalid in trajectory! (begining)");
		// as_.setAborted(result_);
		actionServer_.setAborted();
		return;
	}

	int ipt_next = 1; // index these points, start from the first one
	
	motor_joints = trajectory.points[0].positions;
	// start interpolation, use time to control this process
	ros::Rate rate_timer(1/dt);
	while (t_stream <= t_final) { // check if current time exceeds final time		

		for (int i=0; i<no_of_joints; i++) {
			cmd_jnts[i] = motor_joints[i]; 
			ROS_INFO(" joints are %f " , cmd_jnts[i]);
		}
		send_joint_commands_(cmd_jnts); // send these joint position to controller

		// time control
		rate_timer.sleep();
		t_stream = t_stream + dt;
	}

	// output the final point
	cmd_jnts = trajectory.points[no_of_points - 1].positions;
	send_joint_commands_(cmd_jnts);
	//actionServer_.setSucceeded(result_); // finally successful on this action...
	actionServer_.setSucceeded();
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "six_dof_server_node");
	ros::NodeHandle nh;	

	ROS_INFO("instantiating an object of class TrajectoryActionServer...");
	TrajectoryActionServer actionServer_object(&nh);

	ROS_INFO("going into spin...");

	while (ros::ok()) {
		ros::spinOnce();
		ros::Duration(0.1).sleep();
	}

	return 0;
}
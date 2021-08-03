#include <smb_highlevel_controller/Controller.hpp>
#include <string.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <boost/thread/thread.hpp>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>

double max_yaw = 0;
namespace controller
{

    Controller::Controller(ros::NodeHandle &nodeHandle) : nodeHandle_(nodeHandle) // @suppress("Class members should be properly initialized")
    {
        //ros::init(0, 0, "tf2_listener");
        std::string topic;
        int queue_size;
        std::string odom_topic;
        if (not nodeHandle_.getParam("topic", topic))
        {
            ROS_ERROR("Parameter for topic not found");
        }
        if (not nodeHandle_.getParam("queue_size", queue_size))
        {
            ROS_ERROR("Parameter for topic not found");
        }
        if (not nodeHandle_.getParam("odom_topic", odom_topic))
        {
            ROS_ERROR("Parameter for odom topic not found");
        }

        vis_pub_ = nodeHandle_.advertise<visualization_msgs::Marker>("/visualization_marker", queue_size);
        cmd_vel_publisher_ = nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel", queue_size);
        scanSubscriber_ = nodeHandle_.subscribe(topic, queue_size, &Controller::scanCallback, this);

        odomSubscriber_ = nodeHandle_.subscribe(odom_topic, queue_size, &Controller::odomCallback, this);
        arm_pub = nodeHandle_.advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command",1);
        gripper_pub = nodeHandle_.advertise<trajectory_msgs::JointTrajectory>("/gripper_controller/command",1);
        // timer_ = nodeHandle_.createTimer(ros::Duration(2.0), &Controller::timerCallback, this, true);		//--> debugging
    }

    Controller::~Controller()
    {
    }

	void Controller::scanCallback( const sensor_msgs::LaserScan::ConstPtr &msg) {

		// scan for ranges
		int ranges_length = msg->ranges.size();
		float front_direction_index = msg->ranges[ranges_length/2];

		float range_min = msg->ranges[0];
		float range_min_index = 0;
		//ROS_INFO("Number of ranges: %i", ranges_length);
		for(int i = 0; i < ranges_length; ++i){
			if(msg->ranges[i] < range_min and (i < 144 or i > 215)){	// between 144 and 215 is just sensor detecting the robot itself
				range_min = msg->ranges[i];
				range_min_index = i;
			}
			//ROS_INFO("Sensor num: %i          range: %f",i, msg->ranges[i]);
		}

		// print info
		ROS_INFO_STREAM_THROTTLE(2.0,"Range min: " << range_min);
		ROS_INFO_STREAM_THROTTLE(2.0,"Range front: " << msg->ranges[front_direction_index]);
		ROS_INFO_STREAM_THROTTLE(2.0,"x:" <<  x);
		ROS_INFO_STREAM_THROTTLE(0.2,"yaw:" <<  yaw);
		//ROS_INFO_STREAM_THROTTLE(0.5,"max_yaw:" <<  max_yaw);




		// START THE ROBOT	x:-1.24    y:-0.57
		geometry_msgs::Twist cmd_val_values;
		float arm_pose_default[4] = {0.0, 0.0, 0.0, 0.0};
		float arm_pose_up[4] = {0.0, 0.7, 0.1, -0.22};


		// 0-1: go infront of batteries and turn left
		if(stage == 0 && Controller::goForward(-0.15, true))
			stage = 1;
		if(stage == 1 && Controller::turn(1.0))
			stage = 2;

		// 2-5: pick up the battery
		float arm_pose_down11[4] = {0.0, -0.14, -0.11, -0.22};
		if(stage == 2 && Controller::controllGripper(true)) // --> open gripper
			stage = 3;
		if(stage == 3 && Controller::controllArm(arm_pose_up)) // --> move arm on the block
			stage = 4;
		if(stage == 4 && Controller::controllGripper(false)) // --> close gripper
			stage = 5;
		if(stage == 5 && Controller::controllArm(arm_pose_down11)) // --> move arm up
			stage = 6;

		// 6-13: put battery in first blue car
		float arm_pose_down12[4] = {0.0, 0.9, -0.34, -0.22};
		if(stage == 6 && Controller::turn(2.0))	//--> turn back
			stage = 7;
		if(stage == 7 && Controller::goForward(-0.55, true))	//--> go back to the blue car
			stage = 8;
		if(stage == 8 && Controller::turn(3.0))	// --> turn right
			stage = 9;
		if(stage == 9 && Controller::aproachObjectMinDistance(msg->ranges[front_direction_index], 0.26))	// --> approach blue car
			stage = 10;
		if(stage == 10 && Controller::controllArm(arm_pose_down12)) // --> move arm on the block
			stage = 11;
		if(stage == 11 && Controller::controllGripper(true)) // --> open gripper
			stage = 12;
		if(stage == 12 && Controller::controllArm(arm_pose_default)) // --> move arm to default position
			stage = 13;
		// CHECKPOINT 1 postition: x:-0.52    y:-0.58
		// 13+ do the same for other 3 batteries
		// blue car x positions:-0.1, 0.3, 0.7

		// fill second car
		if(stage == 13 && Controller::turn(0.0))	// --> turn forward
			stage = 14;
		if(stage == 14 && Controller::goForward(-0.06, true))
			stage = 15;
		if(stage == 15 && Controller::turn(1.0))
			stage = 16;
		float arm_pose_down21[4] = {0.0, -0.14, -0.11, -0.22};
		if(stage == 16 && Controller::controllGripper(true)) // --> open gripper
			stage = 17;
		if(stage == 17 && Controller::controllArm(arm_pose_up)) // --> move arm on the block
			stage = 18;
		if(stage == 18 && Controller::controllGripper(false)) // --> close gripper
			stage = 19;
		if(stage == 19 && Controller::controllArm(arm_pose_down21)) // --> move arm up
			stage = 20;
		float arm_pose_down22[4] = {0.0, 0.9, -0.34, -0.22};/*
		if(stage == 20 && Controller::turn(2.0))	//--> turn back
			stage = 21;
		if(stage == 21 && Controller::goForward(-0.55, true))	//--> go to the blue car n2
			stage = 22;*/
		if(stage == 22 && Controller::turn(3.0))	// --> turn right
			stage = 23;
		if(stage == 23 && Controller::aproachObjectMinDistance(msg->ranges[front_direction_index], 0.26))	// --> approach blue car
			stage = 24;
		if(stage == 24 && Controller::controllArm(arm_pose_down22)) // --> move arm on the block
			stage = 25;
		if(stage == 25 && Controller::controllGripper(true)) // --> open gripper
			stage = 26;
		if(stage == 26 && Controller::controllArm(arm_pose_default)) // --> move arm to default position
			stage = 27;
		// CHECKPOINT 2 postition: x:-0.127    y:-0.58

		// fill third car
		if(stage == 27 && Controller::turn(0.0))	// --> turn forward
			stage = 28;
		if(stage == 28 && Controller::goForward(0.1, true))
			stage = 29;
		if(stage == 29 && Controller::turn(1.0))
			stage = 34;
		float arm_pose_down31[4] = {0.0, -0.14, -0.11, -0.22};
		if(stage == 30 && Controller::controllGripper(true)) // --> open gripper
			stage = 31;
		if(stage == 31 && Controller::controllArm(arm_pose_up)) // --> move arm on the block
			stage = 32;
		if(stage == 32 && Controller::controllGripper(false)) // --> close gripper
			stage = 33;
		if(stage == 33 && Controller::controllArm(arm_pose_down31)) // --> move arm up
			stage = 34;
		float arm_pose_down32[4] = {0.0, 0.9, -0.34, -0.22};
		if(stage == 34 && Controller::turn(0.0))	//--> turn forward
			stage = 35;
		if(stage == 35 && Controller::goForward(0.37, true))	//--> go to the blue car n3
			stage = 36;
		if(stage == 36 && Controller::turn(3.0))	// --> turn right
			stage = 37;
		if(stage == 37 && Controller::aproachObjectMinDistance(msg->ranges[front_direction_index], 0.26))	// --> approach blue car
			stage = 38;
		if(stage == 38 && Controller::controllArm(arm_pose_down32)) // --> move arm on the block
			stage = 39;
		if(stage == 39 && Controller::controllGripper(true)) // --> open gripper
			stage = 40;
		if(stage == 40 && Controller::controllArm(arm_pose_default)) // --> move arm to default position
			stage = 41;






		// DEBUGGING
		if(stage == -1 && Controller::turn(3.0))	//--> turn right
			stage = -2;
		if(stage == -2 && Controller::turn(2.0))	//--> turn back
			stage = -3;
		if(stage == -3 && Controller::turn(1.0))	//--> turn left
			stage = -4;



	}

    void Controller::timerCallback(const ros::TimerEvent &e)	// for debugging
    {
    	Controller::controllGripper(true);
        ros::Duration(5).sleep();
    	Controller::controllGripper(false);
    }

    bool Controller::aproachObjectMinDistance(float front_dir_sen_length, float goal_distance){
    	if(front_dir_sen_length <= goal_distance ){
			ROS_INFO("Finished approaching            stage: %i", stage);
			ros::Duration(2.0).sleep();
    		return true;
    	}
    	else{
    		if (cmd_val_values_.linear.x == 0.0)
    			ROS_INFO("Start approaching            stage: %i", stage);

			cmd_val_values_.linear.x = 0.02;
			cmd_vel_publisher_.publish(cmd_val_values_);
			return false;
    	}
    }

    bool Controller::goForward(double xy_goal, bool xy){	// booy xy tells us which is our target variable --> true = x, false = y
    	double error = -1;
    	if(xy)
    		error = abs(xy_goal - x);
    	else
    		error = abs(xy_goal - y);

		if (error > 0.3){
			cmd_val_values_.linear.x = 0.3;
		}
		else if(error > 0.1){
			cmd_val_values_.linear.x = 0.2;
			cmd_val_values_.angular.z = 0.0;
		}
		else{
			cmd_val_values_.angular.z = 0.0;
			cmd_val_values_.linear.x = 0.0;
			cmd_vel_publisher_.publish(cmd_val_values_);
			ROS_INFO("Finished going forward            stage: %i", stage);
			ros::Duration(2.0).sleep();
			return true;
		}
		cmd_vel_publisher_.publish(cmd_val_values_);
		return false;
	}



    bool Controller::turn(double direction)
    { // direction = 0 --> forward; direction 1 --> left; direction 2 --> back; direction 3 --> right
        geometry_msgs::Twist cmd_val_values;

        double base_turn_yaw = 1.571;	// maybe 1.56424
        double target_yaw = direction * base_turn_yaw;
        double multiplier = 1.0;
        if(direction == 2.0)
        	target_yaw = 2 * base_turn_yaw;  // 3.13;
        if(direction == 3.0)
        	target_yaw =  -1.0 * base_turn_yaw;



		if(not rotating){
			ROS_INFO("Turning in direction %f        target yaw: %f", direction, target_yaw);
			cmd_val_values.angular.z = 0.5;
			rotating = true;
		}
		else{
			double error = abs(target_yaw - yaw);
			if (error > 0.3){
				cmd_val_values_.angular.z = multiplier * 0.5;
			}
			else if(error > 0.01){
				cmd_val_values_.angular.z = multiplier * 0.05;
			}
			else{
				cmd_val_values_.angular.z = 0.0;
				cmd_vel_publisher_.publish(cmd_val_values_);
				ROS_INFO("Finished turning            stage: %i", stage);
				rotating = false;
				ros::Duration(2.0).sleep();
				return true;
			}
		}
		cmd_vel_publisher_.publish(cmd_val_values_);
		return false;
    }


    bool Controller::controllArm(float pose[4])
    {
    	if(moving_arm)
    			return false;

    	moving_arm = true;
        ROS_INFO("Start moving arm            stage: %i", stage);
        trajectory_msgs::JointTrajectory traj;
        trajectory_msgs::JointTrajectoryPoint points_n;

        traj.header.frame_id = "";
        traj.joint_names.resize(4);
        traj.points.resize(1);
        traj.points[0].positions.resize(4);

        traj.joint_names[0] ="joint1";
        traj.joint_names[1] ="joint2";
        traj.joint_names[2] ="joint3";
        traj.joint_names[3] ="joint4";

        traj.header.stamp = ros::Time::now();
        traj.points[0].positions[0] = pose[0];
        traj.points[0].positions[1] = pose[1];
        traj.points[0].positions[2] = pose[2];
        traj.points[0].positions[3] = pose[3];
        traj.points[0].time_from_start = ros::Duration(1);

        arm_pub.publish(traj);
        ros::spinOnce();
        ros::Duration(4.0).sleep();
        ROS_INFO("Finish moving arm            stage: %i", stage);
        moving_arm = false;
        return true;
    }

    bool Controller::controllGripper(bool openClose)	// open = True, close = false
    {
        float pose = 0.0;
        if(openClose){
        	ROS_INFO("Opening gripper            stage: %i", stage);
        	pose = 0.015;
        }
        else{
        	ROS_INFO("Closing gripper            stage: %i", stage);
        	pose = 0.0;
        }

        trajectory_msgs::JointTrajectory traj;
        trajectory_msgs::JointTrajectoryPoint points_n;

        traj.header.frame_id = "";
        traj.joint_names.resize(1);
        traj.points.resize(1);
        traj.points[0].positions.resize(1);
        traj.points[0].effort.resize(1);

        traj.joint_names[0] ="gripper";

        traj.header.stamp = ros::Time::now();
        traj.points[0].positions[0] = pose;
        traj.points[0].time_from_start = ros::Duration(1);
        traj.points[0].effort[0] = 0.0000000000000000001;

        gripper_pub.publish(traj);
        ros::Duration(2.35).sleep();
        ROS_INFO("Finish moving gripper            stage: %i", stage);
        return true;

    }

    // Update x, y and yaw values
    void Controller::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        x = msg->pose.pose.position.x;
        y = msg->pose.pose.position.y;

        tf::Quaternion RQ2;
        double roll, pitch, yaaw;
        tf::quaternionMsgToTF(msg->pose.pose.orientation, RQ2);
        tf::Matrix3x3(RQ2).getRPY(roll, pitch, yaaw);
        yaw = yaaw;
        if(yaw > max_yaw)
        	max_yaw = yaw;
        // ROS_INFO("x [%f]", x);
        // ROS_INFO("y [%f]", y);
        // ROS_INFO("yaw [%f]", yaw);
    }


} // namespace smb_highlevel_controller

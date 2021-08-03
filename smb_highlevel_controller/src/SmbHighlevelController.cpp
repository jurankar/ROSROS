#include <smb_highlevel_controller/SmbHighlevelController.hpp>
#include <string.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

int stage = 0;

namespace smb_highlevel_controller {

	SmbHighlevelController::SmbHighlevelController(ros::NodeHandle &nodeHandle) :    nodeHandle_(nodeHandle){
		//ros::init(0, 0, "tf2_listener");
		std::string topic;
		int queue_size;
		if(not nodeHandle_.getParam("topic", topic)){
			ROS_ERROR("Parameter for topic not found");
		}
		if(not nodeHandle_.getParam("queue_size", queue_size)){
			ROS_ERROR("Parameter for topic not found");
		}

		vis_pub_ = nodeHandle_.advertise<visualization_msgs::Marker>( "/visualization_marker", queue_size );
		cmd_vel_publisher_ = nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel", queue_size);
		scanSubscriber_ = nodeHandle_.subscribe(topic, queue_size, &SmbHighlevelController::scanCallback, this);








	}

	SmbHighlevelController::~SmbHighlevelController() {
	}


	void SmbHighlevelController::scanCallback( const sensor_msgs::LaserScan::ConstPtr &msg) {


		/*
		static tf2_ros::TransformBroadcaster transBroadcaster;
		geometry_msgs::TransformStamped transformStamped;

		ROS_INFO("Before the move:");
		ros::Duration(2.0).sleep();
		ROS_INFO("Start the move:");
		transformStamped.header.stamp = ros::Time::now();
		transformStamped.header.frame_id = "base_link";
		transformStamped.child_frame_id = "my_stick";
		transformStamped.transform.translation.x = 0.15;
		transformStamped.transform.translation.y = 1.0;
		transformStamped.transform.translation.z = 0.02;
		tf2::Quaternion quat;
		quat.setRPY(0.0, 2.2, 0.0);
		transformStamped.transform.rotation.x = quat.x();
		transformStamped.transform.rotation.y = quat.y();
		transformStamped.transform.rotation.z = quat.z();
		transformStamped.transform.rotation.w = quat.w();
		transBroadcaster.sendTransform(transformStamped);
		ROS_INFO("Transform finished");
		//ros::Duration(200.0).sleep();
		*/


		/*
		static tf::TransformBroadcaster br;
		tf::Transform transform;
		transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
		tf::Quaternion q;
		q.setRPY(0, 1.5, 0);
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "my_stick"));
		ROS_INFO("Transform finished");
		*/




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
		//ros::Duration(200).sleep();


		geometry_msgs::Twist cmd_val_values;

		// stage 0 --> go infront of batteries
		if(stage == 0){
			ROS_INFO("Going forward");
			cmd_val_values.linear.x = 0.75;
			cmd_vel_publisher_.publish(cmd_val_values);

			if(range_min < 0.25){
				cmd_val_values.linear.x = 0.3;
				cmd_vel_publisher_.publish(cmd_val_values);
			}

			if(range_min < 0.15){
				ROS_INFO("Stopping");
				cmd_val_values.linear.x = 0.0;
				cmd_vel_publisher_.publish(cmd_val_values);
				stage = 1;
			}
		}

		else if(stage == 1){
			// pick up a block like a forklift would
			stage = 2;
		}


		ROS_INFO_STREAM_THROTTLE(0.1,"Range min: " << range_min);
		//ROS_INFO_STREAM_THROTTLE(0.25,"Range 0: " << msg->ranges[0]);
		//ROS_INFO_STREAM_THROTTLE(0.25,"Range 256: " << msg->ranges[ranges_length-1]);


	}

	void SmbHighlevelController::turn(int direction){	// direction = 1 --> left 90* ....direction = 2 --> right 90*		--> DOESNT WORK YET
		geometry_msgs::Twist cmd_val_values;


		cmd_val_values.linear.x = 0.0;
		if(direction == 1){
			ROS_INFO("Turning left");
			cmd_val_values.angular.z = 1.0;
		}
		else if(direction == 2){
			ROS_INFO("Turning right");
			cmd_val_values.angular.z = -1.0;
		}
		cmd_vel_publisher_.publish(cmd_val_values);
		ros::Duration(2.35).sleep();
		cmd_val_values.angular.z = 0.0;
		cmd_vel_publisher_.publish(cmd_val_values);

	}

}  // namespace smb_highlevel_controller

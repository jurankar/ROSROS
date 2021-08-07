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
		int front_direction_index = ranges_length/2;

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
		ROS_INFO_STREAM_THROTTLE(5.0,"Range min: " << range_min << "     laser_index: " << range_min_index);
		ROS_INFO_STREAM_THROTTLE(5.0,"Range front: " << msg->ranges[front_direction_index] << "     front_index: " << front_direction_index);
		ROS_INFO_STREAM_THROTTLE(5.0,"x:" <<  x << "  y:" << y << "   yaw:" <<  yaw << "\n");
		//ROS_INFO_STREAM_THROTTLE(5.0,"max_yaw:" <<  max_yaw);
		//ROS_INFO_STREAM_THROTTLE(5.0, msg->ranges);




		// START THE ROBOT	x:-1.24    y:-0.57
		geometry_msgs::Twist cmd_val_values;
		float arm_pose_default[4] = {0.0, 0.0, 0.0, 0.0};
		float arm_pose_battery_down[4] = {0.0, 0.7, 0.1, -0.22};
		float arm_pose_battery_up[4] = {0.0, -0.14, -0.11, -0.22};
		float arm_pose_battery_in_car[4] = {0.0, 0.9, -0.34, -0.22};

		float arm_pose_back[4] = {0.0, -0.7, -0.5, 0.0};


		//	FILL THE CAR BATTERIES

		// 0-1: go infront of batteries and turn left
		if(stage == 0 && Controller::goForward(-0.15, true))
			stage = 1;
		if(stage == 1 && Controller::turn(1.0, true))
			stage = 2;

		// 2-5: pick up the battery
		if(stage == 2 && Controller::controllGripper(true)) // --> open gripper
			stage = 3;
		if(stage == 3 && Controller::controllArm(arm_pose_battery_down)) // --> move arm on the block
			stage = 4;
		if(stage == 4 && Controller::controllGripper(false)) // --> close gripper
			stage = 5;
		if(stage == 5 && Controller::controllArm(arm_pose_battery_up)) // --> move arm up
			stage = 6;

		// 6-13: put battery in first blue car
		if(stage == 6 && Controller::turn(2.0, true))	//--> turn back
			stage = 7;
		if(stage == 7 && Controller::goForward(-0.55, true))	//--> go back to the blue car
			stage = 8;
		if(stage == 8 && Controller::turn(3.0, true))	// --> turn right
			stage = 9;
		if(stage == 9 && Controller::aproachObjectMinDistance(msg->ranges[front_direction_index], 0.26))	// --> approach blue car
			stage = 10;
		if(stage == 10 && Controller::controllArm(arm_pose_battery_in_car)) // --> move arm on the block
			stage = 11;
		if(stage == 11 && Controller::controllGripper(true)) // --> open gripper
			stage = 12;
		if(stage == 12 && Controller::controllArm(arm_pose_default)) // --> move arm to default position
			stage = 13;
		// CHECKPOINT 1 postition: x:-0.52    y:-0.58
		// 13+ do the same for other 3 batteries
		// blue car x positions:-0.1, 0.3, 0.7

		// fill second car
		if(stage == 13 && Controller::turn(0.0, true))	// --> turn forward
			stage = 14;
		if(stage == 14 && Controller::goForward(-0.06, true))
			stage = 15;
		if(stage == 15 && Controller::turn(1.0, true))
			stage = 16;
		if(stage == 16 && Controller::controllGripper(true)) // --> open gripper
			stage = 17;
		if(stage == 17 && Controller::controllArm(arm_pose_battery_down)) // --> move arm on the block
			stage = 18;
		if(stage == 18 && Controller::controllGripper(false)) // --> close gripper
			stage = 19;
		if(stage == 19 && Controller::controllArm(arm_pose_battery_up)) // --> move arm up
			stage = 22;
		if(stage == 20 && Controller::turn(2.0, true))	//--> turn back
			stage = 21;
		if(stage == 21 && Controller::goForward(-0.55, true))	//--> go to the blue car n2
			stage = 22;
		if(stage == 22 && Controller::turn(3.0, true))	// --> turn right
			stage = 23;
		if(stage == 23 && Controller::aproachObjectMinDistance(msg->ranges[front_direction_index], 0.26))	// --> approach blue car
			stage = 24;
		if(stage == 24 && Controller::controllArm(arm_pose_battery_in_car)) // --> move arm on the block
			stage = 25;
		if(stage == 25 && Controller::controllGripper(true)) // --> open gripper
			stage = 26;
		if(stage == 26 && Controller::controllArm(arm_pose_default)) // --> move arm to default position
			stage = 27;
		// CHECKPOINT 2 postition: x:-0.127    y:-0.58

		// fill third car
		if(stage == 27 && Controller::turn(0.0, true))	// --> turn forward
			stage = 28;
		if(stage == 28 && Controller::goForward(0.1, true))
			stage = 29;
		if(stage == 29 && Controller::turn(1.0, true))
			stage = 30;
		if(stage == 30 && Controller::controllGripper(true)) // --> open gripper
			stage = 31;
		if(stage == 31 && Controller::controllArm(arm_pose_battery_down)) // --> move arm on the block
			stage = 32;
		if(stage == 32 && Controller::controllGripper(false)) // --> close gripper
			stage = 33;
		if(stage == 33 && Controller::controllArm(arm_pose_battery_up)) // --> move arm up
			stage = 34;
		if(stage == 34 && Controller::turn(0.0, false))	//--> turn forward
			stage = 35;
		if(stage == 35 && Controller::goForward(0.31, true))	//--> go to the blue car n3
			stage = 36;
		if(stage == 36 && Controller::turn(3.0, false))	// --> turn right
			stage = 37;
		if(stage == 37 && Controller::aproachObjectMinDistance(msg->ranges[front_direction_index], 0.26))	// --> approach blue car
			stage = 38;
		if(stage == 38 && Controller::controllArm(arm_pose_battery_in_car)) // --> move arm on the block
			stage = 39;
		if(stage == 39 && Controller::controllGripper(true)) // --> open gripper
			stage = 40;
		if(stage == 40 && Controller::controllArm(arm_pose_default)) // --> move arm to default position
			stage = 41;
		// CHECKPOINT 3 postition: x:0.3   y:-0.58

		// fill fourth car
		if(stage == 41 && Controller::turn(2.0, false))	// --> turn backward
			stage = 42;
		if(stage == 42 && Controller::goForward(0.17, true))
			stage = 43;
		if(stage == 43 && Controller::turn(1.0, false))
			stage = 44;
		if(stage == 44 && Controller::controllGripper(true)) // --> open gripper
			stage = 45;
		if(stage == 45 && Controller::controllArm(arm_pose_battery_down)) // --> move arm on the block
			stage = 46;
		if(stage == 46 && Controller::controllGripper(false)) // --> close gripper
			stage = 47;
		if(stage == 47 && Controller::controllArm(arm_pose_battery_up)) // --> move arm up
			stage = 48;
		if(stage == 48 && Controller::turn(0.0, false))	//--> turn forward
			stage = 49;
		if(stage == 49 && Controller::goForward(0.72, true))	//--> go to the blue car n3
			stage = 50;
		if(stage == 50 && Controller::turn(3.0, false))	// --> turn right
			stage = 51;
		if(stage == 51 && Controller::aproachObjectMinDistance(msg->ranges[front_direction_index], 0.26))	// --> approach blue car
			stage = 52;
		if(stage == 52 && Controller::controllArm(arm_pose_battery_in_car)) // --> move arm on the block
			stage = 53;
		if(stage == 53 && Controller::controllGripper(true)) // --> open gripper
			stage = 54;
		if(stage == 54 && Controller::controllArm(arm_pose_default)) // --> move arm to default position
			stage = 55;
		// CHECKPOINT 3 postition: x:0.66   y:-0.58


		//	PICK UP THE SIGNS AND PUT THEM AROUND CAR
		float arm_pose_sign_down[4] = {0.0, 0.60, 0.17, -0.46};
		float arm_pose_sign_up[4] = {0.0, -0.14, -0.11, -0.22};
		float arm_pose_place_sign[4] = {0.0, 0.83, -0.2, -0.22};

		// go to signs
		if(stage == 55 && Controller::goForward(0.8, true))
			stage = 56;
		if(stage == 56 && Controller::turn(1.0, true))	// --> turn left
			stage = 57;
		if(stage == 57 && Controller::goForward(0.3, false))
			stage = 58;
		if(stage == 58 && Controller::turn(2.0, true))	// --> turn backward
			stage = 59;
		if(stage == 59 && Controller::goForward(0.25, true))
			stage = 60;
		if(stage == 60 && Controller::turn(3.0, false))	// --> turn right	TODO SET TO TRUEEEE
			stage = 61;

		// pick up and place first sign
		if(stage == 61 && Controller::controllArm(arm_pose_back)) // --> move arm to back postition, so the robot also leans back so the sensors see more
			stage = 62;
		if(stage == 62 && Controller::allignWithObject(front_direction_index, range_min_index))	// --> allign with the object
			stage = 63;
		if(stage == 63 && Controller::aproachObjectMinDistance(range_min, 0.2))	// --> approach sign
			stage = 64;
		if(stage == 64 && Controller::allignWithObject(front_direction_index, range_min_index))	// --> allign with the object again
			stage = 65;
		if(stage == 65 && Controller::controllGripper(true)) // --> open gripper
			stage = 66;
		if(stage == 66 && Controller::controllArm(arm_pose_sign_down)) // --> move arm on the sign
			stage = 67;
		if(stage == 67 && Controller::controllGripper(false)) // --> close gripper
			stage = 68;
		if(stage == 68 && Controller::controllArm(arm_pose_sign_up)) // --> move arm up with the sign in hand
			stage = 69;
		if(stage == 69 && Controller::turn(2.0, true)) // --> turn backward	(can't do false because of slippage on the map)
			stage = 70;
		if(stage == 70 && Controller::goForward(-0.65, true))
			stage = 71;
		if(stage == 71 && Controller::turn(1.0, false)) // --> turn left
			stage = 72;
		if(stage == 72 && Controller::goForward(0.83, false))
			stage = 73;
		if(stage == 73 && Controller::turn(2.0, true)) // --> turn backward
			stage = 74;
		if(stage == 74 && Controller::goForward(-1.2, true))
			stage = 75;
		//put sign on the ground
		if(stage == 75 && Controller::controllArm(arm_pose_battery_in_car)) // --> move arm on the block
			stage = 76;
		if(stage == 76 && Controller::controllGripper(true)) // --> open gripper
			stage = 77;
		if(stage == 77 && Controller::controllArm(arm_pose_default)) // --> move arm to default position
			stage = 78;
		// go back to signs
		if(stage == 78 && Controller::turn(0.0, true)) // --> turn forward
			stage = 79;
		if(stage == 79 && Controller::goForward(-0.7, true))
			stage = 80;
		if(stage == 80 && Controller::turn(3.0, false)) // --> turn right
			stage = 81;
		if(stage == 81 && Controller::goForward(0.27, false))
			stage = 82;
		if(stage == 82 && Controller::turn(0.0, true)) // --> turn forward
			stage = 83;
		if(stage == 83 && Controller::goForward(0.03, true))
			stage = 84;
		if(stage == 84 && Controller::turn(3.0, false))
			stage = 85;
		//CHECKPOINT x: -0.02     y:0.215

		// pick up 3 other signs
		double place_sign_y[3] = {0.65, 0.64, 0.7};
		double place_sign_x[3] = {-1.125, -0.81, -0.6};
		double return_to_signs_x[2] = {-0.123, -0.4};	// we don't have array for y because it is constant 0.27
		for(int i = 1; i <= 3; i++){
			int num_of_stages = 24;

			if(stage == 61+i*num_of_stages && Controller::controllArm(arm_pose_back)) // --> move arm to back postition, so the robot also leans back so the sensors see more
				stage = 62+i*num_of_stages;
			if(stage == 62+i*num_of_stages && Controller::allignWithObject(front_direction_index, range_min_index))	// --> allign with the object
				stage = 63+i*num_of_stages;
			if(stage == 63+i*num_of_stages && Controller::aproachObjectMinDistance(range_min, 0.2))	// --> approach sign
				stage = 64+i*num_of_stages;
			if(stage == 64+i*num_of_stages && Controller::allignWithObject(front_direction_index, range_min_index))	// --> allign with the object again
				stage = 65+i*num_of_stages;
			if(stage == 65+i*num_of_stages && Controller::controllGripper(true)) // --> open gripper
				stage = 66+i*num_of_stages;
			if(stage == 66+i*num_of_stages && Controller::controllArm(arm_pose_sign_down)) // --> move arm on the sign
				stage = 67+i*num_of_stages;
			if(stage == 67+i*num_of_stages && Controller::controllGripper(false)) // --> close gripper
				stage = 68+i*num_of_stages;
			if(stage == 68+i*num_of_stages && Controller::controllArm(arm_pose_sign_up)) // --> move arm up with the sign in hand
				stage = 69+i*num_of_stages;
			if(stage == 69+i*num_of_stages && Controller::turn(2.0, true)) // --> turn backward	(can't do false because of slippage on the map)
				stage = 70+i*num_of_stages;
			if(stage == 70+i*num_of_stages && Controller::goForward(-0.65, true))
				stage = 71+i*num_of_stages;
			if(stage == 71+i*num_of_stages && Controller::turn(1.0, false)) // --> turn left
				stage = 72+i*num_of_stages;
			if(stage == 72+i*num_of_stages && Controller::goForward(place_sign_y[i-1], false))
				stage = 73+i*num_of_stages;
			if(i!= 3){
				if(stage == 73+i*num_of_stages && Controller::turn(2.0, true)) // --> turn backward
					stage = 74+i*num_of_stages;
				if(stage == 74+i*num_of_stages && Controller::goForward(place_sign_x[i-1], true))
					stage = 75+i*num_of_stages;
			}
			else if(stage == 73+i*num_of_stages)
				stage = 75+i*num_of_stages;

			//put sign on the ground
			if(stage == 75+i*num_of_stages && Controller::controllArm(arm_pose_battery_in_car)) // --> move arm on the block
				stage = 76+i*num_of_stages;
			if(stage == 76+i*num_of_stages && Controller::controllGripper(true)) // --> open gripper
				stage = 77+i*num_of_stages;
			if(stage == 77+i*num_of_stages && Controller::controllArm(arm_pose_default)) // --> move arm to default position
				stage = 78+i*num_of_stages;
			// go back to signs
			if(i != 3){
				if(stage == 78+i*num_of_stages && Controller::turn(0.0, true)) // --> turn forward
					stage = 79+i*num_of_stages;
				if(stage == 79+i*num_of_stages && Controller::goForward(-0.7, true))
					stage = 80+i*num_of_stages;
			}
			else if(stage == 78+i*num_of_stages)
				stage = 80+i*num_of_stages;

			if(stage == 80+i*num_of_stages && Controller::turn(3.0, false)) // --> turn right
				stage = 81+i*num_of_stages;
			if(stage == 81+i*num_of_stages && Controller::goForward(0.29, false))
				stage = 82+i*num_of_stages;
			if(i != 3){
				if(stage == 82+i*num_of_stages && Controller::turn(0.0, true)) // --> turn forward
					stage = 83+i*num_of_stages;
				if(stage == 83+i*num_of_stages && Controller::goForward(return_to_signs_x[i-1], true))
					stage = 84+i*num_of_stages;
				if(stage == 84+i*num_of_stages && Controller::turn(3.0, false))
					stage = 85+i*num_of_stages;
			}

		}
		// CHECKPOINT x:-0.6    y:0.314



		// LIFTIONG GREEN BLOCKS ON THE TRUCK
		int num_of_blocks = 4;
		int block_counter = 0;
		float arm_pose_green_block_down[4] = {0.0, 0.86, -0.08, 0.0};
		float arm_pose_green_block_up[4] = {0.0, -0.90, 0.0, 0.0};
		float arm_pose_green_block_on_car[4] = {0.0, 1.2, -0.60, -0.37};

		if(stage == 154 && Controller::turn(2.0, true)) // --> turn backward
			stage = 155;
		if(stage == 155 && Controller::controllArm(arm_pose_back))	// --> true, even if takes longer, because if we turn the other way we slip and never stop turning because we miss the goal...so we do another circle and miss the goal again and so on
			stage = 156;
		if(stage == 156 && Controller::goForward(-0.88, true)) // --> move arm to back postition, so the robot also leans back so the sensors see more
			stage = 157;
		if(stage == 157 && Controller::allignWithObject(front_direction_index, range_min_index))	// --> allign with the object
			stage = 158;
		if(stage == 158 && Controller::aproachObjectMinDistance(range_min, 0.12))	// --> approach block			we use range_min because of bumpy terrain we can lose direction --> this is why we adjust it again	(msg->ranges[front_direction_index])
			stage = 159;
		if(stage == 159 && Controller::allignWithObject(front_direction_index, range_min_index))	// --> allign with the object again
			stage = 160;
		if(stage == 160 && Controller::controllGripper(true)) // --> open gripper
			stage = 161;
		if(stage == 161 && Controller::controllArm(arm_pose_green_block_down)) // --> move arm on the block
			stage = 162;
		if(stage == 162 && Controller::controllGripper(false)) // --> close gripper
			stage = 163;
		if(stage == 163 && Controller::controllArm(arm_pose_green_block_up)) // --> move arm on the block
			stage = 164;
		// CHECKPOINT: x:-0.89  y:0.31
		if(stage == 164 && Controller::turn(0.0, false)) // --> turn backward
			stage = 165;
		if(stage == 165 && Controller::goForward(0.24, true)) // --> move arm to back postition, so the robot also leans back so the sensors see more
			stage = 166;
		if(stage == 166 && Controller::turn(1.0, true))
			stage = 167;
		if(stage == 167 && Controller::goForward(0.65, false)) // --> move arm to back postition, so the robot also leans back so the sensors see more
			stage = 168;
		if(stage == 168 && Controller::turn(1.0, true))
			stage = 169;
		if(stage == 169 && Controller::controllArm(arm_pose_green_block_on_car)) // --> move arm on the block
			stage = 170;
		if(stage == 170 && Controller::controllGripper(true)) // --> open gripper
			stage = 171;
		if(stage == 171 && Controller::controllArm(arm_pose_green_block_up))
			stage = 172;
		if(stage == 172 && Controller::turn(3.0, true))
			stage = 173;
		if(stage == 173 && Controller::goForward(0.25, false)){
			block_counter++;
			if(block_counter == num_of_blocks)	// loop mechanism
				stage = 174;	// finish aka. exit condition
			else
				stage = 154;
		}

		//TODO :
		/*
		 * 4th car battery  Y
		 * go to signs	Y
		 * reposition for piciking up signs (new function)	Y
		 * remove stupid moving fucking car	Y
		 * arm pose names fix	Y
		 * pick up signs and put them around car spot	Y
		 * pick up blocks and put them on new car	Y
		 *
		 */




		// DEBUGGING
		if(stage == -1 && Controller::turn(3.0, true))	//--> turn right
			stage = -2;
		if(stage == -2 && Controller::turn(2.0, true))	//--> turn back
			stage = -3;
		if(stage == -3 && Controller::turn(1.0, true))	//--> turn left
			stage = -4;



	}

    void Controller::timerCallback(const ros::TimerEvent &e)	// for debugging
    {
    	Controller::controllGripper(true);
        ros::Duration(5).sleep();
    	Controller::controllGripper(false);
    }

    bool Controller::allignWithObject(int front_direction_index, int range_min_index){
    	// +-40 is not ideal, but because robot often gets trapped in a bump it is unfortunate necessary measure
    	// this could be solved by increasing angular.z, but we can't because corrections would be too big
    	if(front_direction_index-40 <= range_min_index &&  range_min_index <= front_direction_index+40){
			ROS_INFO("Finished alligning            stage: %i", stage);
			cmd_val_values_.angular.z = 0.0;
    		cmd_vel_publisher_.publish(cmd_val_values_);
			ros::Duration(1.0).sleep();
    		return true;
    	}
    	else{
    		if(front_direction_index > range_min_index){
    			cmd_val_values_.angular.z = -0.1;
    			ROS_INFO_STREAM_THROTTLE(2.0, "Right");

    		}
    		else{
    			cmd_val_values_.angular.z = 0.1;
    			ROS_INFO_STREAM_THROTTLE(2.0, "Left");
    		}

    		cmd_vel_publisher_.publish(cmd_val_values_);
    		return false;
    	}


    }

    bool Controller::aproachObjectMinDistance(float front_dir_sen_length, float goal_distance){
    	if(front_dir_sen_length <= goal_distance ){
			ROS_INFO("Finished approaching            stage: %i", stage);
			cmd_val_values_.linear.x = 0.0;
    		cmd_vel_publisher_.publish(cmd_val_values_);
			ros::Duration(1.0).sleep();
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
			ros::Duration(2.0).sleep();		// more sleep because sometimes it continues sliding even after we stop it....so we let it slide a bit
			return true;
		}
		cmd_vel_publisher_.publish(cmd_val_values_);
		return false;
	}



    bool Controller::turn(double direction, bool leftRight)		// leftRight defines the direction of turning ---> left=True     right=False
    { // direction = 0 --> forward; direction 1 --> left; direction 2 --> back; direction 3 --> right
        geometry_msgs::Twist cmd_val_values;
        double multiplier = 1.0;
        if(not leftRight)
            multiplier = -1.0;

        double base_turn_yaw = 1.571;	//3.142	// maybe 1.56424
        double target_yaw = direction * base_turn_yaw;
        if(direction == 2.0)
        	target_yaw = 2 * base_turn_yaw;  // 3.13;
        if(direction == 3.0)
        	target_yaw =  -1.0 * base_turn_yaw;



		if(not rotating){
			ROS_INFO("Turning in direction %f        target yaw: %f", direction, target_yaw);
			cmd_val_values.angular.z = multiplier * 0.5;
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
				ros::Duration(1.0).sleep();
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
        ros::Duration(2.3).sleep();
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
        ros::Duration(1.5).sleep();
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


/*
 * realigned sensor and its size
 * fucking arm
 * fucking bot
 *
 *
 */

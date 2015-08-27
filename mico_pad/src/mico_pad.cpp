/*
 * mico_pad
 * Copyright (c) 2015, Robotnik Automation, SLL
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Robotnik Automation, SLL. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \author Robotnik Automation SLL
 * \brief Allows to use a PS3 or other pad with the jaco_driver mico node
 */

#include <ros/ros.h>
#include <jaco_msgs/JointVelocity.h>
#include <geometry_msgs/TwistStamped.h>
#include <jaco_msgs/HomeArm.h>

#include <sensor_msgs/Joy.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>

#include <actionlib/client/simple_action_client.h>
#include <jaco_msgs/SetFingersPositionAction.h>
#include <jaco_msgs/SetFingersPositionActionGoal.h>
#include <jaco_msgs/FingerPosition.h>

#define DEFAULT_NUM_OF_BUTTONS		20
//#define ARM							0
//#define GRIPPER						1

#define DEFAULT_AXIS_LINEAR_X		1
#define DEFAULT_AXIS_LINEAR_Y       0
#define DEFAULT_AXIS_LINEAR_Z		3
#define DEFAULT_AXIS_ANGULAR_X		1
#define DEFAULT_AXIS_ANGULAR_Y      0
#define DEFAULT_AXIS_ANGULAR_Z		2

#define DEFAULT_SCALE_LINEAR		1.0
#define DEFAULT_SCALE_ANGULAR		1.0

typedef actionlib::SimpleActionClient<jaco_msgs::SetFingersPositionAction> MicoActionClient;

class MicoPad
{
	public:
	MicoPad();

	private:
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	void fingerCallback(const jaco_msgs::FingerPosition::ConstPtr& finger_position);

	ros::NodeHandle nh_;

	//! It will publish into command velocity (for the robot)
	ros::Publisher arm_ref_joint_pub_, arm_ref_cartesian_pub_, gripper_ref_pub_;
	
	//! It will be suscribed to the joystick
	ros::Subscriber joy_sub_;

	//! Number of the button actions
	int button_up_, button_down_;
	int button_select_;
	int button_fold_;
    int button_open_, button_close_;
    int button_euler_;
	//! Number of buttons of the joystick
	int num_of_buttons_;
	//! Pointer to a vector for controlling the event when pushing the buttons
	bool bRegisteredButtonEvent[DEFAULT_NUM_OF_BUTTONS];
	//! buttons to the arm
	int dead_man_arm_;
	//! current connection (0: arm, 1: gripper)
	short deviceConnected;
	//! Service to move the gripper/arm
	ros::ServiceClient gripper_setOperationMode_client;
	ros::ServiceClient gripper_move_client;
	ros::ServiceClient gripper_move_incr_client;
	ros::ServiceClient gripper_close_client;
	
	ros::ServiceClient arm_fold_client;

	int linear_x_, linear_y_, linear_z_, angular_x_, angular_y_, angular_z_;
	double l_scale_, a_scale_;
	std::string topic_joy;
	
	MicoActionClient ac_;	
	
	ros::Subscriber finger_sub_;
	jaco_msgs::FingerPosition finger_pos_;
};


MicoPad::MicoPad() : ac_("/mico_arm_driver/fingers/finger_positions", true) {


	nh_.param("num_of_buttons", num_of_buttons_, DEFAULT_NUM_OF_BUTTONS);

	//GENERIC BUTTONS
	nh_.param("button_up", button_up_, button_up_);  				// Triangle PS3
	nh_.param("button_down", button_down_, button_down_); 	    	// Cross PS3
	nh_.param("button_select", button_select_, button_select_);		// Select PS3
	nh_.param("button_fold", button_fold_, button_fold_);			// Start PS3
	nh_.param("button_open", button_open_, button_open_);			// Circle PS3
	nh_.param("button_close", button_close_, button_close_);	    // Square PS3
	nh_.param("button_euler", button_euler_, button_euler_);	    // Square PS3
	
	nh_.param<std::string>("mico_joy", topic_joy, "joy");	    
	
	ROS_INFO("MicoPad: joy_topic = %s", topic_joy.c_str());

	// ARM CONF
	nh_.param("dead_man_arm", dead_man_arm_, dead_man_arm_);					// R2 PS3

    // JOY AXIS DEFINITION AND SCALING 
	nh_.param("axis_linear_x", linear_x_, DEFAULT_AXIS_LINEAR_X);
    nh_.param("axis_linear_y", linear_y_, DEFAULT_AXIS_LINEAR_Y);
    nh_.param("axis_linear_z", linear_z_, DEFAULT_AXIS_LINEAR_Z);

	nh_.param("axis_angular_x", angular_x_, DEFAULT_AXIS_ANGULAR_X);
    nh_.param("axis_angular_y", angular_y_, DEFAULT_AXIS_ANGULAR_Y);
    nh_.param("axis_angular_z", angular_z_, DEFAULT_AXIS_ANGULAR_Z);

	nh_.param("scale_angular", a_scale_, DEFAULT_SCALE_ANGULAR);
	nh_.param("scale_linear", l_scale_, DEFAULT_SCALE_LINEAR);

	//bRegisteredButtonEvent = new bool(num_of_buttons_);
	for(int i = 0; i < DEFAULT_NUM_OF_BUTTONS; i++){
		bRegisteredButtonEvent[i] = false;
	}

 	 // Listen through the node handle sensor_msgs::Joy messages from joystick
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &MicoPad::joyCallback, this);
	
	finger_sub_ = nh_.subscribe<jaco_msgs::FingerPosition>("/mico_arm_driver/out/finger_position", 10, &MicoPad::fingerCallback, this);

	// Request service to send commands to the arm
	arm_fold_client = nh_.serviceClient<jaco_msgs::HomeArm>("/mico_arm_driver/in/home_arm");

	// Publishes into the arm controller
	arm_ref_joint_pub_ = nh_.advertise<jaco_msgs::JointVelocity>("/mico_arm_driver/in/joint_velocity", 1);
	arm_ref_cartesian_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/mico_arm_driver/in/cartesian_velocity", 1);
	  
    // Publishes into the arm controller
	gripper_ref_pub_ = nh_.advertise<std_msgs::Int32>("/terabot/gripper", 1);
		
    ac_.waitForServer();
    
    finger_pos_.finger1 = 0.0;
    finger_pos_.finger2 = 0.0;
    finger_pos_.finger3 = 0.0;
   	
}

void MicoPad::fingerCallback(const jaco_msgs::FingerPosition::ConstPtr& finger_position)
{
	// Just store current positions in global structure
	finger_pos_.finger1 = finger_position->finger1;
	finger_pos_.finger2 = finger_position->finger2;
	finger_pos_.finger3 = finger_position->finger3;	
	
	// ROS_INFO("pos %5.2f %5.2f %5.2f", finger_pos_.finger1, finger_pos_.finger2, finger_pos_.finger3);
}

void MicoPad::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

	jaco_msgs::JointVelocity arm;
	// geometry_msgs::Twist ref;	
	geometry_msgs::TwistStamped ref;
	
	ref.twist.linear.x = 0.0;
	ref.twist.linear.y = 0.0;
	ref.twist.linear.z = 0.0;
	
	ref.twist.angular.x = 0.0;
	ref.twist.angular.y = 0.0;
	ref.twist.angular.z = 0.0;


	int32_t gripper_ref = 0;
	bool gripper_event = false;
	
	// ARM MOVEMENTS
	// Actions dependant on arm dead-man button
	if (joy->buttons[dead_man_arm_] == 1){
		
		if (joy->buttons[button_up_] == 1){
			if(!bRegisteredButtonEvent[button_up_]){
				bRegisteredButtonEvent[button_up_] = true;
									
			}
		}else if (joy->buttons[button_down_] == 1){
			if(!bRegisteredButtonEvent[button_down_]){
				bRegisteredButtonEvent[button_down_] = true;
				
			}
		
		// Used to change between different robot operation modes (jbj, cartesian-euler and trajectory)
		}else if (joy->buttons[button_select_] == 1){
			if(!bRegisteredButtonEvent[button_select_]){
				bRegisteredButtonEvent[button_select_] = true;				
			}
					
		// Allow to fold the arm throught the pad	
		}else if (joy->buttons[button_fold_] == 1){
			if(!bRegisteredButtonEvent[button_fold_]){
				bRegisteredButtonEvent[button_fold_] = true;
				
				jaco_msgs::HomeArm srv;				
				arm_fold_client.call(srv);		
			}
		}else{
			
			ref.header.stamp = ros::Time::now();
			ref.header.frame_id = "mico_link_base";
						
			if (joy->buttons[button_euler_] == 0) {
				// Arm linear references
				ref.twist.linear.x = l_scale_ * joy->axes[linear_x_];
				ref.twist.linear.y = l_scale_ * joy->axes[linear_y_];
				ref.twist.linear.z = l_scale_ * joy->axes[linear_z_];
				}
			else {				
				ref.twist.angular.x = a_scale_ * joy->axes[angular_x_];					
				ref.twist.angular.y = -a_scale_ * joy->axes[angular_y_];					
				ref.twist.angular.z = -a_scale_ * joy->axes[angular_z_];					
				}
			
            // Gripper speed reference
            // if (joy->axes[button_close_]!=0.0) {                                              
			if (joy->buttons[button_close_] == 1) {                                              
                 gripper_ref = (int) -(joy->axes[button_close_] * 100.0);
                 gripper_event = true;
                 
                 // Close
				 jaco_msgs::SetFingersPositionGoal goal;
				 goal.fingers.finger1 = finger_pos_.finger1 + 2000.0;
				 if (goal.fingers.finger1 > 7000) goal.fingers.finger1 = 7000;
				 goal.fingers.finger2 = finger_pos_.finger2 + 2000.0;
				 if (goal.fingers.finger2 > 7000) goal.fingers.finger2 = 7000;
				 ac_.sendGoal(goal);
				 ac_.waitForResult(ros::Duration(0.0));
                 
            //}else if (joy->axes[button_open_]!=0.0){
				}else if (joy->buttons[button_open_] == 1){
                 gripper_ref = (int) (joy->axes[button_open_] * 100.0);
                 gripper_event = true;
                 
				 // Open
				 jaco_msgs::SetFingersPositionGoal goal;
				 goal.fingers.finger1 = finger_pos_.finger1 - 2000.0;
				 if (goal.fingers.finger1 < 0) goal.fingers.finger1 = 0;
				 goal.fingers.finger2 = finger_pos_.finger2 - 2000.0;
				 if (goal.fingers.finger2 < 0) goal.fingers.finger2 = 0;				 			
				 ac_.sendGoal(goal);
				 ac_.waitForResult(ros::Duration(0.0));                 
			 }
						
            // ROS_INFO("vx=%5.2f, vy=%5.2f, vz=%5.2f", ref.twist.linear.x, ref.twist.linear.y, ref.twist.linear.z);
            // ROS_INFO("ax=%5.2f, ay=%5.2f, az=%5.2f", ref.twist.angular.x, ref.twist.angular.y, ref.twist.angular.z);
						
			bRegisteredButtonEvent[button_up_] = false;
			bRegisteredButtonEvent[button_down_] = false;
			bRegisteredButtonEvent[button_select_] = false;
			bRegisteredButtonEvent[button_fold_] = false;
			
			arm_ref_cartesian_pub_.publish(ref);	
		}
	}
	
	// Publish gripper msg
	if(gripper_event){
		std_msgs::Int32 msg;
		msg.data = gripper_ref; 
		gripper_ref_pub_.publish(msg);									
	}
	
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "mico_pad");
	MicoPad mico_pad;
	ros::spin();
}


//#include <stdio.h>
//#include <stdlib.h>
//#include <vector>
//#include <iostream>
//#include <math.h>
#include <cmath>
#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>

//[W]need to scaling max_velwheel_vel
#define pwm_threshold  220
#define vel_threshold  0.7
#define switch_threshold 1000

//[W]For ROS =======================================
std_msgs::Int16MultiArray rc_input;
std_msgs::Float32MultiArray motor_vel;
std_msgs::Int16MultiArray motor_output;
std_msgs::Float32MultiArray checkarr;
std_msgs::Float32MultiArray sys_vel_arr;
std_msgs::Float32MultiArray motor_vel2;
std_msgs::Float32MultiArray des_R;
//====================================================

//[W]function header========================================
void set_array();
void PWMsCallback(const std_msgs::Int16MultiArray::ConstPtr& rc_sub);
void velocityCallback(const std_msgs::Float32MultiArray::ConstPtr& moniter_vel);
int sgn(double v);
//====================================================

int main(int argc, char** argv) 
{
	set_array();
	ros::init(argc, argv, "rc_pwm_sbus");
	ros::NodeHandle nh;
	
	ros::Publisher pwm_to_vel = nh.advertise<std_msgs::Int16MultiArray>("pwm_to_vel", 1000);
	ros::Publisher check = nh.advertise<std_msgs::Float32MultiArray>("check", 1000);
	ros::Publisher sys_vel = nh.advertise<std_msgs::Float32MultiArray>("sys_vel", 1000);
	ros::Publisher pulse_to_vel2 = nh.advertise<std_msgs::Float32MultiArray>("wheel_vel2", 1000);
	ros::Publisher des_rad = nh.advertise<std_msgs::Float32MultiArray>("des_rad", 1000);
	ros::Subscriber rc_to_pwm = nh.subscribe("RC_readings", 1000, &PWMsCallback);
	ros::Subscriber pulse_to_vel = nh.subscribe("wheel_vel", 1000, &velocityCallback);
	
	// [W] change from 10 to 100 then set baudrate
	ros::Rate loop_rate(100);
	
	while (ros::ok()) 
	{
		pwm_to_vel.publish(motor_output);
		check.publish(checkarr);
		sys_vel.publish(sys_vel_arr);
		pulse_to_vel2.publish(motor_vel2);
		des_rad.publish(des_R);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

//	rc_input.data[0] : Ch1 : left-right
//	rc_input.data[1] : Ch2 : forward-backward
//	rc_input.data[2] : Ch4 : heading angle
//	rc_input.data[3] : Ch5 : connect or disconnect

void PWMsCallback(const std_msgs::Int16MultiArray::ConstPtr& rc_sub)
{
	for (int i = 0; i < 4; i++) //[J]there might be another way not to use for.
	{
		rc_input.data[i] = rc_sub->data[i];	

	}
	//[W] each channels's values
	ROS_INFO(" 1:[%d] 2:[%d] 3:[%d] 4:[%d]", rc_input.data[0], rc_input.data[1], rc_input.data[2], rc_input.data[3]);
	

		
	if (rc_input.data[3] < switch_threshold)
	{
		//rc_input.data[0] : x
		//rc_input.data[1] : y
		//rc_input.data[2] : w
		motor_output.data[0] = (rc_input.data[0] + rc_input.data[1] + rc_input.data[2]) / 3;
		motor_output.data[1] = (-rc_input.data[0] + rc_input.data[1] - rc_input.data[2]) / 3;
		motor_output.data[2] = (-rc_input.data[0] + rc_input.data[1] + rc_input.data[2]) / 3;
		motor_output.data[3] = (rc_input.data[0] + rc_input.data[1] - rc_input.data[2]) / 3;

	}

	if (rc_input.data[3] > switch_threshold)
	{	
		//[W] to increase resolving power change int to float			
		// rc_input.data[2] : R
		// rc_input.data[1] : v_m
		float new_rc = rc_input.data[2];		
		float R = 1 / (tan(new_rc / 255 * M_PI / 2));	
		//[J] If response characteristics of R via tangent function-based mapping is not satisfying, then
		//1. Change the weight (current:1 , changed:10,100,.1,.01 etc..)
		//2. Change the mapping function that is not tangent
		float R_minimum = 0.1;	
		if (abs(R) < R_minimum) R = sgn(R) * R_minimum;
		//[W] modified
		des_R.data[0] = 1/R;
			
		float v2 = rc_input.data[1];
		float L = 0.43;
		float theta = atan(L/R);
		float vx = v2 * sin(theta);
		float vy = v2 * cos(theta);
		float w = vy / R;
		if(abs(w) > 255) w=sgn(w)*255; 	

		//[W] just check 		
		checkarr.data[0]=R;
		checkarr.data[1]=theta;		
		checkarr.data[2]=vx;
		checkarr.data[3]=vy;
		checkarr.data[4]=w;	

		motor_output.data[0] = (vx + vy + w) / 3; 
		motor_output.data[1] = (-vx + vy - w) / 3;
		motor_output.data[2] = (-vx + vy + w) / 3;
		motor_output.data[3] = (vx + vy - w) / 3;

	
	}
	for(int i=0; i<4; i++)
	{
		if(motor_output.data[i] < pwm_threshold)
			sys_vel_arr.data[i] = motor_output.data[i]*vel_threshold/pwm_threshold;
		else
			sys_vel_arr.data[i] = vel_threshold;
	}
}



void velocityCallback(const std_msgs::Float32MultiArray::ConstPtr& moniter_vel) 
{
	
	for(int i=0;i<4;i++)
	{
		motor_vel.data[i]= moniter_vel->data[i];
		motor_vel2.data[i]= moniter_vel->data[i];
	}
}

void set_array()
{
	rc_input.data.resize(4);
	motor_vel.data.resize(4);
	motor_output.data.resize(4);
	checkarr.data.resize(5);
	sys_vel_arr.data.resize(4);
	motor_vel2.data.resize(4);
	des_R.data.resize(1);
}


int sgn(double v)
{
	if (v < 0) return -1;
	if (v >= 0) return 1;
}



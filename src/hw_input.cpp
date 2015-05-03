//motor_input.cpp
//Combines the controller input and joystick input
//Author: Balakumar Sundaralingam

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Joy.h>
#include <math.h>
#define PI 3.14159265

float thresh_vectors=1;
float key=0;
float joy_gain=100;
float factor=10;
float k_high=15,k_low=5;
float chosen=180;
float fl_e=0,fr_e=0,bl_e=0,br_e=0;
int swtch=0;
int fw_swtch=0;
float joy_x_=0,joy_y_=0,joy_yaw_=0,joy_x=0,joy_y=0,joy_yaw=0;
float left_obs=0,right_obs=0;
ros::Publisher fl_pub,fr_pub,bl_pub,br_pub;
int on=0;
void joyCallback(const sensor_msgs::Joy&);
void control_function();
void leftCallback(const std_msgs::Float32&);
void rightCallback(const std_msgs::Float32&);



int main(int argc,char** argv)
{
	ros::init(argc,argv,"motor_input");
	ros::NodeHandle n;
	ros::Rate loop_rate(50);
	ros::Subscriber left_sub=n.subscribe("/optic/left",1,leftCallback);
	ros::Subscriber right_sub=n.subscribe("/optic/right",1,rightCallback);

	ros::Subscriber joy_sub=n.subscribe("joy",1,joyCallback);

	
	fl_pub=n.advertise<std_msgs::Float32>("holobot/fl",100);
	fr_pub=n.advertise<std_msgs::Float32>("holobot/fr",100);
	bl_pub=n.advertise<std_msgs::Float32>("holobot/bl",100);
	br_pub=n.advertise<std_msgs::Float32>("holobot/br",100);
	
	while(ros::ok())
	{
	control_function();
	
	ros::spinOnce();
	loop_rate.sleep();
	}
	return(0);
}
void leftCallback(const std_msgs::Float32& left_)
{
	left_obs=left_.data;
}

void rightCallback(const std_msgs::Float32& right_)
{
	right_obs=right_.data;
}
void joyCallback(const sensor_msgs::Joy& joy_msg_in)
{
	joy_x_=factor*(1+joy_msg_in.axes[1]);//left stick up-down
	joy_y_=factor*(1+joy_msg_in.axes[0]);//left stick left-right
	joy_yaw_=factor*(1+joy_msg_in.axes[2]);//right stick left-right
	
	joy_x=joy_gain*joy_msg_in.axes[1];//left stick up-down
	joy_y=joy_gain*joy_msg_in.axes[0];//left stick left-right
	joy_yaw=joy_gain*joy_msg_in.axes[2];//right stick left-right
	
	if(joy_msg_in.buttons[0]==1)
	{
		ROS_INFO("pressed A");
		swtch=1;
		fw_swtch=0;

	}
	if(joy_msg_in.buttons[1]==1)
	{
		ROS_INFO("pressed B");
		swtch=0;
		fw_swtch=0;
		on=1;
	}
	if(joy_msg_in.buttons[2]==1)
	{
		ROS_INFO("motor off");
		swtch=0;
		fw_swtch=0;
		on=0;
	}

	
}

void control_function()
{
	float velocity=100;

	int theta=0;
	float x_=0,y_=0;
	theta=atan2(joy_y,joy_x)*180/PI;
	std_msgs::Float32 dir_;
	float forw_back=joy_x;
	float left_right=joy_y;
	float cw_ccw=joy_yaw;
	std_msgs::Float32 fl,fr,bl,br;
	float x,y,z;
	x=abs(joy_x);
	y=abs(joy_y);
	z=abs(joy_yaw);
	int dead=0;

	if(abs(joy_x)<25)
	{
		joy_x=0;
	}
	if(abs(joy_y)<25)
	{
		joy_y=0;
	}
	if(abs(joy_yaw)<25)
	{

		joy_yaw=0;
	}	
	if((swtch==1))
	{
	if(left_obs<thresh_vectors&&right_obs<thresh_vectors)
	{
		chosen=180*PI/180;
	}
	else if(left_obs<thresh_vectors)
	{
		chosen=270*PI/180;	
	}
	else if(right_obs<thresh_vectors)
	{
		chosen=90*PI/180;	
	}
	else 
	{
		velocity=0;
	}
	


	x_=-cos(chosen);
	y_=-sin(chosen);

	fl.data=velocity*x_-velocity*y_-joy_yaw;
	fr.data=velocity*x_+velocity*y_+joy_yaw;
	bl.data=velocity*x_+velocity*y_-joy_yaw;
	br.data=velocity*x_-velocity*y_+joy_yaw;
	}	
	else if(on==1)
	{
	fl.data=joy_x-joy_y-joy_yaw;
	fr.data=joy_x+joy_y+joy_yaw;
	bl.data=joy_x+joy_y-joy_yaw;
	br.data=joy_x-joy_y+joy_yaw;
	}
	fl_pub.publish(fl);
	fr_pub.publish(fr);
	bl_pub.publish(bl);
	br_pub.publish(br);

}


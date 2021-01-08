//
// Created by lihao on 19-7-12.
//

#include <termios.h>
#include <signal.h>
#include <fcntl.h>
#include <stdio.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

using namespace std;

//------------------------------- Define -------------------------------//
#define MOVE_FORWARD 	'w'
#define MOVE_BACKWARD 	's'
#define MOVE_LEFT 	'a'
#define MOVE_RIGHT 	'd' 
#define MOVE_UP		'i'
#define MOVE_DOWN	'k'
#define TURN_LEFT	'j'
#define TURN_RIGHT	'l'


//------------------------------- Global variable -------------------------------//
int kfd = 0;
struct termios cooked, raw;
ros::Publisher vel_pub;
vector<double> KP;
int rate;

//------------------------------- Callback function -------------------------------//
void quit(int sig)
{
	(void)sig;
	tcsetattr(kfd, TCSANOW, &cooked);
	ros::shutdown();
}

void TimerCallback(const ros::TimerEvent&)
{
	// Get current key value and clear the buffer
	char ch, keyvalue = 0;
    while(read(kfd, &ch, 1) > 0){
		keyvalue = ch;
	}
	// ROS_INFO("value: %c\n", keyvalue);

    geometry_msgs::Twist cmd_vel = geometry_msgs::Twist();
	switch(keyvalue)
	{
	case MOVE_FORWARD:
		ROS_INFO("MOVE FORWARD");
		cmd_vel.linear.x = 1.0;
		break;
	case MOVE_BACKWARD:
		ROS_INFO("MOVE BACKWARD");
		cmd_vel.linear.x = -1.0;
		break;
	case MOVE_LEFT:
		ROS_INFO("MOVE LEFT");
		cmd_vel.linear.y = 1.0;
		break;
	case MOVE_RIGHT:
		ROS_INFO("MOVE RIGHT");
		cmd_vel.linear.y = -1.0;
		break;
	case MOVE_UP:
		ROS_INFO("UP");
		cmd_vel.linear.z = 1.0;
		break;
	case MOVE_DOWN:
		ROS_INFO("DOWN");
		cmd_vel.linear.z = -1.0;
		break;
	case TURN_LEFT:
		ROS_INFO("TURN LEFT");
		cmd_vel.angular.z = 1.0;
		break;
	case TURN_RIGHT:
		ROS_INFO("TURN RIGHT");
		cmd_vel.angular.z = -1.0;
		break;
	}
   
	// Publish Twist message
	cmd_vel.linear.x *= KP[0];
	cmd_vel.linear.y *= KP[1];
	cmd_vel.linear.z *= KP[2];
	cmd_vel.angular.z *= KP[3];

	vel_pub.publish(cmd_vel);
}

//------------------------------- Main function -------------------------------//
int main(int argc, char **argv)
{
	// Initial node
	ros::init(argc, argv, "keyboardcontrol");
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");
	ROS_INFO("Setup keyboardcontrol node!");

	// Parameter
	nh_priv.getParam("KP", KP);
    nh_priv.param<int>("rate", rate, 10);

	// Usage information
	puts("Reading from keyboard");
	puts("---------------------------");
	puts("Use 'a', 'd', 'w', 's', 'i', 'k' keys to move, 'j' and 'l' keys to turn");

	// Get the console in raw mode                                                              
	tcgetattr(kfd, &cooked);
	memcpy(&raw, &cooked, sizeof(struct termios));
	raw.c_lflag &=~ (ICANON | ECHO);                      
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	tcsetattr(kfd, TCSANOW, &raw);

	// Set read file in non-blocking mode
	int flags = fcntl(kfd, F_GETFL, 0); 
	flags |= O_NONBLOCK;         
	fcntl(kfd, F_SETFL, flags);       

	// Publish Twist message
	vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

	// Create a timer
	ros::Timer timer = nh.createTimer(ros::Duration(1.0 / rate), &TimerCallback);

	// Wait for ctrl'C to quit
	signal(SIGINT,quit);

	// Loop and execute callback function
	ros::spin();

	return 0;
}

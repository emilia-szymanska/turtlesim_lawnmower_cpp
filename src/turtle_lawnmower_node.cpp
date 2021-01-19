#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "turtlesim/TeleportAbsolute.h"
#include "std_srvs/Empty.h"
#include <iostream>

class TurtleLawnmower
{
	ros::NodeHandle nh_;
	ros::Subscriber pose_sub_;
 	ros::Publisher  cmd_pub_;
	
	bool rotate = false;
	int direction = 1;	
	float theta_goal = 3.13;

 	public:
  		TurtleLawnmower();
  		~TurtleLawnmower(){};
  		void turtleCallback(const turtlesim::Pose::ConstPtr& msg);
};


TurtleLawnmower::TurtleLawnmower()
{
	pose_sub_ = nh_.subscribe("turtle1/pose", 1, &TurtleLawnmower::turtleCallback, this);
	cmd_pub_  = nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);

	ros::ServiceClient teleport_client = nh_.serviceClient<turtlesim::TeleportAbsolute>("/turtle1/teleport_absolute");
	turtlesim::TeleportAbsolute::Request req;
	turtlesim::TeleportAbsolute::Response resp;
	req.x 	  = 1;
	req.y 	  = 0.5;
	req.theta = 0;
	teleport_client.call(req, resp);
	
	ros::ServiceClient clear_client = nh_.serviceClient<std_srvs::Empty>("/clear");
  	std_srvs::Empty srv;
  	clear_client.call(srv);
}


void TurtleLawnmower::turtleCallback(const turtlesim::Pose::ConstPtr& msg)
{
	ROS_INFO("Turtle lawnmower@ [%f, %f, %f]", msg->x, msg->y, msg->theta);
	
	if(msg->y >= 10.5 && (msg->x >= 10 || msg->x <= 1))
	{
		ROS_INFO("Turtle has finished its job! Shutting down the node...");
		ros::shutdown();
	}

	geometry_msgs::Twist turtle_cmd_vel;
  	
	
	if(!rotate)
	{
  		turtle_cmd_vel.linear.x = 1;
		// near the edge -> start rotating
		if((direction == 1 && msg->x >= 10) || (direction == -1 && msg->x <= 1))	
		{
			rotate = true;
			direction = -direction;
			if(direction == -1) theta_goal = 3.13;
			else theta_goal = 0.01;
		}
	}
	else
	{
		// don't stop spinning if theta is not close to 0 or PI
		if((theta_goal > msg->theta && direction == -1) || (theta_goal < msg->theta && direction == 1))
		{
  			turtle_cmd_vel.linear.x = 0.3;
  			turtle_cmd_vel.angular.z = -direction*0.9;
		}
		else 	// stop spinning 
		{
  			turtle_cmd_vel.linear.x = 1;
  			turtle_cmd_vel.angular.z = 0;
			rotate = false;
		}
	}
  	
	cmd_pub_.publish(turtle_cmd_vel);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "turtle_lawnmower_node");
	TurtleLawnmower turle_mower;
	ros::spin();

	return 0;
}

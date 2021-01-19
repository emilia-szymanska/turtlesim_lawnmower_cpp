#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "turtlesim/TeleportAbsolute.h"
#include "std_srvs/Empty.h"
#include <iostream>

class TurtleLawnmower
{
	ros::NodeHandle nh_;
	ros::Subscriber sub_;
 	ros::Publisher  pub_;

 	public:
  		TurtleLawnmower();
  		~TurtleLawnmower(){};
  		void turtleCallback(const turtlesim::Pose::ConstPtr& msg);
};


TurtleLawnmower::TurtleLawnmower()
{
	sub_ = nh_.subscribe("turtle1/pose", 1, &TurtleLawnmower::turtleCallback, this);
	pub_ = nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
	//ros::service::waitForService("clear");  //this is optional

	ros::ServiceClient client = nh_.serviceClient<turtlesim::TeleportAbsolute>("/turtle1/teleport_absolute");
	turtlesim::TeleportAbsolute::Request req;
	turtlesim::TeleportAbsolute::Response resp;

	req.x = 1;
	req.y = 1;
	req.theta = 0;
	
	bool success = client.call(req, resp);
  	
	std::cout << "Success: " << success << std::endl;
	
	ros::ServiceClient clearClient = nh_.serviceClient<std_srvs::Empty>("/clear");
  	std_srvs::Empty srv;
  	clearClient.call(srv);
}


void TurtleLawnmower::turtleCallback(const turtlesim::Pose::ConstPtr& msg)
{
	ROS_INFO("Turtle lawnmower@ [%f, %f, %f]", msg->x, msg->y, msg->theta);

  	geometry_msgs::Twist turtle_cmd_vel;
  	turtle_cmd_vel.linear.x = 1;
  	pub_.publish(turtle_cmd_vel);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "turtle_lawnmower_node");
  TurtleLawnmower turle_mower;
  ros::spin();

  return 0;
}

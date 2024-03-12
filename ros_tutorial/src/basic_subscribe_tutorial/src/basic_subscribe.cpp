#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

void CmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
	ROS_INFO("subscribe cmd_vel: linear.x = %.3f, angular.z = %.3f", cmd_vel->linear.x, cmd_vel->angular.z);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "basic_subscribe_node");
	ros::NodeHandle n;

	ros::Subscriber subscriber_cmd_vel;
	subscriber_cmd_vel = n.subscribe("cmd_vel", 1000, CmdVelCallback);

	ros::Rate loop_rate(60);

	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	return (0);
}

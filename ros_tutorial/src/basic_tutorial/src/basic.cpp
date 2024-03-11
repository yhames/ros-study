#include <ros/ros.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "basic_node");
	ros::NodeHandle n;

	ros::Rate loop_rate(60);

	while (ros::ok())
	{
		ROS_INFO("Hello World!");
		ros::spinOnce();
		loop_rate.sleep();
	}
	return (0);
}

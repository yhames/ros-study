#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "basic_publish_node");
	ros::NodeHandle n;

	ros::Rate loop_rate(60);

	geometry_msgs::Twist cmd_vel;	// geometry_msgs::Twists 타입의 cmd_vel 변수를 정의한다.
	ros::Publisher publisher_cmd_vel = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);	// advertise(msg_name, msg_queue_size)를 사용하여 Publisher를 정의한다.

	while (ros::ok())
	{
		/**
		 * geometry_msgs/Twist.msg
		 * @param Vector3 linear
		 * @param Vector3 angular
		 *
		 * geometry_msgs/Vector3
		 * @param float64 x, y, z
		 */
		cmd_vel.linear.x = 10;
		cmd_vel.angular.z = 10;
		publisher_cmd_vel.publish(cmd_vel);	// publish(msg)를 사용하여 발행할 수 있다.
		ros::spinOnce();
		loop_rate.sleep();
	}
	return (0);
}

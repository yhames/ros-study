# 03 - ROS Subscriber

> 목표 : `geometry_msgs/Twist` topic을 수신하는 Subscriber 노드를 생성한다.

## geometry_msgs 패키지 추가

catkin_create_pkg 명령어를 사용할 때 의존성을 명시해주면 자동으로 의존성이 등록됩니다.

```bash
catkin_create_pkg basic_subscribe_tutorial roscpp geometry_msgs
```

## C++ 코드 작성

basic_subscribe_tutorial 패키지의 src 폴더에 다음과 같이 basic_subscribe.cpp 코드를 추가합니다.



```c++
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
```

### Callback 함수와 spinOnce()

subscribe() 함수의 세번째 인자는 토픽을 수신했을 때 호출되는 함수입니다.

반환값은 void 타입이고, 매개변수는 수신하는 토픽의 메시지 타입의 스마트포인터(ConstPtr) 래퍼런스 타입으로 정의합니다.

ros::spinOnce()가 실행되면 토픽이 수신되고 Callback 함수가 실행됩니다.

## 결과

roscore와 basic_publish_node를 모두 실행한 다음 basic_subscribe_node를 실행하면 다음과 같이 출력됩니다.

```bash
[ INFO] [1710214708.649020633]: subscribe cmd_vel: linear.x = 10.000, angular.z = 10.000
[ INFO] [1710214708.666160631]: subscribe cmd_vel: linear.x = 10.000, angular.z = 10.000
[ INFO] [1710214708.683068667]: subscribe cmd_vel: linear.x = 10.000, angular.z = 10.000
[ INFO] [1710214708.699075391]: subscribe cmd_vel: linear.x = 10.000, angular.z = 10.000
[ INFO] [1710214708.715471707]: subscribe cmd_vel: linear.x = 10.000, angular.z = 10.000
[ INFO] [1710214708.733052382]: subscribe cmd_vel: linear.x = 10.000, angular.z = 10.000
[ INFO] [1710214708.748808475]: subscribe cmd_vel: linear.x = 10.000, angular.z = 10.000
[ INFO] [1710214708.766166520]: subscribe cmd_vel: linear.x = 10.000, angular.z = 10.000
[ INFO] [1710214708.783151432]: subscribe cmd_vel: linear.x = 10.000, angular.z = 10.000
[ INFO] [1710214708.798870692]: subscribe cmd_vel: linear.x = 10.000, angular.z = 10.000
[ INFO] [1710214708.815381469]: subscribe cmd_vel: linear.x = 10.000, angular.z = 10.000
[ INFO] [1710214708.833041812]: subscribe cmd_vel: linear.x = 10.000, angular.z = 10.000
```

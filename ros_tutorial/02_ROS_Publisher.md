# 02 - ROS Publisher 생성

> 목표 : `geometry_msgs/Twist` topic을 발행하는 Publisher 노드를 생성한다.

## geometry_msgs 패키지 추가

catkin_create_pkg 명령어를 사용해서 basic_publish_tutorial 패키지를 생성하고

geometry_msgs 패키지를 사용하기 위한 의존성을 추가해야합니다.

### CMakeLists.txt

CMakeLists.txt의 find_package()에 geometry_msgs를 추가합니다.

```cmake
find_package(catkin REQUIRED COMPONENTS
	roscpp
	geometry_msgs
)
```

### package.xml

package.xml에서 다음과 같이 스크립트를 추가합니다.

```xml
<build_depend>geometry_msgs</build_depend>

<build_export_depend>geometry_msgs</build_export_depend>

<exec_depend>geometry_msgs</exec_depend>
```

## C++ 코드 작성

basic_publish_tutorial 패키지의 src 폴더에 다음과 같이 basic_pulish.cpp 코드를 추가합니다.

```c++
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "basic_publish_node");
	ros::NodeHandle n;

	ros::Rate loop_rate(60);

	geometry_msgs::Twist cmd_vel;
	ros::Publisher publisher_cmd_vel = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

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
		publisher_cmd_vel.publish(cmd_vel);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return (0);
}
```

### geometry_msgs::Twist

geometry_msgs::Twist는 다음와 같이 정의되어 있습니다.

```c++
struct Twist {
    geometry_msgs::msg::Vector3 linear;
    geometry_msgs::msg::Vector3 angular;
};
```

여기서 Vector3는 다음과 같이 정의되어있는 구조체입니다.

```c++
struct Vector3 {
    double x;
    double y;
    double z;
};
```

이외에 geometry_msgs에 정의되어있는 message들은 [여기서](https://github.com/rticommunity/ros-data-types/tree/master/geometry_msgs/msg) 확인할 수 있습니다.

> 참고자료  
> [ros-data-types - github](https://github.com/rticommunity/ros-data-types/tree/master/geometry_msgs/msg)

## CMakelists.txt 작성

실행파일을 생성하기 위해 CMakeLists.txt에 다음과 같이 추가합니다.

```cmake
add_executable(basic_pulish_node src/basic_publish_cpp)
target_link_libraries(basic_publish_node ${catkin_LIBRARIES})
```

## 컴파일 및 실행

워크스페이스에서 catkin_make를 실행하고 devel/setup.bash를 사용하여 환경변수를 등록합니다.

```bash
catkin_make; source devel/setup.bash
```

마지막으로 rosrun 명령어를 사용하여 실행합니다.

```bash
rosrun basic_publish_tutorial basic_publish_node
```

## rostopic

발행된 토픽을 확인하기 위해 rostopic 명령어를 사용할 수 있습니다.

```bash
rostopic list
```

위 명령어를 사용하면 현재 pulish 되고 있는 topic들의 목록을 확인할 수 있습니다.

```bash
rostopic echo /cmd_vel
```

위 명령어를 사용하여 해당 topic의 값을 확인할 수 있습니다.

```bash
---
linear:
  x: 10.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 10.0
---
```

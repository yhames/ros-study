# 04 - ROS Custom Message

> 목표 : custom message를 생성한다.

## msg 파일 생성

커스텀 메시지는 다양한 자료형으로 생성할 수 있습니다.

먼저 `catkin_create_pkg` 명령어를 사용하여 패키지를 생성한 후

msg 폴더 안에 `TutorialMsg.msg` 파일을 작성합니다.

```msg
bool Bool
int8 Int8
uint8 UInt8
int16 Int16
uint16 UInt16
int32 Int32
uint32 UInt32
int64 Int64
uint64 UInt64
float32 Float32
float64 Float64
string String
time Time
duration Duration
```

## CMakeList.txt 수정

다음으로 메시지를 생성하기 위한 의존성 패키지를 추가합니다.

메시지를 생성하기 위해서는 `message_generation`와 `std_msgs`가 필요합니다.
또한 런타임에 `message_runtime`가 추가로 필요합니다.

```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  std_msgs
)
```

```cmake
generate_messages(
  DEPENDENCIES
  std_msgs
)
```

```cmake
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tutorial_msgs
  CATKIN_DEPENDS roscpp message_generation std_msgs message_runtime
#  DEPENDS system_lib
)
```

`add_message_files`에 추가하려는 메시지 파일을 명시합니다.

```cmake
add_message_files(
  FILES
  TutorialMsg.msg
)
```

## pacakge.xml 수정

위에서 추가한 패키지들을 `package.xml`에서도 추가합니다.
`message_runtime`의 경우에는 런타임에만 필요하므로 `exec_depend`에만 추가하면 됩니다.

```xml
  <build_depend>message_generation</build_depend>
  <build_export_depend>message_generation</build_export_depend>
  <exec_depend>message_generation</exec_depend>

  <build_depend>std_msgs</build_depend>
  <build_export_depend>std_msgs</build_export_depend>
  <exec_depend>std_msgs</exec_depend>

  <exec_depend>message_runtime</exec_depend>
```

## 컴파일

```bash
catkin_make
```

`devel/include/[패키지이름]` 경로에 헤더파일이 생성된다.

## 커스텀 메시지를 활용한 Publish - Subscribe 노드 생성

basic_msg_tutorial 패키지를 생성하고 다음과 같이 코드를 작성합니다.

### 의존성 추가

CMakeLists.txt의 find_package에 커스텀 메시지 패키지를 추가합니다.

```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tutorial_msgs
)
```

다음으로 package.xml의 find_package에 커스텀 메시지 패키지를 추가합니다.

```xml
  <build_depend>tutorial_msgs</build_depend>
  <build_export_depend>tutorial_msgs</build_export_depend>
  <exec_depend>tutorial_msgs</exec_depend>
```

### Publisher 코드 작성

```c++
#include <ros/ros.h>
#include <tutorial_msgs/TutorialMsg.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "basic_msg_publish_node");
    ros::NodeHandle n;

    tutorial_msgs::TutorialMsg tutorial_msg;
    ros::Publisher publisher_tutorial_msg = n.advertise<tutorial_msgs::TutorialMsg>("tutorial_msg", 1000);

    ros::Rate loop_rate(60);

    while (ros::ok())
    {
        tutorial_msg.Bool = true;
        tutorial_msg.Int8 = 127;
        tutorial_msg.UInt8 = 255;
        tutorial_msg.Int16 = 32767;
        tutorial_msg.UInt16 = 65535;
        tutorial_msg.Int32 = 2147483647;
        tutorial_msg.UInt32 = 4294967295;
        tutorial_msg.Int64 = 9223372036854775807LL;
        tutorial_msg.UInt64 = 18446744073709551615ULL;
        tutorial_msg.Float32 = 123.456f;
        tutorial_msg.Float64 = 123.456;
        tutorial_msg.String = "Tutorial";
        tutorial_msg.Time = ros::Time::now();
        tutorial_msg.Duration = ros::Duration(5.0);

        publisher_tutorial_msg.publish(tutorial_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return (0);
}
```

### add_dependencies 추가

커스텀 메시지를 사용하는 노드는 **메시지가 먼저 생성되어야 합니다**.

이를 위해 다음과 같이 의존성을 추가합니다.

```cmake
add_executable(basic_msg_publish_node src/basic_msg_publish.cpp)
target_link_libraries(basic_msg_publish_node ${catkin_LIBRARIES})
add_dependencies(basic_msg_publish_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
```

### 컴파일 후 실행

워크스페이스 경로에서 컴파일 후 실행하면 toplc list에 커스텀 메시지가` 생성된 것을 볼 수 있습니다.

```bash
/rosout
/rosout_agg
/tutorial_msg
```

rostopic echo 명령어를 사용해서 확인하면 다음과 같이 출력됩니다.

```bash
---
Bool: True
Int8: 127
UInt8: 255
Int16: 32767
UInt16: 65535
Int32: 2147483647
UInt32: 4294967295
Int64: 9223372036854775807
UInt64: 18446744073709551615
Float32: 123.45600128173828
Float64: 123.456
String: "Tutorial"
Time: 
  secs: 1710296974
  nsecs: 330578145
Duration: 
  secs: 5
  nsecs:         0
---
```

### Subscriber 코드 작성

아래와 같이 코드를 작성하고 Publisher와 동일하게 `CMakeLists.txt`에 실행파일을 추가하면

커스텀 메시지를 활용해서 통신하게 됩니다.

```c++
#include <ros/ros.h>
#include "tutorial_msgs/TutorialMsg.h"

void TutorialMsgCallback(const tutorial_msgs::TutorialMsg::ConstPtr& tutorial_msg)
{
  ROS_INFO("tutorial_msg.Bool = %s", tutorial_msg->Bool ? "true" : "false");
  ROS_INFO("tutorial_msg.Int8 = %d", tutorial_msg->Int8);
  ROS_INFO("tutorial_msg.UInt8 = %u", tutorial_msg->UInt8);
  ROS_INFO("tutorial_msg.Int16 = %d", tutorial_msg->Int16);
  ROS_INFO("tutorial_msg.UInt16 = %u", tutorial_msg->UInt16);
  ROS_INFO("tutorial_msg.Int32 = %d", tutorial_msg->Int32);
  ROS_INFO("tutorial_msg.UInt32 = %lu", (unsigned long)tutorial_msg->UInt32);
  ROS_INFO("tutorial_msg.Int64 = %lld", (long long int)tutorial_msg->Int64);
  ROS_INFO("tutorial_msg.UIn64 = %llu", (unsigned long long int)tutorial_msg->UInt64);
  ROS_INFO("tutorial_msg.Float32 = %.2f", tutorial_msg->Float32);
  ROS_INFO("tutorial_msg.Float64 = %.2lf", tutorial_msg->Float64);
  ROS_INFO("tutorial_msg.String = %s", tutorial_msg->String.c_str());
  ROS_INFO("tutorial_msg.Time = %f", tutorial_msg->Time.toSec());
  ROS_INFO("tutorial_msg.Duration = %f", tutorial_msg->Duration.toSec());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "basic_msg_subscribe_node");
  ros::NodeHandle n;

  ROS_INFO("basic_msg_subscribe_node Open");

  ros::Subscriber subscriber_tutorial_msg;
  subscriber_tutorial_msg = n.subscribe("tutorial_msg", 1000, TutorialMsgCallback);

  ros::Rate loop_rate(60);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  ROS_INFO("basic_msg_subscribe_node Close");

  return 0;
}
```
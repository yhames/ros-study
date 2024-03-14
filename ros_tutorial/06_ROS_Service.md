# 06 - ROS Service

> 목표 : Service를 사용하여 데이터를 주고받는 Server와 Client 노드를 생성한다.

## ROS Service

Service란 Node 간에 일회성을 데이터를 주고받는 방법입니다.

rosnode info 명령어를 사용하면 해당 패키지가 제공하는 service의 목록을 볼 수 있습니다.

```bash
rosnode info turtlesim
```

```bash
# ... 
Services: 
 * /clear
 * /kill
 * /reset
 * /spawn
 * /turtle1/set_pen
 * /turtle1/teleport_absolute
 * /turtle1/teleport_relative
 * /turtlesim/get_loggers
 * /turtlesim/set_logger_level
# ... 
```

turtlesim_node를 실행시키고 `rosservice list` 명령어를 사용하면,

다음과 같이 service 목록이 출력됩니다.

```bash
/clear
/kill
/reset
/rosout/get_loggers
/rosout/set_logger_level
/spawn
/turtle1/set_pen
/turtle1/teleport_absolute
/turtle1/teleport_relative
/turtlesim/get_loggers
/turtlesim/set_logger_level
```

`rosservice info` 명령어를 사용하면 해당 service의 정보를 볼 수 있습니다.

```bash
rosservice info turtle1/teleport_absolute
```

```bash
Node: /turtlesim
URI: rosrpc://jeongwpa:52117
Type: turtlesim/TeleportAbsolute
Args: x y theta
```

`Args`를 보면 인자로 `x` `y` `theta`를 갖는 것을 볼 수 있습니다.

`rosservice call` 명령어를 사용하면 service를 요청할 수 있습니다.

```bash
rosservice call /turtle1/teleport_absolute "x: 5.0 y: 10.0 theta: 3.14" 
```

위 명령어를 사용하면 거북이가 해당 방향으로 이동 및 회전하는 것을 볼 수 있습니다.

## Service 패키지 생성

### srv 파일 작성

모듈화를 위해 service를 정의하는 패키지를 생성하고,

srv 폴더 안에 TutorialSrv.srv 파일을 작성합니다.

```c
string command
---
string result
string message
```

사용할 수 있는 자료형은 msg 파일과 동일하지만,

구분자(---)를 기준으로 위쪽은 요청(request) 아래쪽은 응답(response)을 의미합니다.

### CMakeLists.txt 수정

다음과 같이 의존성 패키지를 추가합니다.

```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  std_msgs
)
```

```cmake
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tutorial_srvs
  CATKIN_DEPENDS roscpp message_generation std_msgs message_runtime
#  DEPENDS system_lib
)
```

다음으로 생성한 TutorialSrv.srv를 추가합니다.

```cmake
add_service_files(
  FILES
  TutorialSrv.srv
)
```

마지막으로 generate_messages에 std_msgs를 추가합니다.

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

### package.xml 수정

다음으로 package.xml을 수정합니다.

```xml
  <build_depend>message_generation</build_depend>
  <build_export_depend>message_generation</build_export_depend>
  <exec_depend>message_generation</exec_depend>

  <build_depend>std_msgs</build_depend>
  <build_export_depend>std_msgs</build_export_depend>
  <exec_depend>std_msgs</exec_depend>

  <exec_depend>message_runtime</exec_depend>
```

### 컴파일

워크스페이스에서 catkin_make 명령어를 사용해서 서비스 라이브러리를 생성합니다.

## Server 노드 생성

server 노드를 생성하고 CMakeLists.txt와 package.xml에 생성한 tutorial_srvs에 대한 의존성을 추가합니다.

c++ 코드를 작성한 후 실행파일을 생성하기 위해 CMakeLists.txt를 수정합니다.

이 때 서비스 라이브러리가 먼저 생성되어야하기 때문에 의존성을 추가해야합니다.

```cmake
add_dependencies(basic_service_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
```

위 코드를 추가하면 해당 노드는 컴파일 전에 서비스 라이브러리를 먼저 생성합니다.

```bash
rosservice call /tutorial_command "command: 'tutorial 1'"
```

해당 노드를 실행하고 rosservice call 명령어를 사용하면 다음과 같습니다.

```bash
result: "true"
message: "Receive success"
```

서버 노드에서는 다음과 같이 출력됩니다.

```bash
[ INFO] [1710347058.149711806]: Receive Service call tutorial command : tutorial 1
```

## Client 노드 생성

먼저 Client 노드를 위한 패키지를 생성하고 CMakeLists.txt와 package.xml에 tutorial_srvs 의존성을 추가합니다.

Service 노드와 마찬가지로 c++ 코드를 작성하고, 실행파일을 위해 CMakeLists.txt를 수정합니다.

Client 노드 또한 서비스 라이브러리가 먼저 생성되어야므로 add_dependencies를 추가해야합니다.

```cmake
add_dependencies(basic_service_client_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
```

클라이언트 노드를 실행하면 다음과 같이 응답을 받는 것을 볼 수 있습니다.

```bash
[ INFO] [1710393396.331520749]: basic_service_client_node Open
[ INFO] [1710393396.344256744]: rosservice call /tutorial_command command : 'tutorial 1'
[ INFO] [1710393396.344324345]: Service Call Response result : true
[ INFO] [1710393396.344379034]: Service Call Response message : Receive success
[ INFO] [1710393396.344457881]: basic_service_client_node Close
```

서버 노드에는 다음과 같이 출력이 됩니다.

```bash
[ INFO] [1710393353.210447139]: Receive Service call tutorial command : tutorial 1
```
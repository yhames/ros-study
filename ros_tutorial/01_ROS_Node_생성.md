# 01 - ROS Node 생성

> 목표 : 터미널에 `Hello World!`를 출력하는 Node를 생성한다.

## catkin_create_pkg

ROS는 `cmake` 기반의 `Catkin`이라는 빌드 시스템을 사용합니다.

`Catkin`을 사용하기 위해서는 최상위 폴더인 `workspace`에서 src 폴더를 만든 후

src 경로에서 다음과 같은 명령어를 사용하면 됩니다.

```bash
catkin_create_pkg [패키지명]
```

> `Node`란 최소 단위의 실행 가능한 프로세스를 의미합니다. ROS는 Node 단위로 프로그램을 작성하고 Message 통신을 기반으로 데이터를 주고 받습니다.  
> `Package`란 하나 이상의 노드 혹은 노드 실행을 위한 정보 등을 묶어놓은 것을 의미합니다.  
> Message란 노드 간에 주고받는 데이터를 의미합니다. 변수, 구조체, 배열 등의 형태로 사용할 수 있습니다.  
> `Workspace`란 ROS와 관련된 코드를 모아두는 디렉토리를 의미합니다. 프로젝트 최상위 경로의 `src` 폴더 안에서 패키지를 생성해야 이후에 `catkin_make`를 사용하여 컴파일할 수 있습니다.

## C++ 코드 작성

catkin으로 패키지를 생성하면 여러가지 파일과 폴더가 생성된 것을 볼 수 있습니다.

먼저 src 폴더에서 basic.cpp 파일에 다음과 같은 코드를 생성합니다.

```C++
#include <ros/ros.h>

/**
 * 기본 ROS 템플릿
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "basic_node");
	ros::NodeHandle n;

	ros::Rate loop_rate(60);	// node가 몇 Hz로 동작할 것인지 정의

	while(ros::ok())	// 종료시 false
	{
		ROS_INFO("Hello World!");
		ros::spinOnce();
		loop_rate.sleep();	// 위에서 정의한 Hz만큼 대기
	}
	return (0);
}
```

## CMakelists.txt 작성

작성된 cpp 파일을 빌드하기 위해서는 `CMakelists.txt` 파일에 등록해야합니다.

생성되어있는 Cmakelists.txt 파일에 다음과 같은 위치에

`add_executable`과 `target_link_libraries`를 추가합니다.

```cmake
include_directories(
# include
	${catkin_INCLUDE_DIRS}
)

add_executable(basic_node src/basic.cpp)	# 실행파일을 추가합니다.
target_link_libraries(basic_node ${catkin_LIBRARIES})	# 필요한 라이브러리를 가져옵니다.
```

## catkin_make

워크스페이스 경로에서 다음 명령어를 사용하여 컴파일을 합니다.

```shell
catkin_make
```

## setup.bash 등록

쉘에서 컴파일된 노드를 인식할 수 있도록 `devel` 폴더의 `setup.bash`를 실행합니다.

```shell
source devel/setup.bash
```

## roscore

노드를 실행하기 위해서는 `roscore`가 실행되어 있어야합니다.

`roscore`는 노드 정보와 파라미터를 관리하는 마스터 노드입니다.

다음 명령어를 사용하여 roscore를 실행시킵니다.

```
roscore
```

## rosrun

rosrun 명렁어를 사용하여 컴파일된 실행파일을 실행시킬 수 있습니다.

다음 명령어를 다른 터미널 창에서 실행시킵니다.

```
rosrun [패키지명] [노드_이름]
```

실행하면 다음과 같이 출력됩니다.

```shell
[ INFO] [1710170550.084971378]: Hello World!
[ INFO] [1710170550.101662940]: Hello World!
[ INFO] [1710170550.117662690]: Hello World!
[ INFO] [1710170550.134983524]: Hello World!
[ INFO] [1710170550.151362012]: Hello World!
[ INFO] [1710170550.168253650]: Hello World!
[ INFO] [1710170550.184962587]: Hello World!
[ INFO] [1710170550.201651690]: Hello World!
[ INFO] [1710170550.217474779]: Hello World!
[ INFO] [1710170550.234967941]: Hello World!
[ INFO] [1710170550.251667252]: Hello World!
[ INFO] [1710170550.268273983]: Hello World!
[ INFO] [1710170550.283784582]: Hello World!
```

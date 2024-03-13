# 05 - ROS Launch

> 목표 : launch 파일을 작성하여 여러 노드를 한번에 실행한다.

## ROS Launch란?

ROS Launch는 한 번의 명령으로 여러개의 노드를 실행하는 도구입니다.

ROS Launch의 기능들은 다음과 같습니다.

* 여러개의 노드 실행
* 실행시 roscore 자동 실행
* Node의 이름을 변경하여 실행
* 파라미터 설정
* 노드 출력, 노드 종료시 재실행 등 기타 옵션 설정

## local launch 파일 작성

ROS Launch를 사용할 패키지에서 launch 폴더를 생성하고 `[패키지명].launch` 파일을 생성합니다.

launch 파일의 기본 구조는 다음과 같습니다.

```xml
<?xml version="1.0"?>

<launch>

</launch>
```

### 노드 태그 작성

launch 태그 안에 node 태그를 작성하면 특정 노드를 실행시킬 수 있습니다.

```xml
<launch>
    <node pkg="basic_tutorial" type="basic_node" name="basic_node"/>
</launch>
```

node 태그의 기본 옵션은 다음과 같습니다.

* pkg는 패키지 이름
* type은 실행파일 이름
* name은 노드 이름

output과 같은 추가적인 옵션을 사용할 수도 있습니다.

```xml
<launch>
    <node pkg="basic_tutorial" type="basic_node" name="basic_node" output="screen"/>
</launch>
```

### 같은 노드를 여러개 실행

launch 파일을 사용해서 **같은 노드**를 다른 이름으로 **여러개** 실행할 수 있습니다.

```xml
<launch>
    <node pkg="basic_tutorial" type="basic_node" name="basic_node1" output="screen"/>
    <node pkg="basic_tutorial" type="basic_node" name="basic_node2" output="screen"/>
</launch>
```


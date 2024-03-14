# 06 - ROS Network

> 목표 : 다른 PC에서 실행중인 노드와 통신한다.

## ROS Network

ROS의 노드는 네트워크 설정을 통해 다른 PC의 노드와 통신할 수 있습니다.

ROS 네트워크에는 `Master`와 `Host`가 있습니다.

* Master는 네트워크의 중심 역할을 합니다.

* Host는 Master의 네트워크에 참여하여 Host 간에 데이터를 주고 받습니다.

ROS 네트워크는 **노드를 분산**하여 연산 부담을 줄이거나 **다중 로봇을 제어**하는데 사용됩니다.

## 네트워크 설정

ROS 네트워크를 설정하기 위해서는 참여하고자 하는 Host의 .bashrc 파일에 다음과 같이 환경변수를 추가해야합니다.

```bash
export ROS_MASTER_URI=http://[IP]:11311
export ROS_HOSTNAME=[IP]
```

이 때 Master 또한 자신의 네트워크에 Host로서 참여해야 합니다.

```bash
# Master
export ROS_MASTER_URI=http://192.168.150.141:11311
export ROS_HOSTNAME=http://192.168.150.141
```

```bash
# Host
export ROS_MASTER_URI=http://192.168.150.141:11311
export ROS_HOSTNAME=http://192.168.150.52
```

## 네트워크 실행

Master에서 `roscore`를 실행하면 설정된 네트워크 환경(`ROS_MASTER_URI`)으로 ROS가 실행됩니다.
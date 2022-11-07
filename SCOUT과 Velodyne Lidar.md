### velodyne 구동

- velodyne lidar 관련 패키지 다운 및 기타설정

```bash
$cd ~/catkin_ws/src/ && git clone https://github.com/ros-drivers/velodyne.git
```

```bash
$rosdep install --from-paths src --ignore-src --rosdistro YOURDISTRO -y
```

```bash
$ cd ~/catkin_ws/ && catkin_make
```

```bash
$ source /devel/setup.bash
```



- velodyne lidar 실행

```bash
$ roslaunch velodyne_pointcloud VLP16_points.launch
```



- velodyne lidar를 RVIZ를 이용한 시각화

```bash
$ rosrun rviz rviz -f velodyne
```



- 포인트클라우드 raw데이터 확인

```bash
$ rostopic echo /velodyne_points
```



### SCOUT 가동

#### 1. 직접 짠 코드를 사용하기 전 키보드로 조작해보기 위한 ROS 패키지를 다운받는다.

```bash
$ sudo apt install ros-melodic-teleop-twist-keyboard
$ sudo apt-get install ros-melodic-joint-state-publisher-gui
$ sudo apt install ros-melodic-ros-controllers
```



#### 2. SCOUT을 가동하기 위해 SCOUT관련 패키지를 다운받아 빌드한다.

```bash
$ cd ~/catkin_ws/src
$ git clone -b scout_v2 --depth 1 https://github.com/agilexrobotics/scout_ros.git	
$ git clone -b scout_v2 --depth 1 https://github.com/agilexrobotics/agx_sdk.git
$ cd ..
$ catkin_make
```

- 만약 asio.hpp 오류가 발생할 때

- ```bash
  $sudo apt-get install libasio-dev
  ```



#### 3. SCOUT 가동

- ROS 노드 실행을 위해 ROS마스터와 ROS파라미터 실행

```bash
$ roscore
```

- 캔 bringup을 위해 아래 명령어 실행

```bash
$ sudo ip link set can0 up type can bitrate 500000
```

- 캔을 이용해 연결하면 아래 명령어를 이용해 스카웃 브링업

```bash
$ roslaunch scout_bringup scout_minimal.launch
```

- 직접 짠 코드 및 키보드 조작을 위한 명령어 실행

```bash
$ roslaunch scout_bringup scout_teleop_keyboard.launch
```

위 메뉴얼의 원본 >> https://github.com/agilexrobotics/scout_ros





### REMOTE PC에서 SCOUT과 직접 연결된 PC조작 방법

1. REMOTE PC의 bashrc 에서 ROS_HOSTNAME, ROS_MASTER_URI를 remote ip로 변경하기

2. SCOUT과 직접 연결된 PC의 bashrc에서 ROS_MASTER_URI는 remote ip로, ROS_HOSTNAME은은 SCOUT과 직접 연결된 PC의 ip로 변경하기

3. roscore 실행

   ```bash
   $ roscore
   ```

   

4. scout과 직접 연결된 PC(scout)에 원격접속

   ```bash
   $ ssh scout@{scout연결된PC ip}
   만약 연결이 안된다면 scout pc를 재부팅 해보기
   ```

   원격 접속한 터미널 창에서 실행

   ```bash
   $ sudo ip link set can0 up type can bitrate 500000
   ```

   ```bash
   $ roslaunch scout_bringup scout_mini_robot_base.launch
   ```

   

5. REMOTE PC터미널 창에서 조종 명령어 실행 (현재 ssh 터미널에서 실행해야 가능함)

   ```bash
   $ roslaunch scout_bringup scout_teleop_keyboard.launch
   ```

위 과정을 하기 위해선 로봇과 컨트롤러의 전원이 켜지고 로봇과 누크(작은 PC)가 연결되어 있어야 합니다.


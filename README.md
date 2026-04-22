# RCI_quadruped_robot_navigation

Gazebo 환경에서 Unitree 사족보행 로봇의 강화학습 기반 제어와 내비게이션을 실행하기 위한 ROS 2 Humble 프로젝트입니다.

이 문서는 `agbread/RCI_quadruped_robot_navigation` 저장소의 `1st_project` 브랜치를 기준으로 작성되었습니다.

---

## 1. 프로젝트 개요

이 프로젝트는 Gazebo 시뮬레이터에서 Unitree 계열 사족보행 로봇(Go2, Go2W, B2)을 대상으로 강화학습 기반 제어와 내비게이션 기능을 통합한 ROS 2 프로젝트입니다.

주요 기능은 다음과 같습니다.

- Gazebo 상에서 로봇 시뮬레이션 실행
- RL 정책 기반 보행 제어
- Action client를 통한 상위 명령 전송
- Nav2 기반 내비게이션
- Stair Behavior Tree 기반 계단 이동
- 선택적으로 elevator / door 관련 기능 포함

---

## 2. 개발 환경

- Ubuntu
- ROS 2 Humble
- colcon
- Gazebo
- RViz2

---

## 3. 사전 설치

### 3.1 ROS 2 패키지 설치

```bash
sudo apt update
sudo apt install -y \
  ros-humble-teleop-twist-keyboard \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-control-toolbox \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-gazebo-ros2-control \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-xacro \
  ros-humble-navigation2 \
  ros-humble-octomap-ros \
  ros-humble-octomap-rviz-plugins
```

### 3.2 LibTorch 설치

```bash
mkdir -p ~/libs
cd ~/libs
wget https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.0.1%2Bcpu.zip
unzip libtorch-cxx11-abi-shared-with-deps-2.0.1+cpu.zip
rm libtorch-cxx11-abi-shared-with-deps-2.0.1+cpu.zip

echo 'export Torch_DIR=$HOME/libs/libtorch' >> ~/.bashrc
source ~/.bashrc
```

### 3.3 추가 라이브러리 설치

```bash
sudo apt install -y liblcm-dev libyaml-cpp-dev
```

---

## 4. 저장소 다운로드

`1st_project` 브랜치를 기준으로 작업하려면 아래와 같이 clone 합니다.

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

git clone --branch 1st_project --single-branch --recursive \
  https://github.com/agbread/RCI_quadruped_robot_navigation.git
```

이미 저장소를 clone 해 둔 상태라면 아래처럼 브랜치를 전환할 수 있습니다.

```bash
cd ~/ros2_ws/src/RCI_quadruped_robot_navigation
git checkout 1st_project
git submodule update --init --recursive
```

---

## 5. 빌드 방법

아래 명령으로 workspace를 빌드합니다.

```bash
cd ~/ros2_ws
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_POLICY_VERSION_MINIMUM=3.5
```

빌드가 완료되면 아래 명령으로 환경을 적용합니다.

```bash
source ~/ros2_ws/install/setup.bash
```

매번 자동으로 source 하고 싶다면 다음을 실행합니다.

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 6. 기본 실행 방법

### 6.1 Gazebo + RViz2 실행

기본 로봇은 `go2`입니다.  
`rname` 인자를 사용하여 `go2w`, `b2` 등으로 변경할 수 있습니다.

```bash
# 기본 Go2 실행
ros2 launch rl_sar gazebo.launch.py

# Go2W 실행
ros2 launch rl_sar gazebo.launch.py rname:="go2w"

# B2 실행
ros2 launch rl_sar gazebo.launch.py rname:="b2"
```

### 6.2 RL Controller 실행

로봇의 보행 제어를 담당하는 노드를 실행합니다.

```bash
ros2 run rl_sar rl_sim
```

### 6.3 Action Client 실행

상위 명령 또는 goal을 전송하는 action client를 실행합니다.

```bash
ros2 run rl_action command
```

---

## 7. Navigation 실행 방법

### 7.1 Velodyne PointCloud2를 LaserScan으로 변환

```bash
ros2 run velodyne_laserscan velodyne_laserscan_node --ros-args \
  -r velodyne_points:=/velodyne_points \
  -r scan:=/scan
```

### 7.2 Nav2 실행

사용 환경에 따라 아래 둘 중 하나를 사용합니다.

```bash
ros2 launch rl_sar nav2.launch.py
```

또는

```bash
ros2 launch quadruped_nav2 quadruped_nav2_bringup.launch.py
```

---

## 8. Stair Behavior Tree 실행 방법

Stair BT는 목표 층 요청을 받아 현재 층을 판단하고, 필요한 경우 계단 입구까지 이동한 뒤 계단 이동 시퀀스를 수행합니다.

### 8.1 Stair BT 실행

공식 예시는 아래와 같이 XML 경로를 명시하는 방식입니다.

```bash
ros2 run stair_bt stair_bt_runner --ros-args \
  -p bt_xml_file:="$(ros2 pkg prefix stair_bt)/share/stair_bt/bt_trees/stairs.xml"
```

환경 설정에 따라 아래처럼 간단히 실행하는 경우도 있습니다.

```bash
ros2 run stair_bt stair_bt_runner
```

### 8.2 목표 층 요청 publish

예를 들어 1층으로 이동 요청을 보내려면 아래 명령을 실행합니다.

```bash
ros2 topic pub /stairs/floor_request std_msgs/msg/Int32 "{data: 1}"
```

---

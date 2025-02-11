#!/bin/bash
# ***************************The sim Map is three times larger than the real***************************
# 获取当前脚本的绝对路径
FILEDIR=$(readlink -f "${BASH_SOURCE[0]}")
BASEDIR=$(dirname "${FILEDIR}")
SETUP_FILE="${BASEDIR}/../../devel/setup.bash"
SETUP_DIR=$(dirname "${SETUP_FILE}")

echo "setup.bash is located in: ${SETUP_DIR}"

# 加载 ROS 环境
source "${SETUP_FILE}"

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4_Firmware
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4_Firmware/Tools/sitl_gazebo

# 启动 Gazebo仿真环境
sleep 0.5s
gnome-terminal --tab -- bash -c "\
echo Gazebo; \
roslaunch myer mylanding2.launch; exec bash"

# 启动键盘控制节点(控制车移动) 和 rviz
sleep 2.5s
gnome-terminal --tab -- bash -c "\
echo Keyboard and Rviz; \
roslaunch myer scripts2.launch; exec bash"

# 启动控制器
sleep 1.5s
gnome-terminal --tab -- bash -c "\
echo se3_controller; \
roslaunch se3_for_landing px4_example.launch; exec bash"

# 发送起飞指令
sleep 5.0s
gnome-terminal --tab -- bash -c "\
echo 'FLY!!!'; \
rosservice call /mavros/set_mode '{base_mode: 0, custom_mode: \"OFFBOARD\"}'; \
sleep 0.5s; \
rosservice call /mavros/cmd/arming '{value: true}'; \
exec bash"


wait;

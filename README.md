# shortest-path
Dijkstra, shortest path, bazier, smooth

进入工作空间

cd ~/catkin_ws

建立工作环境

source me

ROS转化成eclipse project 命令

catkin_make --force-cmake -G"Eclipse CDT4 - Unix Makefiles"

cmake -G"Eclipse CDT4 - Unix Makefiles" ..

打开eclipse并进行处理

/usr/lib/eclipse/eclipse

集成启动脚本

bash launch.bash


使用GitHub
添加所有更新文件

git add .

添加注释

git commit -m "balabala"

推送到远程主机

git push -u origin master

启动仿真界面

rosrun rviz rviz

开启读取飞空数据流.
???manifold.launch 启动命令

典型launch启动命令

roslaunch iarc_arena_simulator simulator.launch

roslaunch dji_sdk_demo dji_sdk_client_2layer.launch

roslaunch dji_sdk_demo dji_sdk_client_3layer.launch

roslaunch iarc_arena_simulator simulator_all3layer.launch

roslaunch iarc_arena_simulator simulator7.launch

roslaunch iarc_arena_simulator simulator_cruise.launch

rosrun dji_sdk_demo dji_sdk_test_bazier

rosrun dji_sdk_demo dji_sdk_test_graph

rosrun dji_sdk_demo dji_sdk_test_planner_path

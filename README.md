# shortest-path
Dijkstra, shortest path, bazier, smooth

进入工作空间
cd workspace/hehualin_ws/IARC
建立工作环境
source me

ROS转化成eclipse project 命令
catkin_make --force-cmake -G"Eclipse CDT4 - Unix Makefiles"
cmake -G"Eclipse CDT4 - Unix Makefiles" ..

打开eclipse并进行处理
/usr/lib/eclipse/eclipse

roslaunch iarc_arena_simulator simulator.launch
roslaunch dji_sdk_demo dji_sdk_client_2layer.launch
roslaunch dji_sdk_demo dji_sdk_client_3layer.launch
roslaunch iarc_arena_simulator simulator_all3layer.launch
roslaunch iarc_arena_simulator simulator7.launch
roslaunch iarc_arena_simulator simulator_cruise.launch
rosrun dji_sdk_demo dji_sdk_test_bazier
rosrun dji_sdk_demo dji_sdk_test_graph
rosrun dji_sdk_demo dji_sdk_test_planner_path
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

打开driver
roslaunch velodyne_driver pcap_nodelet_hertz.test

打开cloud
roslaunch velodyne_pointcloud cloud_nodelet_hz.test
roslaunch velodyne_pointcloud cloud_node_hz.test

roslaunch velodyne_driver nodelet_manager.launch
roslaunch velodyne_pointcloud cloud_nodelet.launch

roslaunch velodyne_pointcloud cloud_nodelet_cali.test

./online_node "/media/exbot/283836FC3836C898/SN257_error001.pcap" "/media/exbot/022C83182C83063F/ADV/velodyne/AllCarParams/adu/params/S3-Unit#257.yaml" "/media/exbot/022C83182C83063F/ADV/velodyne/AllCarParams/adu/params/S3-Unit#257.yaml.online.yaml"

cmake -D CMAKE_BUILD_TYPE=RELEASE

开启读取飞空数据流.

(1)Target interaction aim position estimate. (a lower frequent for 1sec, for a higher frequent but has to estimate each time?)
(2)Tasks list update function. what if a new task list arrived when a task is operating ?
(3)Tasks list oscillization is very frequently..
(4)correctness of decision model still need to be checked..
(5)normalization of reward function is meanningful, it has been revised.
(6)searching terminate condition , what if a robot is predicted to be drived into target arena in searching? what is in_arena() for a moving robot?
(7)assume that the robot has infinite ability to reach target position so that to reach target position, and so so.
(8)is it possible to avoid consideration of obstacles?
(9)it is possible to revised AF into a sistem that can consider the arrival time.
(10)init a guard condition.( if the guad condition is satisfied, do not change strategy, if the guard condition
is not satisfied, change strategy )
(11)task status tolerate or task status complete (judge this )  loop rate change into 20Hz, a completely event trigger system..

(12)write my article 



#!/usr/bin/env bash

#/home/hitcsc/.ros/arena_cmd_received.txt
#/home/hitcsc/.ros/arena_info.txt
#/home/hitcsc/.ros/CIARC.txt
#/home/hitcsc/.ros/CIARC_Controller.txt
#/home/hitcsc/.ros/CIARC_Pathlist.txt
#/home/hitcsc/.ros/CIARC_Task.txt
#/home/hitcsc/.ros/CIARC_Tasklist.txt

#rosrun dji_sdk_demo dji_sdk_test_bazier
#python plot2.py

cd test
rosrun dji_sdk_demo dji_sdk_test_planner_path
python plot_edgemap.py

#rosrun dji_sdk_demo dji_sdk_test_bazier
#python plot_bazier.py
cd ..

echo "Complete!"



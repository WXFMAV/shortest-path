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

case $1 in
    1)
        catkin_make
        rosrun dji_sdk_demo dji_sdk_test_planner_path
        cd test        
        python plot_pathplan.py
        eog plot_edgemap.png
        cd ..
    ;;
    2)
        catkin_make
        curpath=$(pwd)
        temppath=$(pwd)"/build/test_results"
        for i in `seq 3`
        do
            paths=$curpath"/data/$i"
            echo $paths
            mkdir -p $paths
            rm $temppath"/*.*"
            roslaunch iarc_arena_simulator simulator_cruise.launch	
            mv $temppath"/rec_arena_info.txt" $paths"/arena_info.txt"
            mv $temppath"/rec_controller.txt" $paths"/CIARC_Controller.txt" 
            mv $temppath"/rec_arena_cmd_received.txt" $paths"/arena_cmd_received.txt" 
        done      
    ;;
    all|*)
	catkin_make
        rm ./build/test_results/*.*
        roslaunch iarc_arena_simulator simulator_cruise.launch	
esac

#exit

#cd test

#rosrun dji_sdk_demo dji_sdk_test_planner_path
#python plot_pathplan.py
#eog plot_edgemap.png
#rosrun dji_sdk_demo dji_sdk_test_planner_path
#python plot_edgemap.py

#rosrun dji_sdk_demo dji_sdk_test_bazier
#python plot_bazier.py

#rosrun dji_sdk_demo dji_sdk_test_graph
#python plot_timecost_edge.py
#python plot_timecost.py

#cd ..

#rm ./build/test_results/*.*
#roslaunch iarc_arena_simulator simulator_cruise.launch

echo "Complete!"




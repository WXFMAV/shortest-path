#!/usr/bin/env bash

#/home/hitcsc/.ros/arena_cmd_received.txt
#/home/hitcsc/.ros/arena_info.txt
#/home/hitcsc/.ros/CIARC.txt
#/home/hitcsc/.ros/CIARC_Controller.txt
#/home/hitcsc/.ros/CIARC_Pathlist.txt
#/home/hitcsc/.ros/CIARC_Task.txt
#/home/hitcsc/.ros/CIARC_Tasklist.txt

files="/home/hitcsc/.ros/arena_cmd_received.txt /home/hitcsc/.ros/arena_info.txt /home/hitcsc/.ros/CIARC.txt /home/hitcsc/.ros/CIARC_Controller.txt /home/hitcsc/.ros/CIARC_Pathlist.txt /home/hitcsc/.ros/CIARC_Task.txt /home/hitcsc/.ros/CIARC_Tasklist.txt"
curpath=$(pwd)
datapath=$curpath"/data"

echo $curpath
echo $files
echo $datapath

mkdir $datapath

for i in `seq 40`
do
  paths=$datapath"/$i"
  echo $paths
    
  mkdir $paths

  roslaunch iarc_arena_simulator simulator.launch

  mv -t $paths $files
     
done


for i in `seq 40`
do
  paths=$datapath"/nocmd_$i"  
  echo $paths
    
  mkdir $paths

  roslaunch iarc_arena_simulator simulator_nocmd.launch

  mv -t $paths $files
     
done


echo "Complete!"




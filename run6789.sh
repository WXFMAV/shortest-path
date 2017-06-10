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
datapath=$curpath

echo $curpath
echo $files
echo $datapath

for i in `seq 50`
do

  paths=$datapath"/data6/$i"
  echo $paths
    
  mkdir -p $paths

  roslaunch iarc_arena_simulator simulator6.launch

  mv -t $paths $files


done

for i in `seq 50`
do
  paths=$datapath"/data7/$i"
  echo $paths
    
  mkdir -p $paths

  roslaunch iarc_arena_simulator simulator7.launch

  mv -t $paths $files
     
done


for i in `seq 50`
do
  paths=$datapath"/data8/$i"
  echo $paths
    
  mkdir -p $paths

  roslaunch iarc_arena_simulator simulator8.launch

  mv -t $paths $files
     
done



for i in `seq 50`
do
  paths=$datapath"/data9/$i"
  echo $paths
    
  mkdir -p $paths

  roslaunch iarc_arena_simulator simulator9.launch

  mv -t $paths $files
     
done

echo "Complete!"




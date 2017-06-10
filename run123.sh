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

  paths=$datapath"/data1/$i"
  echo $paths
    
  mkdir -p $paths

  roslaunch iarc_arena_simulator simulator1.launch

  mv -t $paths $files


done

for i in `seq 50`
do
  paths=$datapath"/data2/$i"
  echo $paths
    
  mkdir -p $paths

  roslaunch iarc_arena_simulator simulator2.launch

  mv -t $paths $files
     
done


for i in `seq 50`
do
  paths=$datapath"/data3/$i"  
  echo $paths
    
  mkdir -p $paths

  roslaunch iarc_arena_simulator simulator3.launch

  mv -t $paths $files
     
done

echo "Complete!"




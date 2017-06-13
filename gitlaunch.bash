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

sourcepath=/home/exbot/workspace/hehualin_ws/IARC

case $1 in
    fresh)
        rm -r ./build
        rm -r ./devel
        cp $sourcepath/*.* .
        cp -r $sourcepath/src .
        cp -r $sourcepath/test .
    ;;
    encode)
        zip -re src.zip ./src
#        zip -re filename.zip filename
#        unzip filename.zip 按提示输入密码  
    ;;
    decode)
        unzip src.zip .       
    ;;
    pull)
        git pull 
#https://github.com/WXFMAV/shortest-path.git
#https://github.com/WXFMAV/shortest-path.git
    ;;
    push)
        git add .
        git commit -m "upate files"
        git push origin master   
    ;;
    all|*)
esac

echo "Complete!"




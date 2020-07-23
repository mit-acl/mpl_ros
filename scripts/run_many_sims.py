#!/usr/bin/env python
# coding=utf-8
# Jesus Tordesillas, jtorde@mit.edu
# date: July 2020

import math
import os
import sys
import time
import rospy
from snapstack_msgs.msg import State
import subprocess
import rostopic

def waitUntilRoscoreIsRunning():
    #  https://github.com/ros-visualization/rqt_robot_plugins/blob/eb5a4f702b5b5c92b85aaf9055bf6319f42f4249/rqt_moveit/src/rqt_moveit/moveit_widget.py#L251
    is_roscore_running=False;

    while(is_roscore_running==False):
        try:
            rostopic.get_topic_class('/rosout')
            is_roscore_running = True
        except rostopic.ROSTopicIOException as e:
            is_roscore_running = False
            pass
    print("Roscore is running!")


def launchCommandAndWaitUntilFinish(command):
        session_name="untitled"
        os.system("tmux kill-session -t" + session_name)
        os.system("tmux new -d -s "+str(session_name)+" -x 300 -y 300")

        commands=[command]

        for i in range(len(commands)):
            print('splitting ',i)
            os.system('tmux split-window ; tmux select-layout tiled')

        for i in range(len(commands)):
            os.system('tmux send-keys -t '+str(session_name)+':0.'+str(i) +' "'+ commands[i]+'" '+' C-m')

        print("Commands sent")
 
        #waitUntilRoscoreIsRunning();
        # os.system("rostopic echo /rosout |grep 'End of simulation' -m 1") #-m 1 will make it return as soon as it finds the first match.

        output_string=""
        while (output_string.find('data: True')==-1):
            try:  #['rostopic', 'echo', '/rosout', '|grep','simulation']
                output_string =str(subprocess.check_output("rostopic echo /end_of_sim -n 1", shell=True)) #, '|grep', 'Results' , '-n', '1'

            except:
                print("An Error occurred")

        time.sleep(3.0)
        print("Sim has finished")
        print("Killing the rest")
        os.system(kill_all)

def writeToFile(sentence):
        file_object = open('./results.txt', 'a')
        file_object.write(sentence);
        file_object.close()

if __name__ == '__main__':

    kill_all="tmux kill-server & killall -9 gazebo & killall -9 gzserver & pkill -f swarm_traj_planner & killall -9 multi_robot_node & killall -9 gzclient & killall -9 roscore & killall -9 rosmaster & pkill faster_node & pkill -f dynamic_obstacles & pkill -f rosout & pkill -f behavior_selector_node & pkill -f rviz & pkill -f rqt_gui & pkill -f perfect_tracker & pkill -f faster_commands"

    #make sure ROS (and related stuff) is not running
    os.system(kill_all)

 
    u_all=[2.0,3.0,4.0,5.0];
    decentralized=["false","true"]
    for i in range(len(decentralized)):
        for j in range(len(u_all)):
            command="roslaunch mpl_test_node test_multi_robot.launch rviz:=false u:="+str(u_all[j]) +" decentralized:="+str(decentralized[i]); #+" runsim:=false log:=false "; #>> $(rospack find swarm_planner)/scripts/results.txt
            launchCommandAndWaitUntilFinish(command);

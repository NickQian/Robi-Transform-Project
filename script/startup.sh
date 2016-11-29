#!/bin/bash

# no for demo purpose
ros_dbg=yes
cmd_ros_compile="catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /home/pi/pi/toby"

cmd_ros_run_servos="rosrun actprocess servos"
cmd_ros_run_actPattern="rosrun actprocess actPattern.py"
cmd_ros_run_cortexp="rosrun cortex cortexp.py"
cmd_ros_run_sens="rosrun sensorprocess sens.py"
cmd_ros_run_cortexn="rosrun cortex cortexn.py"
cmd_ros_run_ap_i="rosrun audioprocess ap_i.py"
cmd_ros_run_ap_o="rosrun audioprocess ap_o.py"
cmd_ros_run_vp_svc="rosrun visualprocess vp_svc.py"
cmd_ros_run_vp="rosrun visualprocess vp.py"


if [ $ros_dbg = yes ]; then

   echo 'It is DDBBGG mode. openning development windows....'

   #--------------------- source edit & compile window-----------------------------
   terminator --layout=SrcCompileLayout --working-directory=/home/pi/toby_ws/src/audioprocess &
   sleep 2s

   #./send2startup.exp $cmd_ros_compile

   #------ leafpad to segment terminals-
   leafpad /home/pi/Desktop/ros-cmd-setup &
   sleep 1s


   #------------------------------ Run windows ------------------------------------
   # login as root which will run mmap
   gksu "/usr/bin/x-terminal-emulator --geometry=600x200+100+500 --working-directory=/home/pi" & 
   sleep 2s
   #--command="rosrun actprocess servos" &

   #terminator --layout=RunLayout --working-directory=/home/pi &
   terminator --geometry=600x200+150+450 --working-directory=/home/pi/ &
   sleep 1s

   terminator --geometry=600x200+200+400 --working-directory=/home/pi/ &
   sleep 1s

   terminator --geometry=600x200+250+350 --working-directory=/home/pi/ &
   sleep 1s

   terminator --geometry=600x200+300+300 --working-directory=/home/pi/ &
   sleep 1s

   terminator --geometry=600x200+350+250 --working-directory=/home/pi/ &
   sleep 1s

   terminator --geometry=600x200+400+200 --working-directory=/home/pi/ &
   sleep 1s

   terminator --geometry=600x200+450+150 --working-directory=/home/pi/ &
   sleep 1s

   terminator --geometry=800x700+500+10 --working-directory=/home/pi/ &
   sleep 3s

   #(./send2startup.exp "$cmd_ros_run_cortexp" 200 400)&
   #(./send2startup.exp "$cmd_ros_run_cortexp" 300 350)&
   #(./send2startup.exp "$cmd_ros_run_cortexp" 400 300)&
   #(./send2startup.exp "$cmd_ros_run_cortexp" 500 250)&


   # ------dirs to segment last 2 terminals---
   pcmanfm /home/pi/toby_ws/src &
   pcmanfm /home/pi/Documents/TobyQin/learn /home/pi/Documents/TobyQin/ros/ &
   pcmanfm /home/pi/pi/robi/share/cortex/install  /home/pi/pi/robi/share/visualprocess/install &

   #------------------------------ roscore and cp back window ------------------------------------
   # "roscore" and cp *.py back
   terminator --layout=CpBackLayout --working-directory=/home/pi/pi/robi &
   sleep 2s                  #wait for roscore ready


   #expect <<-END
#	spawn x-terminal-emulator --geometry=600x400+150+500 --working-directory=/home/pi/toby_ws
#	send "$cmd_ros_run_cortexp"
#	interact
#	expect eof
#	END

   echo "roscore"
   echo $cmd_ros_run_servos
   echo $cmd_ros_run_sens
   echo $cmd_ros_run_ap_i
   echo $cmd_ros_run_ap_o
   echo $cmd_ros_run_actPattern
   echo $cmd_ros_run_cortexp
   echo $cmd_ros_run_vp_svc
   echo $cmd_ros_run_vp
   echo $cmd_ros_run_cortexn

   echo "-------------------------------start up Done!--------------------------------"

else
   echo "It's NOT DBG mode"
   lxterminal -e roslaunch demo.launch 

fi

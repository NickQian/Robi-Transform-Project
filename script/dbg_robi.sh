#!/bin/bash

# C source edit & compile
lxterminal --working-directory=/home/pi/toby_ws/src/audioprocess/src
lxterminal --working-directory=/home/pi/toby_ws

# ROS dbg windows 
terminator&
lxterminal --working-directory=/home/pi --geometry=60x20
lxterminal --working-directory=/home/pi --geometry=70x20
lxterminal --working-directory=/home/pi --geometry=80x20
lxterminal --working-directory=/home/pi --geometry=80x20

# "roscore" and cp *.py back 
lxterminal --working-directory=/home/pi/pi/toby --geometry=20x30 -e ls
lxterminal --geometry=20x30

gksu /usr/bin/x-terminal-emulator

pcmanfm /home/pi/.config/lxsession/LXDE
pcmanfm /home/pi/Documents/TobyQin/learn 
pcmanfm /home/pi/pi/toby/share/cortex/install /home/pi/pi/toby/lib/actprocess/install
leafpad /home/pi/Desktop/ros-cmd-setup

#rosrun actprocess servos

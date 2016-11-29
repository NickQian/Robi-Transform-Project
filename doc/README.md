# HW setup:
1. HW list:   <br/>
Raspberry Pi3 + Xunfei XFM10211(USB port) + Robi   <br/>

2. Steps <br/>
1) connect robi servo's 'CTRL'(or 'Signal') together and tight them to Pi3's GPIO pin 8&9(TXD0&RXD0). connect the GND of servo and Pi3.<br/>
2) I just connect the 5V from GPIO to servo VCC supply. The better way may be providing indepedent power supply to servo VCC. <br/>
3) connect a speaker to Pi3's audio jack. <br/>
4) type ./startup.sh in a ternimal(before typing you need to copy the script to the directory you work) <br/>
5) input the command print on the terminal into the terminals opened by the "startup.sh" script. <br/>
6) last command is: rosrun cortex cortexn.py  
   This means that the top brain of robi is running :)  


2016.11.29 <br/>

3 TODO:
1) servo action server now only support 1 servo command during server period. Need to support all servo at the same time.
2) sensors now are blank (except microphone and camera). Need to add sensors driver and integrate them into ROS to let them publish data.
3) Robi cannot move right now. need to add a balance car under the legs and let them move: controlled by BT.
4) When speaking the microphone and capture the sound and record it. This is stupid. Need a way to improve this.
5) STT and TTS are not good. STT using Xunfei cloud and too slow. TTS result is not smooth. Needs better solution.
6) No object detection right now. Maybe tensor box is a solution. Object recognization using inception has not been integrated into ROS. also face recognization.
7) dialog match now using AIML. need to imporove using CNN things.


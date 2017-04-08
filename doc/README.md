# HW Setup & TODO:  <br/>

1. HW list:   <br/>
   1) Raspberry Pi 3B(with raspbian) <br/>
   2) sensors:  a) MPU6050 + HMC5883L + BMP180 module(search "GY-87" in taobao)  <br/>
                b) ultrasonic module (search "HC-SR04" in taobao) <br/>
                c) usb mic (Xunfei XFM10211 usb mic array is a option, you can apply from xunfei)  <br/>
                d) pi camera(search "pi camera" in taobao)  <br/>
   3) use Robi or "Rapiro" or UBTECH "Alpha 1S" for the servos & body.   <br/>
      if use Robi for RTP, you can choose these series: 1-23, 28-33, 36-38, 46-52.    <br/>
   4) battery & speaker: with Robi or Alpha 1s you already has the battery. you can also search them in taobao.     <br/>
      

2. Play Steps <br/>
1) connect robi servo's 'CTRL'(or 'Signal') together and tight them to Pi3's GPIO pin 8&9(TXD0&RXD0). Connect the GND of servo and Pi3.<br/>
2) I just connect the 5V from GPIO to servo VCC supply. The better way may be providing indepedent power supply to servo's VCC. <br/>
3) connect a speaker to Pi3's audio jack. <br/>
4) type ./startup.sh in a ternimal(before typing you need to copy the script to the directory you work) <br/>
5) input the commands printed by the "startup.sh" into the terminals opened by the startup script. <br/>
6) last command is: rosrun cortex cortexn.py  
   This means that the top brain of robi is running :)  


2016.11.29 <br/>

3. TODO:  <br/>
1.a) servo action server now only support 1 servo command during server period. Need to support all servos at the same time. <br/>
1.b) Robi cannot move right now. need to add a balance car under the legs and let them move: controlled by BT.  <br/>
1.c) integrate moveit! ? <br/>

2.a) sensors now are blank (except microphone and camera). Need to add sensors driver and integrate them into ROS to let them publish data. <br/>
2.b) integrate cartographer for SLAM <br/>

3.a) No object detection right now. Maybe "tensorbox" is a solution.  <br/>
3.b) Object recognization using "inception" has not been integrated into ROS. also face recognization. <br/>

4.a) When speaking the microphone also capture the sound and record it. This is stupid. Need a way to improve this.  <br/>
4.b) STT and TTS are not good. STT is using Xunfei cloud but too slow. TTS result is not smooth. Needs better solution.  <br/>
4.c) Integrate "syntaxnet" to do some analyzation?   <br/>
4.d) dialog match now using AIML. need to imporove using RNN/LSTM things???  <br/>



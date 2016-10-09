#!/usr/bin/env python
# ultrasonic to measure distance -2015.12.10
# PM2.5 sensor, motion sensor, geomagnetism sensor
# http://www.raspberrypi-spy.co.uk/archive/python/ultrasonic_1.py

import os, sys, time
#import RPi.GPIO as gpio
#import robiClib as robiC
#sys.path.append(/opt/ros/indigo/lib/python2.7/dist-packages)

import rospy
#from std_msgs.msg import String
from sensorprocess.msg import sensorsMsg

class sensors:
     def __init__(self):
          self.battery = 77
          self.mechanic = 77
          self.count = 77
          rospy.init_node('sens')
          self.pub = rospy.Publisher('tpc_sens', sensorsMsg, queue_size = 3 )
          self.rate = rospy.Rate(1)   

     def pub_sensors(self):
          self.pub.publish( sensorsMsg(None, self.battery, self.mechanic) )    #rospy will fill header automatically
          print ("Info: <pub_sensors> executed. battery is %d, mechanic is %d" %(self.battery, self.mechanic) )
          #rospy.spin( )
          
          
     def launch(self):
          while not rospy.is_shutdown():
               self.catchInput()
               self.pub_sensors( )               
               self.rate.sleep( )
     
     def update_accelerometer(self ):
          pass
     
     def update_compass(self ):
          pass

     def update_ultrasonic(self ):
          pass
     
     def update_infrareRay(self):
          pass
     
     def catchInput(self):          
          if self.count == 100:
               self.count = 0
          else:
               self.count += 1
               
          self.battery = self.count
          self.mechanic = self.count


"""
#add to dict sensors
sensors = {}
sensors['accelerometer'] =   accelerometer
sensors['compass']       =    compass
sensors['ultrasonic']    =    ultrasonic
sensors['infrareRay']    =    infrareRay
sensors['accelerometer'] =    accelerometer
"""

if __name__ == '__main__':
     sens = sensors()
     sens.launch()




#!/usr/bin/env python
#archipallium -2015.11.13
# as a insect

import rospy
#from std_msg.msg import String
from cortex.msg import cortexaMsg
from sensorprocess.msg import sensorsMsg

import threading
import time
#from sens import sensors as sens
#from mc import servoMove


class cortexa:
    def __init__(self):
        self.battery = 1
        self.mechanic  = 1                #  1 = mechanic2al hot, need rest
        self.selftest = 1                    #  'Pass'/ 'Subhealth': 70/ 'Fail'
        print ('initializing... battery =', self.battery)
        print ('initializing... mechanic =',  self.mechanic)
        rospy.init_node('cortexa')             #node name
        self.pub = rospy.Publisher("tpc_cortexa", cortexaMsg, queue_size = 10)     #(name, data_class, subscriber_listener=None, tcp_nodelay=False, latch=False, headers=None, queue_size=None)
        self.rate = rospy.Rate(1)    # 1Hz

    def pub_cortexa(self):
        self.pub.publish( cortexaMsg(None, self.battery, self.mechanic, self.selftest )  )  #'CORTEXA'
        print ('Info: <pub_cortexa> finished.'  )

    def sub_sensors(self):
        rospy.Subscriber('tpc_sens', sensorsMsg, self.callback_tpc_sens)
        #self.rate.sleep( ) 
        #rospy.spin( )

    def callback_tpc_sens(self, data):
        self.battery = data.battery
        self.mechanic = data.mechanic
        header = data.header
        timestamp = header.stamp.to_sec( )
        print rospy.get_caller_id(), header.seq, 'Info: cortexa heard that battery= %d mechanic = %d at %12f' %(self.battery,  self.mechanic, timestamp)
        
    def launch(self):
        while not rospy.is_shutdown():
            self.basicLifeCheck( )
            #rospy.loginfo('Info: cortexa running <launch>. pub info: ' )
            self.pub_cortexa( )
            self.sub_sensors( )
            self.rate.sleep( )                          #rospy.sleep( 0.1 )            
        
                
    def basicLifeCheck(self):
        self.sub_sensors( )
        
        if self.battery < 10:
            print ('Warning: battery < 10, i need to charge...')
        if self.mechanic > 90:
            print ('Warning: Mechanic hot!  > 90. need some rest....')

if __name__ == '__main__':
    toby = cortexa()
    toby.launch()


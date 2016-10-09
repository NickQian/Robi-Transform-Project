#!/usr/bin/env python
# neopallium -2015.11.13. for mem & forcast. This is the top brain.
# ??use conversation to improve speech recognization like what human do??

import sys, os
import rospy

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist    #TwistStamped/TwistWithCovariance/TwistWithCovarianceStamped | Vector3 | Wrench |Transform | Quaternion|Pose|Pose2D | Polygon | Point | Inertia |Accel
from sensor_msgs.msg import BatteryState, Imu, LaserScan, RegionOfInterest, Temperature, MagneticField, Range

from audioprocess.srv import *
from cortex.srv import cortexSrv
from visualprocess.msg import visualMsg
from sensorprocess.msg import sensorsMsg
from cortex.msg import cortexpMsg
import bt as btree

'''
import roslib
roslib.load_manifest('cortex')                 #load package
import actionlib
from actionprocess.action import talkGesture                     #import action file
'''

class cortexn(cortexa):
    '''
    Heart Of the Machine/me
    '''
    def __init__(self):
        
        # build the behavior tree
        bt = btree( )

        # Write back to black board
        bt.bb.achievementDesire = self.achievementDesire
        bt.bb.creativeDesire     = self.creativeDesire                                
        self.playDesire = bt.bb.playDesire                                                                 # 100: tedious
        self.energy = (self.battery + self.mechanic + self.playDesire + self.achievementDesire + self.creativeDesire + self.environ )/ 6  # 1-depressed;   50-mild   100-energy full

        # Fetch from BB
        self.age     = bt.bb.age
        self.name = bt.bb.name
        self.totalHealth = bt.bb.totalHealth
        self.personality = bt.bb.personality

        # Update from tpc
        self.battery      = None                           
        self.mechanic = None
        self.selftest     = None
        self.vtrack_once = None

        # flags for subscribers
        self.flg_sub_cortexp = None
        self.flg_sub_cortexa = None
        self.flg_sub_visual = None
        self.flg_sub_sens = None


        #Insts for interfaces
        self.actclient_cortexp = actionlib.SimpleActionClient('CortexpAction',  actPatternAction )  

        # Init node 
        rospy.init_node('cortexn')
        self.rate = rospy.Rate(2)        # 2Hz


        

    #------------------- launch ------------------
    def launch(self):
        while not rospy.is_shutdown():
            #self.sub_cortexa( )
            self.sub_visual( )
            #self.sub_cortexp( )
            #self.sub_sens( )
            
            bt.run()
            

    #-------------------------  (1): actclient to cortexp --------------------
    def actclient_req_cortexp( ):                   # send NEO-CMD to cortexp
        client = actionlib.SimpleActionClient('bt_act_client', DoDishesAction)
        client.wait_for_server( )
        client.send_goal(DoDishGoal(en_headtrack = True))
        client.wait_for_result(rospy.Duration.from_sec(5.0) )
        

    #-------------------------- (2): actclient to mouth -----------------------
    def actclient_req_mouth(cmd_brain, content, txt):      
        
        try:

            return resp.status
        except rospy.ServiceException, err:
            print "Warning: Service call failed: %s" %err
            
            
    #-------------------------- (3): subscriber to visual ---------------------
    def sub_visual( ):       
        rospy.Subscriber('tpc_visual', visualMsg, self.callback_tpc_visual )
        self.flg_sub_visual = False
        self.rate.sleep()
        
    
    def callback_tpc_visual(data):
        #objDetects = data.objDetects
        roi = data.roi
        motion_detected = data.motion_detected
        print ("******* get msg from tpc_visual. roi/motion_detected are:", roi, motion_detected)
        #peopleDetects = data.peopleDetects
        #peopleRecog    = data.peopleRecog
        #ooi
        #roi
        self.flg_sub_visual = True


    #-------------------------- (4): subscriber to cortexp --------------------
    def sub_cortexp():  
        rospy.Subscriber("tpc_cortexp", cortexpMsg, slef.callback_tpc_cortexp )
        
        self.flg_sub_cortexp = False
        self.rate.sleep( )
        

    def callback_tpc_cortexp(self,data):
        vtrackEvt = data.evt_visual_track
        if cortexpMsg.VTRACK_DONE == vtrackEvt:
            self.vtrack_once = True
            
            print ("====hahahaha ====")

        # Set the flag
        self.flg_sub_cortexp = True            
   

        
    #--------------------------  (5) ??? ------------------------------------
    def proxy_cortexp(self, cmd_neo, ):       
        rospy.wait_for_service('srv_cortexp')
        print ('Info: End <wait_for_service> in <client_cortexp_req>' )
        try:
            handle_neo = rospy.ServiceProxy('srv_cortexp', cortexpSrv)
            result = handle_neo(cmd_neo, queryData )                                # ?
            return result.result                                                                            # ?
        except rospy.ServiceException, err:
            print "Warning: Service call failed: %s" %err
            


    #--------------------------- (6) subscriber to sens ----------------------------
    def sub_sens(self):     
        rospy.Subscriber('tpc_sens', sensorsMsg, callback_tpc_sens)
        self.flg_sub_sens = False
        self.rate.sleep()

    def callback_tpc_sens(self, data):
        self.battery = data.battery
        self.mechanic = data.mechanic
        #header = data.header
        timestamp = header.stamp.to_sec( )
        # print rospy.get_caller_id(), header.seq, 'Info: cortexa heard that battery= %d mechanic = %d at %12f' %(self.battery,  self.mechanic, timestamp)

        self.flg_sub_sens = True



    #---------------------------- (7) subscriber to cortexa -------------------------
    def sub_cortexa(self):  
        rospy.Subscriber('tpc_cortexa', cortexaMsg, callback_tpc_cortexa)
        self.flg_sub_cortexa = False
        self.rate.sleep()
        

    def callback_tpc_cortexa(self, data):
        self.selftest   = data.selftest
        timestamp       = header.stamp.to_sec( )
        self.battery    = data.battery
        self.mechanic   = data.mechanic
        print (rospy.get_caller_id(), header.seq, 'Info: cortexa heard that selftest= %d  time: %12f' %(self.selftest, timestamp) )

        self.flg_sub_cortexa = True



        


    def basicLifeRun(self):
        if  self.energy < 20:
            print ('Info: Energy < 20, i\'m feeling depressing... let me check...')
            client_mouth_req()
        elif self.energy > 90:
            print ('AAAAAA...refreshed!!! I am feeling good! ')
        else:
           print ('basicLifeCheck: totalHealth = ', cortexa.energy )

        print ('cortexa.battery = ', cortexa.battery)
            
        if (cortexa.behavior != 'playing'):
            cortexa.play -= 1
        else:
            cortexa.play += 1

        if (cortexa.behavior =='playing') | (cortexa.behavior =='walking'):
            cortexa.mechanic -=1
        else:
            cortexa.mechanic +=1
            
        if (cortexa.behavior =='watching') |(cortexa.behavior =='talking') :
            cortexa.creative +=1
        elif (cortexa.behavior =='idle'):
            cortexa.creative -=1
        else:
            pass            

        t1_tickle = threading.Timer(2.0, self.basicLifeRun )
        t1_tickle.start()

        print ('---->')
        cortexa.energy = (cortexa.battery + cortexa.mechanic + cortexa.play + cortexa.achievement + cortexa.creative + cortexa.environ )/ 6      
        cortexa.count += 2
        servoMove(cortexa.count)
        print ('cortexa.energy = ', cortexa.energy)
        self.basicLifeCheck()


    
class meetPerson:
    cvst[]   #conversation[]
    mem[]
    
    def __init__(self, face):        
        if faceMatch(face, nameOut) == True:
            self.personName = nameOut
    def recognizePerson(self, face):
        self.name = opencv(face)
        return self.name
    def newPerson(self, ):
        pass
    
    def sayHi(self):
        if recognizePerson(self, face) is not 0:
            print ('Hi, my old friend, %s' %self.name)
        else
            newPerson()



def main():
    crn = cortexn()
    crn.launch()


if __name__ == '__main__':
    main()
    
    


'''
Interrupts(Timer):
timer 1: 1s heart beat (check self status. See something? hear something?)
timer 2: ?

使用hardirq
http://blog.csdn.net/zhangskd/article/details/21992933

在include/linux/sched.h里声明。
 request_irq()调用的定义：
int request_irq(unsigned int irq,
 void (*handler)(int irq, void *dev_id, struct pt_regs *regs),
 unsigned long irqflags,
 const char * devname,
 oid *dev_id  );
 irq:         要申请的硬件中断号。在Intel平台，范围是0～15。
 handler: 向系统登记的中断处理函数。这是一个回调函数，中断发生时，系统掉用这个函数，传入的参数包
 括硬件中断号,device id,寄存器值。dev_id就是下面的request_irq时传递给系统的参数dev_id。
 irqflags: 中断处理的一些属性。比较重要的有SA_INTERRUPT,标明中断处理程序是快速处理程序（设置
 SA_INTERRUPT）还是慢速处理程序（不设置SA_INTERRUPT）。快速处理程序被调用时屏蔽
 所有中断。慢速处理程序不屏蔽。还有一个 SA_SHIRQ属性，设置了以后运行多个设备共享中
 断。
 dev_id:  中断共享时会用到。一般设置为这个设备的device结构本身或者NULL。中断处理程序可以用
 dev_id找到相应的控制这个中断的设备，或者用irq2dev_map找到中断对应的设备。
 
 
void free_irq(unsigned int irq, void *dev_id):
 不再使用已注册的中断服务时，使用 free_irq() 函数将其从内核注销掉。该函数在 2.4 内核和 2.6
 内核中原型相同:
#include <linux/interrupt.h>
                       void free_irq (unsigned int irq, void *dev_id);
 irq       :   是将要注销掉的中断服务函数的中断号；
 dev_id:  指定与request_irq() 函数中使用的 dev_id 值相同的值。

'''

'''
Soft Interrupts:
1. Action block
2. Sensor abnormal data
3.
'''

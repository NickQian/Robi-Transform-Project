#motion control (motor control board with BLE) -2015.11.23
#PWM period =20ms(50Hz), pulse =1ms-2ms->0-180  1.5ms=center
''' Inputs: Action comands
--- Outputs:N Channels Motion controls to motors
'''

import robiClib as robiC
import os, sys


def moveForward(self, speed, distance):
    print ('moving forward...')
def moveBack(self, speed, distance):
    print ('moving back...')
def turnLeft (self, speed, distance):
    print ('turning left...')
def turnRight(self, speed, amp):
    print ('turning right')

'''-----------Hands--------------'''
def armlUp(self, speed, amp):
    print ('left hand uping...speed =s%', speed)
    raise NotImplementedError
  
def armlDown(self, speed, amp):
    print ('left hand downing...speed =s%', speed)
    raise NotImplementedError
  
def armrUp(self, speed, amp):
    print ('right hand uping...speed =s%', speed)
    raise NotImplementedError
  
def armrDown(self, speed, amp):
    print ('right hand downing...speed =s%', speed)
def armlLeft(self, speed, amp):
    print ('left hand uping...speed =s%', speed)
def armlRight(self, speed, amp):
    print ('left hand downing...speed =s%', speed)
def armrLeft(self, speed, amp):
    print ('right hand uping...speed =s%', speed)
def armrRight(self, speed, amp):
    print ('right hand turning right...speed =s%', speed)
def gestureHi():
    pass
def gestureNo():
    pass
def gestureYes():
    pass
def gestureGreat():
    pass
def gestureOK():
    pass 
def handlCatch(self, speed, amp):
    print ('left hand catching...speed =s%', speed)
def handlRelease(self, speed, amp):
    print ('left hand releasing...')
def handrCatch(self, speed, amp):
    print ('right hand catching...')
def handrRelease(self, speed, amp):
    print ('right hand releasing...')
    
'''--------------Servo---------------'''
def servoMove (ID, pos = 45, tim=1000):
    position = pos * 10
    time = tim / 10                          #tim is "ms"
    

    robiC.servoSet(ID, position, time )      # 0 time means no time req

if __name__ ==  '__main__':
 
    os.system('omxplayer -o local boring.wav')

    robiC.peri_Init(19200, 115000, 19200)
    #robiC.servoSetID(4)

    servoMove(1, 45, 300)
    servoMove(2, 40, 1000)
    servoMove(4, 90, 600)

    servoMove(1, -45, 2000)
    servoMove(2, 45,  300)
    servoMove(4, 60,  1000)

    servoMove(1, 0,   500)
    servoMove(2, 0,   2000)
    servoMove(4, 0,   2000)

    os.system('omxplayer -o local boring.wav')

    sys.exit()   

    print "|||||||ENDED||||||. Check the fucking status."

#!/usr/bin/env python3

#scp pi@10.42.0.2:/home/pi/catkin_ws/src/orientation/src/send_move_lidar.py /home/oskar/Desktop/send_move_lidar.py
#scp /home/oskar/Desktop/scripte_bachelor/Code/lm1.py pi@10.42.0.3:/home/pi/catkin_ws/src/orientation/src/

import time
import math
import sys,os
import json
import subprocess
import rospy
from geometry_msgs.msg import PointStamped
from datetime import datetime

print("Mouseread")

#----------------------
def onFailedStart(myNo):
    print 
    print('Mouse %d data access failed. Few tips:'%myNo)
    task = subprocess.Popen("cat /proc/bus/input/devices", shell=True, stdout=subprocess.PIPE)
    data = task.stdout.read()
    assert task.wait() == 0
    m=0
    for line in data.decode('utf-8').split("\n"):
        #    print line
        if "H: Handlers=mouse" in line:
            #print line
            lL=line.split()
            #print lL[1]
            msNo=int(lL[1][-1])
            devName='/dev/input/mouse%d'%msNo
            #MOUSEFILE "/dev/input/mouse0\0" 
            stats = os.stat(devName)

            print ('see: ',devName)
            #print( 'access=',oct(stats.st_mode& 0777))

    print(' try:   sudo chmod a+r ',devName)
    print
    print('use:  mouseMsg.py [mouseNo] [rate/Hz]')

def callback():
    pub = rospy.Publisher('m_1', PointStamped, queue_size = 10)
    print('Maus 1 wurde gestartet#####################################################')
    my_position = PointStamped()
    
    #rospy.Rate(1)
    myName='devMouse0' 
  
    xpos = 0
    ypos = 0
    newypos = 0
    newxpos = 0

    counter = 0
  
    while not rospy.is_shutdown():
        
        state = ord(fm.read(1))
        dx = ord(fm.read(1))
        dy = ord(fm.read(1))

        #convert bits in 'state' in to an array 'mouse_state'
        mouse_state = [(state & (1 << i)) >> i for i in range(8)]

        # if mouse moving to the left. dx must be a negative value
        if mouse_state[4] == 1:
            dx = dx - 256    # convert 2's complement negative value

        # if mouse moving down
        if mouse_state[5] == 1:
            dy = dy - 256

        if (mouse_state[6] == 1) or (mouse_state[7]==1):
            print("Overflow!")

        # update the position
        xpos += dx
        ypos += dy
        newypos = ypos * 0.0023067599684640262
        #value from first measurement 0.0084381727
        newxpos = xpos * 0.0022384822182865763

        #print(xpos, ypos, newxpos, newypos)
   
        my_position.point.x = newxpos
        my_position.point.y = newypos
        my_position.point.z = 0
        '''
        nsecs, secs = math.modf(time.time())
        my_position.header.stamp.secs = int(secs)
        my_position.header.stamp.nsecs = int(nsecs * 1000000000)
        '''
        my_position.header.stamp = rospy.Time.now()

        counter += 1
        if counter == 8:

            pub.publish(my_position)
            counter = 0

if __name__ == "__main__":
    rospy.init_node('m_1_publisher')
    myNo = 0
    devName='/dev/input/mouse%d'%myNo
    
    fm = open(devName, 'rb')
    callback()

    


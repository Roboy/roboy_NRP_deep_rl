#! /usr/bin/env python2.7

import sys
import rospy
import threading
import time

from std_msgs.msg import Float64

rospy.init_node('sendactivation', anonymous=True)

topic = sys.argv[1]
activation = float(sys.argv[2])

pub = rospy.Publisher('/gazebo_muscle_interface/'+topic+'/cmd_activation', Float64, queue_size=1, latch = True)
pub.publish(Float64(data=activation))

time.sleep(0.5)
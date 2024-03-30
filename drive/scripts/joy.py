#!/usr/bin/env python3

import math
import rospy
from sensor_msgs.msg import Joy
from drive.msg import IROC

# mode 1 - 50, mode 2 - 100, mode 3 - 150, mode 4 - 200, mode 5 - 255

drive_mode = 1

def joy_callback(msg):
   
   iroc = IROC()
   global drive_mode

   drive_axes = [msg.axes[i] for i in range(0,2)]
   arm_axes = [msg.axes[i] for i in range(2,4)]
   mode = [msg.buttons[i] for i in range(6,8)]

   for id in range(len(mode)):
      if mode[id] == 1 and id == 0 and drive_mode > 1:
         drive_mode -=1
      elif mode[id] == 1 and id == 1 and drive_mode < 5:
         drive_mode += 1
      
   drive_axes = [i*drive_mode*255/5 for i in drive_axes]

   iroc.driveName = "IROC Drive"
   iroc.drivecommands = drive_axes
   iroc.driveMode = drive_mode
   iroc.armName = "IROC Arm"
   iroc.armcommands = arm_axes
   pub.publish(iroc)
   
if __name__ == "__main__":

   rospy.init_node("IROCdrive", anonymous=False)
   rospy.loginfo("Running JoyStick Interface !!!")

   pub = rospy.Publisher("/iroc/joy", IROC, queue_size=10)
   rospy.Subscriber('joy', Joy, joy_callback)

   rospy.spin()
#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import time


class SavePoses(object):
    def __init__(self):
        
        self._pose = Pose()
        self._pose_sub = rospy.Subscriber('/aft_mapped_to_init', Odometry , self.sub_callback)
        self.write_to_file()

    def sub_callback(self, msg):
        
        self._pose = msg.pose.pose
        rospy.loginfo("Written all Poses to poses.txt file")
        with open('ALOAM.txt', 'a') as file:

            file.write( str(msg.header.stamp.secs)  + ' '+ str(self._pose.position.x) + ' '+ str(self._pose.position.y) + ' '+ str(self._pose.position.z) +
            ' '+str(self._pose.orientation.x) +' '+ str(self._pose.orientation.y) +' ' + str(self._pose.orientation.z) + ' ' + str(self._pose.orientation.w) +'\n')

    def write_to_file(self):
        pass
        

if __name__ == "__main__":
    rospy.init_node('spot_recorder', log_level=rospy.INFO) 
    save_spots_object = SavePoses()
    rospy.spin() # mantain the service open.

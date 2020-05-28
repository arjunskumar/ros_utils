#!/usr/bin/env python
import rospy 

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class OdomPathGenerator():
    def __init__(self):
        rospy.init_node('OdomPathGenerator', anonymous = False)
        
        rospy.loginfo('Press CTRL + C to stop the node ')

        self.rate = rospy.Rate(10)

        self.odometry_susbcriber = rospy.Subscriber('/vins_estimator/odometry', Odometry, self.odom_callback)

        self.path_publisher = rospy.Publisher('odom_path', Path, queue_size=10)

        self.path = Path()

        rospy.on_shutdown(self.shutdown)
       
    def odom_callback(self, data):

        self.path.header = data.header
        pose = PoseStamped()
        pose.header = data.header
        pose.pose = data.pose.pose
        self.path.poses.append(pose)
        
        self.path_publisher.publish(self.path)
        self.rate.sleep()

    def shutdown(self):
        """ Safe shutdown the node """
        rospy.loginfo('Closing the node')
        self.path_publisher.publish(Path())
        rospy.sleep(1)

if __name__== '__main__':
    try:
        OdomPathGenerator()
        rospy.spin()
    except:
        rospy.loginfo("End of the node." )
#!/usr/bin/python
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class ProtectionZoneMonitor():
    def __init__(self):
        self.protection_range = 1.0
        self.protection_behavior_engaged = False
        rospy.Subscriber('scan_filtered', LaserScan, self.scan_callback)
        self.pause_pub = rospy.Publisher('move_base_simple/pause', Bool, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    def scan_callback(self, scan):
        scan_length = len(scan.ranges)
        count = 0 
        for i in scan.ranges:
            if i <= self.protection_range:
                count += 1
        if count > 1:
            rospy.logwarn("Found an obstacle in the protection zone. PAUSING ROBOT TILL OBSTACLE CLEARS!")

            msg = Bool()
            msg.data = True
            self.pause_pub.publish(msg)

            twist = Twist()
            self.cmd_vel_pub.publish(twist)

            self.protection_behavior_engaged = True
        else:
            if (self.protection_behavior_engaged):
                rospy.logwarn("Good to release protection behavior")

                msg = Bool()
                msg.data = False
                self.pause_pub.publish(msg)
                self.protection_behavior_engaged = False

if __name__=="__main__":
    rospy.init_node('protection_zone_monitor')
    prot = ProtectionZoneMonitor()
    rospy.spin()
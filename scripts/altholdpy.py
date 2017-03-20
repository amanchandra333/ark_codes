#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Empty
from ardrone_autonomy.msg import Navdata
from geometry_msgs.msg import Twist

vel = Twist()

def callback(msg):
    alt=msg.altd

def althold():
    pub = rospy.Publisher("/cmd_vel" Twist, queue_size=1000)
    rospy.Subscriber("/ardrone/navdata", Navdata, callback)
    rospy.init_node('althold', anonymous=True)
    rate = rospy.Rate(10)
    rospy.spin()
    while not rospy.is_shutdown():
        pub.publish(vel)
        rate.sleep()

if __name__ == '__main__':
    try:
        althold()
    except rospy.ROSInterruptException:
        pass

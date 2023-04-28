#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

mambo_new_twist = Twist()


def cb_getTwist(msg):
    global mambo_new_twist
    mambo_new_twist.linear.x = 0
    mambo_new_twist.linear.y = 5.0 * msg.linear.x
    mambo_new_twist.angular.z = -msg.angular.z


def init():
    rospy.init_node('mambo_twist_transform', anonymous=True)
    rospy.Subscriber("/mambo/cmd_vel", Twist, cb_getTwist)
    pub = rospy.Publisher("/tello/cmd_vel", Twist, queue_size=10)
    rate = rospy.Rate(15)
    rospy.loginfo("Changing Direction [ACTIVE]")
    while not rospy.is_shutdown():
        pub.publish(mambo_new_twist)
        rate.sleep()


if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass

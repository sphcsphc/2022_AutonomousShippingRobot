#!/home/scout/.pyenv/versions/rospy368/bin/python
# -- coding: utf-8 --

import rospy
from std_msgs.msg import String


def main():
    rospy.init_node('print_system')
    publisher = rospy.Publisher('which_print', String, queue_size=1)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        key = input("DWA print call:")
        publisher.publish(key)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

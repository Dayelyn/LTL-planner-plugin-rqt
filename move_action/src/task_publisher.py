#!/usr/bin/env python

import rospy
import rospkg
import yaml
import os
from std_msgs.msg import String


def task_publisher(task_input):
    pub = rospy.Publisher('ltl_task', String, queue_size=10)
    rospy.init_node('task_publisher', anonymous=False)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(task_input)
        rate.sleep()


if __name__ == '__main__':
    rp = rospkg.RosPack()
    try:
        try:
            with open(os.path.join(rp.get_path('rqt_ltl'), 'map/', 'task.yaml'), 'r') as streamr:
                load = yaml.load(streamr)
            task_publisher(load.get('total_task'))
        except IOError:
            rospy.logerr('Missing rqt_ltl/map/task.yaml')

    except rospy.ROSInterruptException:
        rospy.loginfo('No task is being published!')

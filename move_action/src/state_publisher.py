#!/usr/bin/env python

import rospy
from move_action.srv import MoveAct

'''
    This node is simply designed for testing the call of services. It will be substituted by the GUI in the future
'''


def srv_move_base_action_client(pose_mode):
    try:
        rospy.wait_for_service('move_base_action_service')
        param_pass = rospy.ServiceProxy('move_base_action_service', MoveAct)
        info = param_pass(pose_mode)
        return info.result

    except rospy.ServiceException:
        print "Service call failed"


if __name__ == '__main__':
    rospy.init_node('action_client')
    move_result = srv_move_base_action_client('PoseQueue')
    while not rospy.is_shutdown():
        if move_result == "Navigation finished!":
            rospy.loginfo(move_result)
            break
        else:
            continue










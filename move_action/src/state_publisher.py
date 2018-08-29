#!/usr/bin/env python

import rospy
from move_action.srv import MoveAct, MoveActResponse

'''
    This node is simply designed for testing the call of services. It will be substituted by the GUI in the future
'''


def srv_move_base_action_client(pose_mode):
    try:
        rospy.wait_for_service('move_base_action_service')
        move_action = rospy.ServiceProxy('move_base_action_service', MoveAct)
        info = move_action(pose_mode)
        return info

    except rospy.ServiceException:
        print "Service call failed"


if __name__ == '__main__':
    rospy.init_node('action_client')
    move_result = srv_move_base_action_client('PlanSynthesis')
    while not rospy.is_shutdown():
        if move_result.result == "Navigation finished!":
            rospy.loginfo(move_result)
            break
        else:
            continue










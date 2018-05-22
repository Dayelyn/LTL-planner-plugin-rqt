#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_action.srv import NextMove, MoveAct, MoveActResponse
# srv generate Response, need to be included


def act_move_base_client(pose_input, index):
    '''
    Action client of move_base. Send goal pose to MoveBaseAction to move the robot.
    :param pose_input: Pose sequence(list) from plan_service, Pose[](Point(x, y, z), Quaternion(x, y, z, w))
    :param index: Sequence(list) index of poses
    :return: The result of MoveBaseAction
    '''

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = pose_input[index]

    client.send_goal(goal)
    client.wait_for_result()
    # Follow the instruction on ROS wiki of simple action client

    return client.get_result()


def srv_next_move_planner_client(pose_mode):
    '''
    The client of next_move_planner service(plan_service), request a planned pose sequence
    :param pose_mode: no special use
    :return: a pose sequence after LTL planning
    '''

    try:
        rospy.wait_for_service('next_move_planner_service')
        pose_loader = rospy.ServiceProxy('next_move_planner_service', NextMove)
        pose_list = pose_loader(pose_mode)
        return pose_list.poses

    except rospy.ServiceException:
        print "Service call failed"


def move_base_action(req):
    '''
    The function deals with the request of move_base_action_server()
    :param req: string status
    :return: info: string result
    '''

    try:
        pose_list = srv_next_move_planner_client(req.status)
        index = 0

        while not rospy.is_shutdown():

            result = act_move_base_client(pose_list, index)

            if result and (index <= len(pose_list) - 2):
                # Here we need a shorter length of pose_list. It is defined in P_MAG_TS package
                rospy.loginfo("Goal[%s] execution done!", index)
                index = index + 1
                # Move index to get pose value in pose sequence
            else:
                info = "Navigation finished!"
                rospy.loginfo(info)
                return MoveActResponse(info)

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation interrupted.")


def move_base_action_server():
    rospy.init_node('move_base_action_server')
    rospy.Service('move_base_action_service', MoveAct, move_base_action)
    rospy.spin()


if __name__ == '__main__':
    move_base_action_server()


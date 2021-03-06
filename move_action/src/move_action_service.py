#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_action.srv import NextMove, MoveAct, MoveActResponse
# srv generate Response, need to be included


def act_move_base_client(next_move):
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
    goal.target_pose.pose = next_move.pose

    flag = rospy.get_param('urgent_task')

    if flag == 0:
        client.send_goal(goal)
    else:
        client.cancel_goal()

    client.wait_for_result()
    # Follow the instruction on ROS wiki of simple action client
    return client.get_result()


def srv_next_move_planner_client(mode):
    '''
    The client of next_move_planner service(plan_service), request a planned pose sequence
    :param pose_mode: no special use
    :return: a pose sequence after LTL planning
    '''

    try:
        rospy.wait_for_service('next_move_planner_service')
        pose_loader = rospy.ServiceProxy('next_move_planner_service', NextMove)
        next_move = pose_loader(mode)
        return next_move

    except rospy.ServiceException:
        print "Service call failed"


def move_base_action(req):
    '''
    The function deals with the request of move_base_action_server()
    :param req: string status
    :return: info: string result
             rest_goals: geometry_msgs/Pose[] rest_goals
    '''

    try:
        next_move = srv_next_move_planner_client('PlanSynthesis')
        index = 0
        first_move = True

        while not rospy.is_shutdown():
            result = act_move_base_client(next_move)
            if first_move:
                next_move = srv_next_move_planner_client('FirstMove')
                first_move = False
            else:
                if result:
                    #index += 1
                    #rospy.loginfo("Goal[%s] execution done!", index)
                    current_move = next_move
                    next_move = srv_next_move_planner_client('NextMove')
                    print '@@@@@@@@@@'
                    print next_move
                    print '@@@@@@@@@@'
                    print (next_move == current_move)

                    if next_move == current_move:
                        index +=1
                    if index >=3:
                        rospy.set_param('finish_task', 1)
                        index = 0
                        info = "Navigation finished!"
                        rospy.loginfo(info)
                        return MoveActResponse(info)
                else:
                    info = "Navigation interrupt!"
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


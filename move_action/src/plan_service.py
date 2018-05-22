#!/usr/bin/env python

from P_MAS_TG.ts import MotionFts, ActionModel, MotActModel
from P_MAS_TG.planner import ltl_planner

from move_action.srv import NextMove, NextMoveResponse
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import String

import rospy
import rospkg
import time
import yaml
import os


def task_callback(total_task):
    '''
    Subscriber callback of task_publisher. Parse task into hard and soft part
    :param total_task: hard_task + ',' + soft_task
    :return: hard_task, soft_task
    '''
    global hard_task, soft_task
    task_list = total_task.data.split(',')
    # Follow the definition in task_publisher
    hard_task = task_list[0]
    soft_task = task_list[1]


def pose_callback(pose_data):
    '''
    Subscriber callback of amcl_pose. Call amcl to get current estimated pose as initial pose
    :param pose_data: returned pose from amcl, Pose(Point(x, y, z), Quaternion(x, y, z, w))
    :return: current_pose = (x, y, yaw)
    '''
    global current_pose
    header = pose_data.header
    pose = pose_data.pose
    if (not current_pose[0]) or (header.stamp > current_pose[0]):
        # Always return the newest pose
        current_pose[0] = header.stamp
        euler_angle = euler_from_quaternion([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
        # Roll pitch yaw
        current_pose[1] = (pose.pose.position.x, pose.pose.position.x, euler_angle[2])
    return False


def next_move_plan(req):
    '''
    The function deals with the request of next_move_planner_server()
    :param req: string status
    :return: poseseq.poses = PoseArray[], contains the sequence of planned poses
    '''
    rospy.Subscriber('ltl_task', String, task_callback, queue_size=1, buff_size=2**24)
    # Subscribe task_publisher to get LTL task

    rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, pose_callback, queue_size=1, buff_size=2**24)
    # Subscribe amcl to get current estimated pose as initial pose. Planning will be given based on initial pose
    # Subscribing needs time and next sentences will run first. Hence, here needs a delay.

    rospy.sleep(1.0)

    robot_motion.set_initial(current_pose[1])
    robot_model = MotActModel(robot_motion, robot_action)
    robot_planner = ltl_planner(robot_model, hard_task, soft_task)
    start = time.time()
    robot_planner.optimal(10, 'static')
    # Synthesize motion FTS, action model and task to plan a pose sequence

    if req.status == 'PoseQueue':
        poseseq = PoseArray()
        poseseq.header.frame_id = "map"
        poseseq.header.stamp = rospy.Time.now()

        for index in range(len(robot_planner.run.pre_plan) - 2):
            qua_angle = quaternion_from_euler(0, 0, robot_planner.run.pre_plan[index][2], 'rxyz')
            poseseq.poses.append(Pose(Point(robot_planner.run.pre_plan[index][0], robot_planner.run.pre_plan[index][1], 0.000), Quaternion(qua_angle[0], qua_angle[1], qua_angle[2], qua_angle[3])))
            # Here can not directly use qua_angle value cuz it is not a quaternion type but only a tuple type
            # Data structure is given in P_MAG_TS, please read the instruction in the package
	    print poseseq.poses
        return NextMoveResponse(poseseq.poses)
    else:
        print 'No acceptable move'
        return None


def next_move_planner_server():
    rospy.init_node('next_move_planner_server')
    rospy.Service('next_move_planner_service', NextMove, next_move_plan)
    rospy.spin()


if __name__ == "__main__":
    rp = rospkg.RosPack()
    try:
        with open(os.path.join(rp.get_path('rqt_ltl'), 'map/', 'task.yaml'), 'r') as streamr:
            load = yaml.load(streamr)
        map_name = load.get('map_name')
        start = load.get('start_point')

        with open(os.path.join(rp.get_path('rqt_ltl'), 'map/' + map_name, 'data.yaml'), 'r') as streamr:
            load = yaml.load(streamr)

        regions = load.get('regions')
        edges = load.get('edges')
        ap = load.get('ap')
        action = load.get('action')
        #start = regions.keys()[regions.values().index(set(['r1']))]

        robot_motion = MotionFts(regions, ap, 'map')
        robot_motion.add_un_edges(edges, unit_cost=0.1)
        robot_action = ActionModel(action)
        init_pose = PoseWithCovarianceStamped()
        current_pose = [init_pose.header.stamp, [0, 0, 1]]
        hard_task = ''
        soft_task = ''

    except IOError:
        print 'Missing .yaml, open /hotel instead!'
        with open(os.path.join(rp.get_path('rqt_ltl'), 'map/hotel', 'data.yaml'), 'r') as streamr:
            load = yaml.load(streamr)

        regions = load.get('regions')
        edges = load.get('edges')
        ap = load.get('ap')
        action = load.get('action')
        start = regions.keys()[regions.values().index(set(['r1']))]

        robot_motion = MotionFts(regions, ap, 'map')
        robot_motion.add_un_edges(edges, unit_cost=0.1)
        robot_action = ActionModel(action)
        init_pose = PoseWithCovarianceStamped()
        current_pose = [init_pose.header.stamp, [0, 0, 1]]
        hard_task = ''
        soft_task = ''

    next_move_planner_server()

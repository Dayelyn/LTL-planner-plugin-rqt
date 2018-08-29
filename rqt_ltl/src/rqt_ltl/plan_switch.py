#!/usr/bin/env python

import rospy
import rospkg
import roslaunch
import yaml
import os


def urgent_task_execution(urgent_task):
    rp = rospkg.RosPack()
    try:
        with open(os.path.join(rp.get_path('rqt_ltl'), 'map', 'task.yaml'),'r') as streamr:
            temp_task = yaml.load(streamr)
            temp_task['total_task'] = urgent_task + ',' + ''

        with open(os.path.join(rp.get_path('rqt_ltl'), 'map', 'task.yaml'),'w') as streamw:
            yaml.dump(temp_task, streamw, default_flow_style=False, allow_unicode=False)

        os.system('rosnode kill task_publisher')
        # Respawn task_publisher to publisher changed task

    except IOError:
        print 'Missing task.yaml.'


def urgent_task_return(original_task):
    rp = rospkg.RosPack()
    try:
        with open(os.path.join(rp.get_path('rqt_ltl'), 'map', 'task.yaml'), 'r') as streamr:
            temp_task = yaml.load(streamr)
            temp_task['total_task'] = original_task + ',' + ''

        with open(os.path.join(rp.get_path('rqt_ltl'), 'map', 'task.yaml'), 'w') as streamw:
            yaml.dump(temp_task, streamw, default_flow_style=False, allow_unicode=False)

        os.system('rosnode kill task_publisher')

    except IOError:
        print 'Missing task.yaml.'


if __name__ == '__main__':
    rospy.init_node('plan_switcher', anonymous=True)
    rp = rospkg.RosPack()
    while not rospy.is_shutdown():
        u_task_finished = rospy.get_param('finish_task')
        if u_task_finished:
            rospy.set_param('finish_task', 0)
            original_task = rospy.get_param('o_task')
            try:
                with open(os.path.join(rp.get_path('rqt_ltl'), 'map', 'task.yaml'), 'r') as streamr:
                    temp_task = yaml.load(streamr)
                    temp_task['total_task'] = original_task + ',' + ''

                with open(os.path.join(rp.get_path('rqt_ltl'), 'map', 'task.yaml'), 'w') as streamw:
                    yaml.dump(temp_task, streamw, default_flow_style=False, allow_unicode=False)

                os.system('rosnode kill task_publisher')
                # Respawn task_publisher to publisher changed task

                package = 'move_action'
                executable = 'state_publisher.py'
                node = roslaunch.core.Node(package, executable)

                launch = roslaunch.scriptapi.ROSLaunch()
                launch.start()
                process = launch.launch(node)
                print 'task switched!'

            except IOError:
                print 'Missing task.yaml.'


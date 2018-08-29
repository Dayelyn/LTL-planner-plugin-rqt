# Need to install PyYaml and libyaml to load and dump .yaml files
import os
import sys
import pxssh

import rospkg
import rospy
import roslaunch
import actionlib_msgs.msg

import yaml
import ltl_mapviewer_widget
import ltl_mapdefine_widget
from plan_switch import urgent_task_execution, urgent_task_return

from python_qt_binding import loadUi
from python_qt_binding.QtCore import *
from python_qt_binding.QtGui import *
from python_qt_binding.QtWidgets import QWidget

from P_MAS_TG.ts import MotionFts, ActionModel, MotActModel
from P_MAS_TG.planner import ltl_planner


class LTLPlanNavWidget(QWidget):

    def __init__(self):

        super(LTLPlanNavWidget, self).__init__()

        self.rp = rospkg.RosPack()
        ui_file = os.path.join(self.rp.get_path('rqt_ltl'), 'resource', 'ltl_widget.ui')
        loadUi(ui_file, self)

        self.s = pxssh.pxssh()
        self.remote = None

        self.MapWidget = ltl_mapviewer_widget.MplWidget()
        self.MapDefineWidget = ltl_mapdefine_widget.MouseTracker()
        self._ui_init()
        
        self.map_name = []
        self.load = None
        self.regions = dict()
        self.ap = set()
        self.edges = []
        self.action = dict()
        self.origin = []
        self.map_scale = 0
        self.hard_task = None

        self.node = None
        self.launch = None
        self.process = None
        self.publisher = None
        self.temp_task = None

        # self.onlyDouble = QDoubleValidator()
        # self.ScaleInput.setValidator(self.onlyDouble)
        self.DefaultButton.clicked.connect(self._default_setting)
        self.MapSelection.activated.connect(self._map_selection)
        self.PlanExample.activated.connect(self._task_examples)
        self.InitPosButton.clicked.connect(lambda: self._init_robot(self.map_name))
        self.PlanSynButton.clicked.connect(lambda: self._plan_syn(self.map_name, self.ap, self.regions, self.edges, self.action, self.hard_task))
        # https://stackoverflow.com/questions/28575034/connecting-to-a-function-with-arguments
        self.PlanExeButton.clicked.connect(self._plan_exe)
        self.MapButton.clicked.connect(self._map_define)
        self.StopButton.clicked.connect(self._stop_plan)
        self.NavButtion.clicked.connect(self._navigation)
        self.UTaskExecuteButton.clicked.connect(self._urgent_task_execution)
        self.UTaskReturnButton.clicked.connect(self._urgent_task_return)

    def _urgent_task_execution(self):
        if not self.UTaskInput.text():
            sys.stdout = EmittingStream(textWritten=self.text_written)
            print 'No urgent task needs to be planned.'
            sys.stdout = sys.__stdout__
            pass
        else:
            self._stop_plan()
            try:
                urgent_task_execution(self.UTaskInput.text())
                rospy.set_param('u_task', self.UTaskInput.text())
                rospy.set_param('o_task', self.TaskInputR1.text())
                os.system('rosnode kill task_publisher')
                # Respawn task_publisher to publisher changed task

                self._navigation()
            except IOError:
                sys.stdout = EmittingStream(textWritten=self.text_written)
                print 'Missing task.yaml.'
                sys.stdout = sys.__stdout__

    def _urgent_task_return(self):
        self._stop_plan()
        try:
            urgent_task_return(self.TaskInputR1.text())
            rospy.set_param('finish_task', 0)
            os.system('rosnode kill task_publisher')
            self._navigation()
        except IOError:
            sys.stdout = EmittingStream(textWritten=self.text_written)
            print 'Missing task.yaml.'
            sys.stdout = sys.__stdout__

    def _map_define(self):
        self.MapDefineWidget.scene.clear()
        self.MapDefineWidget.openMap()
        self.MapDefineWidget.show()

    def _map_selection(self):
        self.MapDefineWidget.close()

        if self.MapSelection.currentIndex():
            self.map_name = self.MapSelection.currentText()

            map_path = os.path.join(self.rp.get_path('rqt_ltl'), 'map/' + self.map_name, 'map.png')

            self.MapDefineWidget.getMapName(self.map_name)
            self.loadYaml()
            self.MapWidget.getMapParam(map_path, self.map_scale, self.origin, self.load.get('drawing'), self.load.get('graph'))
            self.MapWidget.showMap()

        else:
            self.PlanViewer.clear()
            self.PlanViewer.append('Please select a map')
            self.MapWidget.clearMap()
            self.Scalelabel.clear()
            self.Originlabel.clear()
            self.map_name = []

        return self.map_name

    def loadYaml(self):
        try:
            with open(os.path.join(self.rp.get_path('rqt_ltl'), 'map/' + self.map_name, 'map.yaml'), 'r') as streamr:
                self.load = yaml.load(streamr)
            self.map_scale = self.load.get('resolution')
            self.origin = self.load.get('origin')
            self.Scalelabel.setText(str(self.map_scale))
            self.Originlabel.setText(str(self.origin))

            with open(os.path.join(self.rp.get_path('rqt_ltl'), 'map/' + self.map_name, 'data.yaml'), 'r') as streamr:
                self.load = yaml.load(streamr)

            self.regions = self.load.get('regions')
            self.edges = self.load.get('edges')
            self.ap = self.load.get('ap')
            self.action = self.load.get('action')

            sys.stdout = EmittingStream(textWritten=self.text_written)
            sys.stdout = sys.__stdout__

        except IOError:
            pass

    def _task_examples(self):

        task_exp = [
            '(([]<> r2) && ([]<> r4))',
            '(<> (r3 && <>r5))'
            #'(<> (r1 && <>(r2 && <> r3)))'
        ]

        idx = self.PlanExample.currentIndex() - 1

        if idx == -1:
            self.TaskInputR1.clear()
            self.PlanViewer.clear()
            self.PlanViewer.append('Please select a example task')

        elif idx == 2:
            self.hard_task = self.TaskInputR1.text()

        else:
            self.TaskInputR1.setText(task_exp[idx])
            self.hard_task = self.TaskInputR1.text()

        return self.hard_task

    def _init_robot(self, map_name):

        if not self.InitPosInputR1.text():
            self.PlanViewer.append('Please input the initial position of robot_1')

        else:
            start_pos = self.regions.keys()[self.regions.values().index(set([str(self.InitPosInputR1.text())]))]
            # From values to key, use property instead of position coordinate
            pos_x = start_pos[0]
            pos_y = start_pos[1]
            pos_z = start_pos[2]
            # Parse position coordinate
            arg_map = 'world:='+map_name
            arg_x = 'x:='+str(pos_x)
            arg_y = 'y:='+str(pos_y)
            arg_z = 'z:=' + str(pos_z)
            sys.argv.append(arg_map)
            sys.argv.append(arg_x)
            sys.argv.append(arg_y)
            sys.argv.append(arg_z)
            # Add parameter before roslaunch, since roslaunch api does not support parameter passing
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/lin/rosSpace/src/tb_simulation/launch/tbgazebo.launch"])
            launch.start()

    def _plan_syn(self, map_name, ap, regions, edges, action, hard_task):
        self.PlanViewer.clear()

        #self.s.login('12.0.5.2', 'ubuntu', 'ubuntu', original_prompt='$$S', port=22, auto_prompt_reset=True)

        if not (map_name and hard_task and self.InitPosInputR1.text()):
            if not map_name:
                self.PlanViewer.append('Please select a map first')
            elif not hard_task:
                self.PlanViewer.append('Please input a task first')
            else:
                self.PlanViewer.append('Please input a initial position')

        else:
            start_pos = regions.keys()[regions.values().index(set([str(self.InitPosInputR1.text())]))]
            # here the region must start with lower-case r instead of capital R

            robot_motion = MotionFts(regions, ap, map_name)
            robot_motion.set_initial(start_pos)
            robot_motion.add_un_edges(edges, unit_cost=0.1)
            robot_action = ActionModel(action)
            robot_model = MotActModel(robot_motion, robot_action)
            sys.stdout = EmittingStream(textWritten=self.text_written)
            robot_planner = ltl_planner(robot_model, hard_task, None)
            robot_planner.optimal(10, 'static')
            sys.stdout = sys.__stdout__

            self.PlanViewer.append('The start point is: '+str(start_pos))
            self.PlanViewer.append('It will move to the next point: '+str(robot_planner.next_move))

            stream = file(os.path.join(self.rp.get_path('rqt_ltl'), 'map', 'task.yaml'), 'w')
            data = dict(total_task=hard_task + ',' + '',
                        start_point=start_pos,
                        map_name=map_name
                        )
            yaml.dump(data, stream, default_flow_style=False, allow_unicode=False)

    def _plan_exe(self):
        #self.s.sendline('export TURTLEBOT_MAP_FILE=/home/ubuntu/test0528/testmap/test.yaml')
        #self.s.sendline('roslaunch turtlebot_navigation amcl_demo.launch')
        #Todo: add Rviz launch for visulization
        if self.launch is not None:
            self.launch.shutdown()
            self.launch = None

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, [os.path.join(self.rp.get_path('move_action'), 'src/launch', 'move_navi.launch')])
        self.launch.start()

    def _stop_plan(self):
        rospy.set_param('urgent_task', 1)
        self.publisher = rospy.Publisher('/move_base/cancel', actionlib_msgs.msg.GoalID, queue_size=10)
        self.publisher.publish(None, None)
        # Cancel goals and move_base action

    def _navigation(self):
        if self.publisher is not None:
            self.publisher.unregister()
            rospy.set_param('urgent_task', 0)
        else:
            self.publisher = rospy.Publisher('/move_base/cancel', actionlib_msgs.msg.GoalID, queue_size=10)
            self.publisher.publish(None, None)
            # Initialize move_base cancellation, otherwise stop button needs to be clicked twice.

        package = 'move_action'
        executable = 'state_publisher.py'
        self.node = roslaunch.core.Node(package, executable)

        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()
        process = launch.launch(self.node)

    def _default_setting(self):

        self.MapSelection.currentIndexChanged.connect(self._map_selection)
        self.PlanExample.currentIndexChanged.connect(self._task_examples)
        self.MapSelection.setCurrentIndex(1)
        self.PlanExample.setCurrentIndex(1)
        self.InitPosInputR1.setText('r1')
        self.MapSelection.currentIndexChanged.disconnect(self._map_selection)
        self.PlanExample.currentIndexChanged.disconnect(self._task_examples)

    def _ui_init(self):

        self.PlanViewer.clear()
        self.MapSelection.clear()
        self.mapLayout.addWidget(self.MapWidget)
        self.MapWidget.show()

        map_list = ('select a map', 'hotel', 'hospital', 'gazebo', 'pal_office', 'testmap')
        for _map in map_list:
            self.MapSelection.addItem(_map)

        self.PlanExample.clear()
        ex_plan_list = ('choose an example task', 'Example Task 1', 'Example Task 2', 'Custom')
        for _explan in ex_plan_list:
            self.PlanExample.addItem(_explan)

    def shutdown_plugin(self):
        self.MapDefineWidget.close()

    def save_settings(self, plugin_settings, instance_settings):
        # instance_settings.set_value('splitter', self._splitter.saveState())
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # if instance_settings.contains('splitter'):
        #   self._splitter.restoreState(instance_settings.value('splitter'))
        # else:
        #       self._splitter.setSizes([100, 100, 200])
        pass

    def text_written(self, text):
        cursor = self.PlanViewer.textCursor()
        cursor.movePosition(QTextCursor.End)
        cursor.insertText(text)
        self.PlanViewer.setTextCursor(cursor)
        self.PlanViewer.ensureCursorVisible()
        # https://stackoverflow.com/questions/8356336/how-to-capture-output-of-pythons-interpreter-and-show-in-a-text-widget
        # Hijack print from console to textBrowser


class EmittingStream(QObject):

    textWritten = Signal(str)

    def write(self, text):
        self.textWritten.emit(str(text))

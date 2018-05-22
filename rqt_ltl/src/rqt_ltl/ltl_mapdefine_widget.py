# import sys
import os
import rospkg
from collections import OrderedDict
import math
import networkx as nx
import yaml
import copy

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding.QtWidgets import QWidget
# python_qt_binding does not support qstringlist, use python string list instead
# Hint: QStringlistview can use python string list directly
from python_qt_binding import loadUi


class MouseTracker(QWidget):

    def __init__(self):
        '''
        Map-define widget
        '''
        super(MouseTracker, self).__init__()

        self.rp = rospkg.RosPack()
        ui_file = os.path.join(self.rp.get_path('rqt_ltl'), 'resource', 'map_widget.ui')
        loadUi(ui_file, self)
        self.setWindowTitle('Map Define')

        self.map_name = ''
        self._map_size = 0
        self.scene = QGraphicsScene()
        self._map = QPixmap()
        self.pixMapItem = MapPropertyPixmap(self._map, None, self.scene)  # use this instead of addPixmap
        self.setMouseTracking(False)

        # https://stackoverflow.com/questions/22963306/what-is-the-signal-of-selecting-an-item-in-qlistview
        self.saveButton.clicked.connect(self._saveYaml)
        self.loadButton.clicked.connect(self._loadYaml)
        self.ResetButton.clicked.connect(self._resetMap)
        self.NodesAddButton.clicked.connect(self._nodeAdd)
        self.EdgesAddButton.clicked.connect(self._edgeAdd)
        self.EdgesRemoveButton.clicked.connect(self._edgeRemove)
        #self.EdgesGenButton.clicked.connect(self._edgeAutoGenerate)
        #self.EdgesSimButton.clicked.connect(self._edgeSimplifiedGen)
        self.EdgesClrButton.clicked.connect(self._edgeClear)
        self.QuitButton.clicked.connect(self._quit)
        self.MarkSizecheckBox.stateChanged.connect(self._setMark)
        self.MarkSizecheckBox.setCheckState(Qt.Unchecked)
        #self.UpdatecheckBox.stateChanged.connect(self._updateRealtime)
        #self.UpdatecheckBox.setCheckState(Qt.Unchecked)

        self.markPos = []
        self.markAngle = []
        self.mapPos = []
        self.sGraph = nx.Graph()
        self.fGraph = nx.complete_graph(0)
        self.mapProperty = dict()
        self.mapProReverse = dict()
        self.mapProSimplified = dict()
        self.mapNodes = []
        self.mapEdges = []

        self.ap = set()
        self.regions = dict()
        self.edges = []
        self.action = dict()

    def getMapName(self, map_name):
        self.map_name = map_name

    def openMap(self):
        try:
            with open(os.path.join(self.rp.get_path('rqt_ltl'), 'map/' + self.map_name, 'map.yaml'), 'r') as streamr:
                regions_load = yaml.load(streamr)
        except IOError:
            print 'Missing ' + self.map_name + '/map.yaml, open hotel instead'
            self.map_name = 'hotel'
            with open(os.path.join(self.rp.get_path('rqt_ltl'), 'map/' + self.map_name, 'map.yaml'), 'r') as streamr:
                regions_load = yaml.load(streamr)

        self._map = QPixmap(os.path.join(self.rp.get_path('rqt_ltl'), 'map/' + self.map_name, 'map.png'))
        if self._map.isNull():
            self._map = QPixmap(os.path.join(self.rp.get_path('rqt_ltl'), 'map/' + self.map_name, 'map.pgm'))
        self._map_size = self._map.size()
        self.scene.setSceneRect(QRectF(0, 0, self._map_size.width(), self._map_size.height()))
        self.pixMapItem = MapPropertyPixmap(self._map, None, self.scene)  # use this instead of addPixmap
        self.pixMapItem.setPixmap(self._map)
        self.pixMapItem.getMapYaml(regions_load.get('resolution'), regions_load.get('origin'))
        self.view.setScene(self.scene)
        self.view.fitInView(QRectF(0, 0, self._map_size.width(), self._map_size.height()), Qt.KeepAspectRatio)
        self.NodeslistView.setModel(self.pixMapItem.map_model)
        self.EdgeslistView.setModel(self.pixMapItem.edges_model)
        self.EdgeslistView.selectionModel().selectionChanged.connect(self._edgeChanged)

    def _updateRealtime(self):
        '''
        Bring bugs, stop using
        '''
        if self.UpdatecheckBox.isChecked():
            self.pixMapItem.update_flag = 1
            self.pixMapItem.update()
        else:
            self.pixMapItem.update_flag = 0
            self.pixMapItem.update()

    def _setMark(self):
        if self.MarkSizecheckBox.isChecked():
            self.pixMapItem.pen_size = 2
            self.pixMapItem.text_size = 150
        else:
            self.pixMapItem.pen_size = 5
            self.pixMapItem.text_size = 1000
        self.pixMapItem.update()

    def _saveYaml(self):
        # They were pointing to the same address and change together
        # if directly use self.markPos = self.pixMapItem.mark_pos
        self.edges[:] = []
        self.markPos = self.pixMapItem.mark_pos[:]
        self.mapPos = self.pixMapItem.map_pos[:]
        self.markAngle = self.pixMapItem.mark_angle[:]
        self.sGraph = self.pixMapItem.simplified_graph.copy()
        self.fGraph = self.pixMapItem.full_graph.copy()
        self.mapProperty = copy.copy(self.pixMapItem.map_property)
        self.mapProReverse = copy.copy(self.pixMapItem.map_property_reverse)
        self.mapProSimplified = copy.copy(self.pixMapItem.map_property_simplified)
        self.mapNodes = self.pixMapItem.map_data[:]
        self.mapEdges = self.pixMapItem.edges_data[:]

        self.regions = dict(copy.copy(self.mapProperty))
        for item in self.mapProReverse.keys():
            self.ap.add(item)

        for item in sorted(self.sGraph.edges()):
            point1 = self.mapProReverse[item[0]]
            point2 = self.mapProReverse[item[1]]
            self.edges.append((point1, point2))

        stream = file(os.path.join(self.rp.get_path('rqt_ltl'), 'map/' + self.map_name, 'data.yaml'), 'w')
        data = dict(
            regions=self.regions,
            ap=self.ap,
            edges=self.edges,
            action=self.action,
            graph=self.sGraph,
            drawing=self.mapProSimplified,
            markPos=self.markPos,
            mapPos=self.mapPos,
            markAngle=self.markAngle,
            fGraph=self.fGraph,
            mapProperty=self.mapProperty,
            mapProReverse=self.mapProReverse,
            mapNodes=self.mapNodes,
            mapEdges=self.mapEdges
        )
        yaml.dump(data, stream, default_flow_style=False, allow_unicode=True)

    def _loadYaml(self):
        with open(os.path.join(self.rp.get_path('rqt_ltl'), 'map/' + self.map_name, 'data.yaml'), 'r') as streamr:
            regions_load = yaml.load(streamr)

        self.pixMapItem.resetMap()
        self.pixMapItem.mark_pos = regions_load.get('markPos')
        self.pixMapItem.map_pos = regions_load.get('mapPos')
        self.pixMapItem.mark_angle = regions_load.get('markAngle')
        self.pixMapItem.simplified_graph = regions_load.get('graph')
        self.pixMapItem.full_graph = regions_load.get('fGraph')
        self.pixMapItem.map_property = regions_load.get('mapProperty')
        self.pixMapItem.map_property_reverse = regions_load.get('mapProReverse')
        self.pixMapItem.map_property_simplified = regions_load.get('drawing')
        self.pixMapItem.map_data = regions_load.get('mapNodes')
        self.pixMapItem.edges_data = regions_load.get('mapEdges')
        self.pixMapItem.map_model.setStringList(self.pixMapItem.map_data)
        self.pixMapItem.edges_model.setStringList(sorted(self.pixMapItem.edges_data))
        self.pixMapItem.update()

    def _resetMap(self):
        self.pixMapItem.resetMap()
        self.EdgesInput.clear()
        self.NodeslistView.update()
        self.EdgeslistView.update()

    def _nodeAdd(self):
        if not len(self.NodesInput.text()) == 0:
            self.StateLabel.clear()
            self.pixMapItem.nodeAdd(self.NodesInput.text())
        else:
            self.StateLabel.setText('Please input node')

    def _edgeAdd(self):
        if not len(self.EdgesInput.text()) == 0:
            self.StateLabel.clear()
            self.pixMapItem.edgeAdd(self.EdgesInput.text())
        else:
            self.StateLabel.setText('Please input edge')

    def _edgeRemove(self):
        selectedIndex = self.EdgeslistView.selectionModel().currentIndex()
        selectedEdge = selectedIndex.data()
        self.pixMapItem.edgeRemove(selectedEdge)
        self.EdgeslistView.update()

    def _edgeClear(self):
        self.pixMapItem.edgeClear()
        self.EdgeslistView.update()

    def _edgeAutoGenerate(self):
        self.pixMapItem.edgeAutoGenerate()
        self.EdgeslistView.selectionModel().clearSelection()
        self.pixMapItem.is_edge_selected = self.EdgeslistView.selectionModel().hasSelection()
        self.EdgeslistView.update()

    def _edgeSimplifiedGen(self):
        self.pixMapItem.updateSimplifiedGraph()
        self.EdgeslistView.selectionModel().clearSelection()
        self.pixMapItem.is_edge_selected = self.EdgeslistView.selectionModel().hasSelection()
        self.EdgeslistView.update()

    def _edgeChanged(self):
        selectedIndex = self.EdgeslistView.selectionModel().currentIndex()
        isedgeSelected = self.EdgeslistView.selectionModel().hasSelection()
        selectedEdge = selectedIndex.data()
        # https://stackoverflow.com/questions/14546913/how-to-get-item-selected-from-qlistview-in-pyqt
        self.pixMapItem.edgeChanged(selectedEdge, isedgeSelected)
        self.EdgeslistView.update()

    def _quit(self):
        self.close()

    def wheelEvent(self, event):
        factor = 1.41 ** (event.delta() / 240.0)
        self.view.scale(factor, factor)
        self.repaint()

    def paintEvent(self, event):
        if self.pixMapItem.angle > math.pi:
            angle = self.pixMapItem.angle - 2 * math.pi
        else:
            angle = self.pixMapItem.angle
        self.PoseLabel.setText(
            'Pos: ( %.1f : %.1f )' % (self.pixMapItem.map_x + self.pixMapItem.origin[0], self.pixMapItem.map_y + self.pixMapItem.origin[1]) + '  Yaw: ' + str(angle))
        self.NodeslistView.update()
        self.update()


class MapPropertyPixmap(QGraphicsPixmapItem):

    def __init__(self, pixmap, parent=None, scene=None):
        super(MapPropertyPixmap, self).__init__(pixmap, parent, scene)
        # Here needs to inherit 3 parameters
        # self.setObjectName('MapPropertyPixmap') Here is confused why there is no attribute of setObjectName
        self._map = pixmap
        self._scale = 0.000001
        self.origin = []
        self.angle = 0
        self.radius = 0.5
        self.update_flag = 1

        self.mouse_x = 0
        self.mouse_y = 0
        self.yaw_line_start_x = 0
        self.yaw_line_start_y = 0
        self.map_x = 0
        self.map_y = 0

        self.mark_pos = []
        self.mark_angle = []
        self.map_pos = []
        self.map_property = OrderedDict()
        self.map_property_reverse = dict()
        self.map_property_simplified = dict()
        self.distance = []

        self.line = QLineF()
        self.pen = QPen(Qt.red, 10, Qt.SolidLine)
        self.brush = QBrush(Qt.white)
        self.pen_size = 5
        self.text_size = 1000

        self.mouse_pressed_flag = False
        self.mouse_released_flag = True
        self.is_edge_selected = False
        self.setAcceptHoverEvents(False)

        self.map_data = []
        self.map_model = QStringListModel(self.map_data)

        self.edges_data = []
        self.edges_model = QStringListModel(self.edges_data)

        self.simplified_graph = nx.Graph()
        self.full_graph = nx.complete_graph(0)
        self.selected_edge = []

    def getMapYaml(self, scale, origin):
        self._scale = scale
        self.origin = origin
        # Yaw of the map is neglected

    def resetMap(self):
        self.map_data[:] = []
        self.map_model.setStringList(self.map_data)
        self.edges_data[:] = []
        self.edges_model.setStringList(self.edges_data)
        self.resetParam()
        self.update()

    def nodeAdd(self, pos):
        try:
            pos = pos.split(',')
            distance = []
            tmpx = float(str(pos[0]))
            tmpy = float(str(pos[1]))
            tmpyaw = float(str(pos[2]))
            axis_x = round((tmpx - self.origin[0]) / self._scale)
            axis_y = round(self._map.height() - (tmpy - self.origin[1]) / self._scale)

            if axis_x >= self._map.width() or axis_x <= 0 or axis_y >= self._map.height() or axis_y <= 0:
                print 'out of range'
                # Nodes should be in the range of map
                pass
            else:
                pos_tmp = (axis_x, axis_y)
                if len(self.mark_pos) == 0:
                    self.mark_pos.append(pos_tmp)
                    self.map_pos.append((tmpx, tmpy))
                else:
                    for pose in self.mark_pos:
                        distance.append(math.sqrt((pose[0] - pos_tmp[0]) ** 2 + (pose[1] - pos_tmp[1]) ** 2))

                    if min(distance) < 2 * self.radius / self._scale:
                        print 'Too close'
                    else:
                        self.mark_pos.append(pos_tmp)
                        self.map_pos.append((tmpx, tmpy))

                if len(distance) == 0:
                    self.mark_angle.append(round(tmpyaw, 3))
                    self.map_data.append('r' + str(len(self.mark_angle)) + ':' + str(
                        self.map_pos[len(self.mark_angle) - 1]) + ', yaw=' + str(self.mark_angle[len(self.mark_angle) - 1]))
                    for index in range(len(self.mark_angle)):
                        self.map_property[(self.map_pos[index][0], self.map_pos[index][1], self.mark_angle[index])] = set(['r' + str(index + 1), ])
                else:
                    if min(distance) < 2 * self.radius / self._scale:
                        pass
                    else:
                        self.mark_angle.append(round(tmpyaw, 3))
                        self.map_data.append('r'+str(len(self.mark_angle))+':'+str(self.map_pos[len(self.mark_angle)-1])+', yaw='+str(self.mark_angle[len(self.mark_angle)-1]))
                        for index in range(len(self.mark_angle)):
                            self.map_property[(self.map_pos[index][0], self.map_pos[index][1], self.mark_angle[index])] = set(['r' + str(index+1), ])

                self.mappingReverse()
                self.updateFullGraph(self.map_property)
                if self.update_flag:
                    self.updateSimplifiedGraph()
                self.update()

        except IndexError:
            print 'Please input in a right format'

    def edgeAdd(self, edge):
        try:
            added_edge = edge.split(',')
            node1 = str(added_edge[0])
            node2 = str(added_edge[1])

            edge_tuple = (node1, node2)
            edge_tuple_reverse = (node2, node1)

            if (edge_tuple in self.simplified_graph.edges()) or (edge_tuple_reverse in self.simplified_graph.edges()):
                pass
            else:
                if (node1 in self.map_property_reverse) and (node2 in self.map_property_reverse):
                    self.simplified_graph.add_edges_from([edge_tuple])
                else:
                    pass

            self.edges_data = map('<----------->'.join, self.simplified_graph.edges())
            self.edges_model.setStringList(sorted(self.edges_data))
            self.update()
        except IndexError:
            print 'input edge between two nodes'

    def edgeRemove(self, selectedEdge):
        try:
            remove_edge = selectedEdge.split('<----------->')
            self.simplified_graph.remove_edge(str(remove_edge[0]), str(remove_edge[1]))
            self.edges_data = map('<----------->'.join, self.simplified_graph.edges())
            self.edges_model.setStringList(sorted(self.edges_data))
            self.selected_edge[:] = []
            self.update()
        except AttributeError:
            print 'please select one edge to remove'

    def edgeClear(self):
        self.edges_data[:] = []
        self.edges_model.setStringList(self.edges_data)
        self.simplified_graph.clear()
        self.simplified_graph.add_nodes_from(self.full_graph.node)
        self.update()

    def edgeChanged(self, selectedEdge, isedgeSelected):
        self.is_edge_selected = isedgeSelected
        self.selected_edge = selectedEdge.split('<----------->')
        self.update()

    def edgeAutoGenerate(self):
        # https://stackoverflow.com/questions/36077759/edge-between-all-nodes-in-networkx-graph
        # https://stackoverflow.com/questions/11696078/python-converting-a-list-of-tuples-to-a-list-of-strings
        self.edges_data[:] = []
        self.edges_model.setStringList(self.edges_data)
        self.simplified_graph.clear()

        self.simplified_graph.add_nodes_from(self.full_graph.node)
        self.simplified_graph.add_edges_from(self.full_graph.edges())
        self.edges_data = map('<----------->'.join, self.simplified_graph.edges())
        self.edges_model.setStringList(sorted(self.edges_data))
        self.update()

    def paint(self, painter, option, widget=None):
        painter.setPen(self.pen)
        painter.setBrush(self.brush)
        painter.drawPixmap(0, 0, self._map)
        if self.mouse_pressed_flag:
            self.drawLine(painter)
            self.drawMark(painter)

        if self.mouse_released_flag:
            self.drawMark(painter)
            self.drawEdge(painter)

    def drawEdge(self, painter):
        line = []
        for item in self.simplified_graph.edges():
            x1 = (self.map_property_reverse[item[0]][0] - self.origin[0])/self._scale
            y1 = self._map.height() - (self.map_property_reverse[item[0]][1] - self.origin[1])/self._scale
            x2 = (self.map_property_reverse[item[1]][0] - self.origin[0])/self._scale
            y2 = self._map.height() - (self.map_property_reverse[item[1]][1] - self.origin[1])/self._scale
            line.append(QLineF(x1, y1, x2, y2))

        pen = QPen(Qt.red, 1, Qt.SolidLine)
        painter.setPen(pen)

        for l in line:
            painter.drawLine(l)

        if self.is_edge_selected and (not len(self.selected_edge) == 0):
            pen = QPen(Qt.blue, 5, Qt.SolidLine)
            painter.setPen(pen)
            x1 = (self.map_property_reverse[self.selected_edge[0]][0] - self.origin[0]) / self._scale
            y1 = self._map.height() - (self.map_property_reverse[self.selected_edge[0]][1] - self.origin[1]) / self._scale
            x2 = (self.map_property_reverse[self.selected_edge[1]][0] - self.origin[0]) / self._scale
            y2 = self._map.height() - (self.map_property_reverse[self.selected_edge[1]][1] - self.origin[1]) / self._scale
            painter.drawLine(QLineF(x1, y1, x2, y2))
            # When painting edges, remove the origin offsets

    def drawLine(self, painter):
        self.line = QLineF(self.yaw_line_start_x, self.yaw_line_start_y, self.mouse_x, self.mouse_y)
        self.angle = math.radians(self.line.angle())
        painter.drawLine(self.line)

    def drawMark(self, painter):
        font = QFont()
        font.setPointSizeF(self.text_size * self._scale)
        pen = QPen(Qt.black, self.pen_size, Qt.SolidLine)
        painter.setPen(pen)
        painter.setBrush(self.brush)
        painter.setFont(font)
        for index in range(len(self.mark_pos)):
            painter.drawEllipse(self.mark_pos[index][0] - self.radius/self._scale, self.mark_pos[index][1] - self.radius/self._scale, 2 * self.radius/self._scale, 2 * self.radius/self._scale)
            painter.drawText(self.mark_pos[index][0]-self.text_size * 0.7 * self._scale, self.mark_pos[index][1]+self.text_size / 2 * self._scale, 'r'+str(index+1))
            # painter.drawLine(self.mark_pos[index][0], self.mark_pos[index][1], self.mark_pos[index][0] + 50 * math.cos(self.line.angle()), self.mark_pos[index][1] + 50 * math.sin(self.line.angle()))

    def mousePressEvent(self, event):
        self.map_x = round(event.pos().x() * self._scale, 1)
        self.map_y = round((self._map.height() - event.pos().y()) * self._scale, 1)
        self.yaw_line_start_x = event.pos().x()
        self.yaw_line_start_y = event.pos().y()
        self.mouse_pressed_flag = True
        tmpx = round(self.map_x + self.origin[0], 1)
        tmpy = round(self.map_y + self.origin[1], 1)
        # Here needs to consider the origin offset of the map

        if event.button() == Qt.LeftButton:
            pos_tmp = (event.pos().x(), event.pos().y())

            if len(self.mark_pos) == 0:
                self.mark_pos.append(pos_tmp)
                self.map_pos.append((tmpx, tmpy))
            else:
                for pos in self.mark_pos:
                    self.distance.append(math.sqrt((pos[0] - event.pos().x())**2 + (pos[1] - event.pos().y())**2))

                if min(self.distance) < 2 * self.radius / self._scale:
                    pass
                else:
                    self.mark_pos.append(pos_tmp)
                    self.map_pos.append((tmpx, tmpy))

        if event.button() == Qt.RightButton:
            for pos in self.mark_pos:
                self.distance.append(math.sqrt((pos[0] - event.pos().x())**2 + (pos[1] - event.pos().y())**2))

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:

            if self.angle > math.pi:
                self.angle = self.angle - 2 * math.pi

            if len(self.distance) == 0:
                self.mark_angle.append(round(self.angle, 3))
                self.map_data.append('r' + str(len(self.mark_angle)) + ':' + str(
                    self.map_pos[len(self.mark_angle) - 1]) + ', yaw=' + str(self.mark_angle[len(self.mark_angle) - 1]))
                for index in range(len(self.mark_angle)):
                    self.map_property[(self.map_pos[index][0], self.map_pos[index][1], self.mark_angle[index])] = set(['r' + str(index + 1), ])

            else:
                if min(self.distance) < 2 * self.radius / self._scale:
                    pass
                else:
                    self.mark_angle.append(round(self.angle, 3))
                    self.map_data.append('r'+str(len(self.mark_angle))+':'+str(self.map_pos[len(self.mark_angle)-1])+', yaw='+str(self.mark_angle[len(self.mark_angle)-1]))
                    for index in range(len(self.mark_angle)):
                        self.map_property[(self.map_pos[index][0], self.map_pos[index][1], self.mark_angle[index])] = set(['r' + str(index+1), ])

            self.distance[:] = []

        elif event.button() == Qt.RightButton:
            for dist in self.distance:
                if dist < self.radius / self._scale:
                    self.mark_pos.pop(self.distance.index(dist))
                    self.mark_angle.pop(self.distance.index(dist))
                    self.map_pos.pop(self.distance.index(dist))
                    self.map_data[:] = []
                    self.map_property.clear()
                    for index in range(len(self.mark_angle)):
                        self.map_property[(self.map_pos[index][0], self.map_pos[index][1], self.mark_angle[index])] = set(['r' + str(index+1), ])
                        self.map_data.append('r'+str(index+1)+':'+str(self.map_pos[index])+', yaw='+str(self.mark_angle[index]))

            self.distance[:] = []

        self.mappingReverse()
        self.updateFullGraph(self.map_property)
        if self.update_flag:
            self.updateSimplifiedGraph()
        self.mouse_pressed_flag = False
        self.mouse_released_flag = True
        # https://stackoverflow.com/questions/10472907/how-to-convert-dictionary-into-string
        # https://stackoverflow.com/questions/5253773/convert-list-of-dicts-to-string
        self.update()

    def mouseMoveEvent(self, event):
        self.mouse_x = event.pos().x()
        self.mouse_y = event.pos().y()
        self.update()

    def mappingReverse(self):
        nodes = []
        posseq = []
        self.map_property_reverse.clear()
        self.map_property_simplified.clear()

        for region in self.map_property.values():
            nodes.append(' '.join([i for i in region]))
        for pos in self.map_property.keys():
            posseq.append(pos)

        # https://stackoverflow.com/questions/13264946/how-to-assign-list-of-values-to-a-key-using-ordereddict-in-python
        for i, key in enumerate(nodes):
            self.map_property_reverse[key] = posseq[i]
            self.map_property_simplified[key] = [posseq[i][0], posseq[i][1]]

    def updateFullGraph(self, map_property):
        G = nx.complete_graph(len(map_property.keys()))
        nodes = []
        for label in map_property.values():
            nodes.append(' '.join([i for i in label]))
        node_map = {i: nodes for i, nodes in enumerate(nodes)}
        self.full_graph = nx.relabel_nodes(G, node_map)

    def updateSimplifiedGraph(self):
        self.simplified_graph.clear()
        self.simplified_graph.add_nodes_from(self.full_graph.nodes())
        # Add two nearest edge for every node here
        self.simplified_graph.add_edges_from(self.simplifyEdge(self.map_property_simplified))
        self.edges_data = map('<----------->'.join, self.simplified_graph.edges())
        self.map_model.setStringList(self.map_data)
        self.edges_model.setStringList(sorted(self.edges_data))
        self.update()

    def simplifyEdge(self, mapping):
        distance = {key: [] for key in mapping.keys()}
        edge = []

        for key1 in mapping.keys():
            for key2 in mapping.keys():
                if not key1 == key2:
                    d = {math.sqrt((mapping[key1][0] - mapping[key2][0]) ** 2 + (mapping[key1][1] - mapping[key2][1]) ** 2): key2}
                    distance[key1].append(d)
        # Calculate distance between two different nodes

        if len(distance) >= 3:
            for key in distance:
                edge.append((key, sorted(distance[key])[0].values()[0]))
                edge.append((key, sorted(distance[key])[1].values()[0]))
                # only reserve two nearest nodes
            unordered = [frozenset(element) for element in edge]
            cleaned = set(unordered)
            edge = [tuple(element) for element in cleaned]
            # remove duplicate tuple
            # https://stackoverflow.com/questions/36755714/how-to-ignore-the-order-of-elements-in-a-tuple
        else:
            edge = self.full_graph.edges()
        return edge

    def resetParam(self):
        self.mouse_x = 0
        self.mouse_y = 0
        self.yaw_line_start_x = 0
        self.yaw_line_start_y = 0
        self.map_x = 0
        self.map_y = 0
        self.mark_pos = []
        self.distance = []
        self.map_pos = []
        self.mark_angle = []
        self.simplified_graph.clear()
        self.full_graph.clear()
        self.selected_edge = []
        self.map_property.clear()
        self.map_property_reverse.clear()
        self.map_property_simplified.clear()

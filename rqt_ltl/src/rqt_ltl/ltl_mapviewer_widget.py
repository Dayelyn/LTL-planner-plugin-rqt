from python_qt_binding import QtGui, QtCore
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas

import matplotlib.pyplot as plt
import matplotlib.image as mimg
import networkx as nx


class MplCanvas(FigureCanvas):
    '''
    TODO: Need a scaling with mouse wheel
    '''
    def __init__(self):
        self.fig = plt.figure()
        super(MplCanvas, self).__init__(self.fig)
        self.setSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
        self.updateGeometry()
        self.ax = self.fig.add_axes([0, 0, 1, 1])  # No margin

        self.map_path = []
        self._scale = 0
        self._height = 0
        self._width = 0
        self.img = None
        self.regions = {}
        self.edges = []
        self.graph = nx.Graph()
        self.mapproperty = nx.Graph()

    def drawMap(self):
        self.img = mimg.imread(self.map_path)
        self._height = self.img.shape[0]
        self._width = self.img.shape[1]

        self.ax.imshow(self.img, extent=[0, self._width * self._scale, 0, self._height * self._scale], interpolation="nearest", aspect='equal')
        # parameter extent can scale the axis
        self.ax.set_axis_off()
        self._add_node(self.mapproperty)
        nx.draw_networkx(self.mapproperty, self.regions, node_size=30/self._scale, node_color='w', width=3.0, edge_color='r', font_size=str(0.35/self._scale))

    def _add_node(self, graph):
        graph.clear()
        graph.add_nodes_from(self.graph.nodes())
        graph.add_edges_from(self.graph.edges())


class MplWidget(QtGui.QWidget):

    def __init__(self, parent = None):
        QtGui.QWidget.__init__(self, parent)
        self.canvas = MplCanvas()
        self.canvas.setFocusPolicy(QtCore.Qt.ClickFocus)
        self.canvas.setFocus()
        self.vbl = QtGui.QVBoxLayout()
        self.vbl.addWidget(self.canvas)
        self.setLayout(self.vbl)

    def getMapParam(self, map_path, scale, origin, regions, graph):
        self.canvas.map_path = map_path
        self.canvas._scale = scale
        self.canvas.regions = regions
        for key in regions:
            self.canvas.regions[key][0] = regions[key][0] - origin[0]
            self.canvas.regions[key][1] = regions[key][1] - origin[1]
        self.canvas.graph = graph

    def showMap(self):
        self.canvas.ax.cla()
        self.canvas.drawMap()
        self.canvas.draw()

    def clearMap(self):
        self.canvas.ax.cla()
        self.canvas.ax.set_axis_off()
        self.canvas.draw()






#!/usr/bin/env python
import rospy
import sys, random
import signal
import cv2
import numpy
import matplotlib
from matplotlib.backends.backend_qt5agg import (
    FigureCanvasQTAgg as FigureCanvas,
    NavigationToolbar2QT as NavigationToolbar)
from matplotlib.figure import Figure
from PyQt5 import QtCore, QtGui, QtWidgets
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

##########################################
## Plot a histogram from a colour image ##
##########################################
##
## From the terminal run:
##
##  rosrun utils online_histogram.py image:=/stereo_down/left/image_rect_color
##
## or from a launchfile:
##
##  <node name="online_histogram" pkg="utils" type="online_histogram.py">
##    <remap from="image" to="/stereo_down/left/image_rect_color"/>
##  </node>

def handleIntSignal(signum, frame):
    '''Ask app to close if Ctrl+C is pressed.'''
    QtWidgets.QApplication.closeAllWindows()

class PlotWindowWidget(QtWidgets.QMainWindow):
    def __init__(self, parent=None):
        QtWidgets.QMainWindow.__init__(self, parent)
        self.setWindowTitle('Histogramm')
        self.create_main_frame()
        self.on_draw()

    def save_plot(self):
        pass

    def on_about(self):
        pass

    def on_pick(self, event):
        pass

    def on_draw(self):
        self.axes.clear()
        self.axes.grid(True)
        self.canvas.draw()

    def create_main_frame(self):
        self.main_frame = QtWidgets.QWidget()
        self.dpi = 100
        self.fig = Figure((5.0, 4.0), dpi=self.dpi)
        self.canvas = FigureCanvas(self.fig)
        self.canvas.setParent(self.main_frame)
        self.axes = self.fig.add_subplot(111)
        self.canvas.mpl_connect('pick_event', self.on_pick)
        self.mpl_toolbar = NavigationToolbar(self.canvas, self.main_frame)
        vbox = QtWidgets.QVBoxLayout()
        vbox.addWidget(self.canvas)
        vbox.addWidget(self.mpl_toolbar)
        self.main_frame.setLayout(vbox)
        self.setCentralWidget(self.main_frame)

class OnlineHistWidget(PlotWindowWidget):
  def __init__(self):
    PlotWindowWidget.__init__(self)
    rospy.init_node('visualizer', anonymous=True)
    self.subscriber = rospy.Subscriber("image", Image, self.plotResults, queue_size = 1 )
    self.bridge = CvBridge()

  def plotResults(self, data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    self.axes.clear()
    self.axes.set_autoscaley_on(False)

    color = ('b','g','r')
    for i,col in enumerate(color):
        histr = cv2.calcHist([cv_image],[i],None,[256],[0,256])
        (dummy, maxVal, dummy, dummy) = cv2.minMaxLoc(histr)
        histr = histr / maxVal;
        self.axes.plot(histr,color = col)
        self.axes.set_xlim([0,256])
        self.axes.set_ylim([0,1])
    self.canvas.draw()

if __name__ == "__main__":
    signal.signal(signal.SIGINT, handleIntSignal)
    app = QtWidgets.QApplication(sys.argv)
    window = OnlineHistWidget()
    window.show()
    while not rospy.is_shutdown():
        app.exec_()
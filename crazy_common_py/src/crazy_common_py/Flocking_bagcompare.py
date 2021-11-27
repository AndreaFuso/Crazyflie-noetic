import math

import rosbag
import rospkg
from crazy_common_py.common_functions import rad2deg
from crazy_common_py.default_topics import DEFAULT_CF_STATE_TOPIC, DEFAULT_MOTOR_CMD_TOPIC, \
    DEFAULT_ACTUAL_DESTINATION_TOPIC
from matplotlib.pyplot import plot, xlabel, ylabel,show, figure, title, ylim, subplot, xlim, legend, axes, Circle, \
    gca, axis, subplots, scatter
from numpy import array, argmax, linspace, ones, zeros, flip, concatenate, linalg, hstack, mean
from enum import Enum
from crazyflie_messages.msg import SwarmStates
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import FancyArrowPatch, Ellipse
from matplotlib.patches import Patch
from matplotlib.lines import Line2D
from mpl_toolkits.mplot3d import proj3d
from crazy_common_py.common_functions import Vector3, RotateVector, deg2rad, rad2deg
from pykalman import KalmanFilter
from scipy.signal import savgol_filter, correlate

class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0, 0), (0, 0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        FancyArrowPatch.draw(self, renderer)


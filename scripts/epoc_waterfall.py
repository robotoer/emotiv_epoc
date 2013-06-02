#!/usr/bin/env python

from __future__ import print_function

import matplotlib
matplotlib.use('GTKAgg')

# ROS imports.
import roslib
import rospy
from std_msgs.msg import UInt32

import numpy
from matplotlib import pylab

# Sensor channels.
channel = '/epoc/signal/F3'


class EPOCWaterfallGrapher(object):
    def __init__(self):
        # Create an empty buffer.
        self.signal_window = numpy.zeros(2048)
        self.received = 0

        self.signal_figure = pylab.figure(1)
        self.signal_plot = self.signal_figure.add_subplot(1, 1, 1)
        self.signal_specgram = self.signal_plot.specgram(
            self.signal_window,
            NFFT=1024,
            Fs=128)
        self.last_specgram = self.signal_specgram[-1]
        pylab.ion()

        rospy.init_node('epoc_waterfall', anonymous=True)
        self.signal_subscriber = rospy.Subscriber(
            channel,
            UInt32,
            self.handle_channel)

    def handle_channel(self, data):
        # Shift the window over 1.
        self.received += 1
        self.signal_window = self.signal_window[1:]

        # Add new data to the window.
        self.signal_window = numpy.append(self.signal_window, data.data)

        # Draw the current window.
        self.signal_plot.images.remove(self.last_specgram)
        self.signal_specgram = self.signal_plot.specgram(
            self.signal_window,
            NFFT=1024,
            Fs=128)
        self.last_specgram = self.signal_specgram[-1]
        print(self.signal_specgram)

        pylab.draw()
        pylab.show()


if __name__ == '__main__':
    roslib.load_manifest('emotiv_epoc')
    waterfall = EPOCWaterfallGrapher()
    rospy.spin()

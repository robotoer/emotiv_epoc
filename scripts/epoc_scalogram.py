#!/usr/bin/env python
#
# epoc_scalogram.py
#
# Generates a scalogram from EEG signals read from the Emotiv EPOC headset.
# Currently this only can read data from a pre-recorded ROS '.bag' file. For
# more information see: http://www.ros.org/wiki/rosbag.
#
# Usage:
#   python epoc_scalogram.py /path/to/bag/file.bag

from __future__ import print_function

import matplotlib
matplotlib.use('GTKAgg')

import argparse
import gobject
import numpy
import rosbag
import scipy.misc
import subprocess
import threading
import yaml

from Wavelets import Morlet
from matplotlib import animation
from matplotlib import cm
from matplotlib import pylab
from matplotlib import pyplot
from mpl_toolkits.mplot3d import axes3d

import rospy
from std_msgs.msg import UInt32


def read_bag(bag_location):
    # Open a bag file.
    bag = rosbag.Bag(bag_location)
    print("Bag information:")
    print(bag)
    print()
    print()

    # Get information about the .bag file.
    bag_info_command = ['rosbag', 'info', '--yaml', bag_location]
    bag_info_process = subprocess.Popen(
        bag_info_command,
        stdout=subprocess.PIPE)
    bag_info = yaml.load(bag_info_process.communicate()[0])

    # Extract the topics recorded in the .bag file.
    bag_channels = {
        topic['topic']: topic['messages']
        for topic
        in bag_info['topics']
    }

    # Setup the buffers for holding data extracted from the bag file.
    buffers = {
        topic: numpy.zeros(bag_channels[topic])
        for topic
        in bag_channels.keys()
    }
    buffer_sizes = {
        topic: 0
        for topic
        in bag_channels.keys()
    }

    # Read data from the bag file.
    messages = bag.read_messages(topics=bag_channels.keys())
    for topic, msg, t in messages:
        buffers[topic][buffer_sizes[topic]] = msg.data
        buffer_sizes[topic] += 1

    # Cleanup.
    print("Done! Read: %d messages" % sum(buffer_sizes.values()))
    bag.close()

    return buffers


def draw_specgram(signal, name):
    # Use specgram.
    pylab.specgram(signal, NFFT=256, Fs=128)
    # pylab.title(name)


def draw_scalogram(signal, name):
    # Perform a wavelet transform on the input signal.
    print("Transforming '%s' using a Morlet wavelet..." % name)
    transformed = Morlet(
        signal,
        largestscale=4,
        notes=16,
        scaling='log')
    power_matrix = transformed.getpower()

    # Display the power matrix generated during the wavelet transform.
    pylab.imshow(
        power_matrix,
        cmap=pylab.cm.jet,
        aspect='auto')


def draw_3D_scalogram(signal, name, draw_contours=False):
    # Perform a wavelet transform on the input signal.
    print("Transforming '%s' using a Morlet wavelet..." % name)
    transformed = Morlet(
        signal,
        largestscale=4,
        notes=16,
        scaling='log')
    power_matrix = transformed.getpower()

    # Downsample the image.
    matrix_dimensions = numpy.shape(power_matrix)
    mat_size = (matrix_dimensions[0], 1000)
    power = scipy.misc.imresize(power_matrix, mat_size)

    # Display the power matrix generated during the wavelet transform.
    fig = pyplot.figure(name)
    pyplot.title(name)
    axis = fig.gca(projection='3d')

    # Setup each axis' data.
    f = numpy.arange(mat_size[0])
    t = numpy.arange(mat_size[1])
    f_major, t_major = numpy.meshgrid(f, t)

    # Plot everything.
    axis.plot_surface(t_major, f_major, power.transpose(), rstride=8, cstride=8, alpha=0.3)
    if draw_contours:
        axis.contour(t_major, f_major, power.transpose(), zdir='z', offset=0, cmap=cm.coolwarm)
        axis.contour(t_major, f_major, power.transpose(), zdir='x', offset=0, cmap=cm.coolwarm)
        axis.contour(t_major, f_major, power.transpose(), zdir='y', offset=0, cmap=cm.coolwarm)
    axis.set_xlabel('t')
    axis.set_ylabel('f')
    axis.set_zlabel('amplitude')


class EPOCWaterfallGrapher(threading.Thread):
    """
    Listens to a ROS channel and displays the data as a spectrogram.

    @param figure to draw to.
    @param channel to listen to.
    @param window_size to display.
    @param channel_position of the graph on the screen.
    @param channel_count is the number of channels being displayed.
    """
    def __init__(self, figure, channel, window_size, channel_position, channel_count):
        super(EPOCWaterfallGrapher, self).__init__()
        self.channel = channel

        # Create an empty buffer.
        self.signal_window = numpy.zeros(window_size)
        self.signal_lock = threading.RLock()

        # Add a subplot to the specified figure.
        self.figure = figure
        self.signal_plot = figure.add_subplot(1, channel_count, channel_position)
        self.signal_specgram = self.signal_plot.specgram(
            self.signal_window,
            NFFT=1024,
            Fs=128)
        self.last_specgram = self.signal_specgram[-1]

        # Setup the subscriber.
        self.received = 0
        self.signal_subscriber = rospy.Subscriber(
            self.channel,
            UInt32,
            self.handle_message)
        print("Listening to ros topic: %s" % self.channel)

        # self.run()

        # gobject.timeout_add(100, self.draw_specgram)
        # gobject.idle_add(self.draw_specgram)

    def run(self):
        pyplot.show()

    def handle_message(self, data):
        print("Starting to receive...")
        # with self.signal_lock:
        # Shift the window over 1.
        self.received += 1
        self.signal_window = self.signal_window[1:]

        # Add new data to the window.
        self.signal_window = numpy.append(self.signal_window, data.data)

        # print("\rReceived: %d" % self.received, end='')
        print("Received: %d" % self.received)

    def draw_specgram(self):
        # if not rospy.is_shutdown():
        # Draw the current window.
        self.signal_plot.images.remove(self.last_specgram)

        # with self.signal_lock:
        self.signal_specgram = self.signal_plot.specgram(
            self.signal_window,
            NFFT=1024,
            Fs=128)

        self.last_specgram = self.signal_specgram[-1]
        pylab.get_current_fig_manager().canvas.draw()

        # self.figure.canvas.draw_idle()
        # pyplot.draw()

        # Print the signal vector to see what's going on with the plotting being messed up.
        # print(self.signal_window)

        print("Draw specgram...")


def ros_start():
    rospy.init_node('epoc_scalogram')
    rospy.spin()


def pyplot_start(*args):
    pyplot.draw()


if __name__ == '__main__':
    # Setup command line arguments for epoc_scalogram.
    parser = argparse.ArgumentParser(
        description='Generate a scalogram from EEG signals read from the Emotiv EPOC headset.')
    parser.add_argument('target')
    parser.add_argument('--target-type', dest='target_type', choices=['ros_topic', 'bag'], default='ros_topic')
    parser.add_argument('--display-type', dest='display_type', choices=['3d', '3d_contour', 'heatmap'], default='heatmap')
    parser.add_argument('--transform-type', dest='transform_type', choices=['morlet', 'fft'], default='fft')
    args = parser.parse_args()

    if args.target_type == 'bag':
        # Read the bag file.
        buffers = read_bag(args.target)
        print("Read buffers.")

        # Only display the signal channels.
        signals = [topic for topic in buffers if 'signal' in topic]
        print("Read signals.")

        # Draw 3d plots.
        if args.display_type == '3d' or args.display_type == '3d_contour':
            print("Drawing 3D plot.")
            for topic in signals:
                draw_3D_scalogram(buffers[topic], topic, args.display_type == '3d_contour')

        # Draw 2d plots.
        elif args.display_type == 'heatmap':
            print("Drawing heatmap.")
            pylab.figure('EEG scalograms')
            pylab.title('EEG scalograms')

            if args.transform_type == 'fft':
                for i, topic in enumerate(signals):
                    pylab.subplot(len(signals), 1, i)
                    draw_specgram(buffers[topic], topic)
            elif args.transform_type == 'morlet':
                for i, topic in enumerate(signals):
                    pylab.subplot(len(signals), 1, i)
                    draw_scalogram(buffers[topic], topic)
            else:
                print("Unsupported 'transform-type': %s" % args.transform_type)

        # Display everything.
        pyplot.show()

    elif args.target_type == 'ros_topic':
        # pyplot.ion()
        waterfall_figure = pylab.figure('EEG waterfall plot')

        # Open up listeners
        grapher = EPOCWaterfallGrapher(
            waterfall_figure,
            args.target,
            2048,
            1,
            1)
        grapher.start()
        rospy.init_node('epoc_scalogram')
        rospy.spin()

        # def animate(i):
        #     grapher.draw_specgram()
        # animation.FuncAnimation(waterfall_figure, animate, 100, repeat=False)
        # pyplot.show()

        # thread = threading.Thread(target=ros_start)
        # thread.start()

        # gobject.idle_add(ros_idle)
        # pyplot.show()
        # thread = threading.Thread(target=pyplot.show)
        # thread.start()

        # ros_start()
        # gobject.idle_add(pyplot_start)

        # pyplot.show()

        # rospy.init_node('epoc_scalogram')
        # rospy.spin()
        # while not rospy.is_shutdown():
        #     pyplot.draw()

    else:
        print("Unrecognized target-type: %s" % args.target_type)

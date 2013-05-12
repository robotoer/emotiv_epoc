#!/usr/bin/env python

from __future__ import print_function

# ROS imports.
import roslib
import rospy
from std_msgs.msg import UInt32

# Other imports.
import gevent
from emokit.emotiv import Emotiv

# Sensor channels.
channels = [
    'F3',
    'FC6',
    'P7',
    'T8',
    'F7',
    'F8',
    'T7',
    'P8',
    'AF4',
    'F4',
    'AF3',
    'O2',
    'O1',
    'FC5',
    'X',
    'Y'
]

signal_publishers = {
    channel: rospy.Publisher('epoc/signal/%s' % channel, UInt32)
    for channel
    in channels}
quality_publishers = {
    channel: rospy.Publisher('epoc/quality/%s' % channel, UInt32)
    for channel
    in channels}


def epoc_publish():
    # Open a connection to the Emotiv EPOC.
    headset = Emotiv()
    gevent.spawn(headset.setup)
    gevent.sleep(1)

    # Initialize ROS node+publisher.
    rospy.init_node('epoc_publish')

    # Start the publishing loop.
    rospy.loginfo('Starting publishing loop...')
    published_count = 0
    try:
        while not rospy.is_shutdown():
            # Get the next packet from the EPOC.
            packet = headset.dequeue()

            # Publish the the EEG channels and accelerometer values.
            for channel in channels:
                signal = UInt32(packet.sensors[channel]['value'])
                quality = UInt32(packet.sensors[channel]['quality'])
                signal_publishers[channel].publish(signal)
                quality_publishers[channel].publish(quality)

            # Update and print information count.
            published_count += 1
            print('\rPublished: %d' % published_count, end='')

            gevent.sleep(0)
    except rospy.ROSInterruptException:
        headset.close()

if __name__ == '__main__':
    roslib.load_manifest('emotiv_epoc')
    epoc_publish()

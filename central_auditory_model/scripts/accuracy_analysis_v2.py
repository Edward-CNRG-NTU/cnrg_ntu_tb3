#!/usr/bin/env python
import numpy as np
import collections
import threading
import progressbar
import time
import os

import rospy, rospkg
from std_msgs.msg import UInt8
from std_msgs.msg import String
from std_msgs.msg import Header
from central_auditory_model.msg import AngleEstimation
from confusion_matrix import draw_confusion_matrix


NODE_NAME = 'accuracy_analysis'
ANGLE_INDEX_TOPIC_NAME = '/binaural_audio/wave_stereo/angle_index'
FILE_PATH_NAME_TOPIC_NAME = '/binaural_audio/wave_stereo/file_path_name'
SUB_TOPIC_NAME = '/central_auditory_model/ic_stream/angle_estimation'

N_CAPTURE_SAMPLES = 400
SUPPORTED_ANGLES = [90, 60, 30, 0, 330, 300, 270]

def run_analysis():
    ang_idx_pub = rospy.Publisher(ANGLE_INDEX_TOPIC_NAME, UInt8, queue_size=1)
    # file_pn_pub = rospy.Publisher(FILE_PATH_NAME_TOPIC_NAME, String, queue_size=1)

    global capture_msg
    capture_msg = None
    capture_event = threading.Event()

    def angle_estim_cb(data):
        global capture_msg
        capture_msg = data
        capture_event.set()

    rospy.Subscriber(SUB_TOPIC_NAME, AngleEstimation, angle_estim_cb)

    time.sleep(1.)

    rospy.loginfo('start subscribing to %s' % SUB_TOPIC_NAME)

    angle_index = 0
    ang_idx_pub.publish(angle_index)

    capture_start = time.time() + 1.
    angle_list = []
    delay_list = []
    angle_stat_list = []
    bar = progressbar.ProgressBar(max_value=N_CAPTURE_SAMPLES)
    print 'analyzing (%d) %d' % (angle_index, SUPPORTED_ANGLES[angle_index])

    while not rospy.is_shutdown():
        
        if capture_event.wait(1.):
            capture_event.clear()
        else:
            rospy.logwarn('main loop timeout!')
            continue                
        
        if capture_msg.timecode.to_sec() > capture_start:
            angle_list.append(capture_msg.angle_index)
            delay_list.append(capture_msg.header.stamp.to_sec() - capture_msg.timecode.to_sec())

            n_msg = len(angle_list)
            bar.update(n_msg)
            if n_msg >= N_CAPTURE_SAMPLES:
                bar.finish()

                angle_stat = np.histogram(angle_list, bins=np.arange(len(SUPPORTED_ANGLES) + 1))[0].astype(np.float) / n_msg
                delay_stat = (np.mean(delay_list), np.std(delay_list))

                angle_stat_list.append(angle_stat)

                print n_msg, angle_stat, delay_stat

                if angle_index < len(SUPPORTED_ANGLES) - 1:
                    angle_index = angle_index + 1
                else:
                    draw_confusion_matrix(angle_stat_list, classes=SUPPORTED_ANGLES)
                    angle_index = 0
                    angle_stat_list = []

                # angle_index = angle_index + 1 if angle_index < len(SUPPORTED_ANGLES) - 1 else 0
                ang_idx_pub.publish(angle_index)

                capture_start = time.time() + 1.
                angle_list = []
                delay_list = []
                bar = progressbar.ProgressBar(max_value=N_CAPTURE_SAMPLES)
                print 'analyzing (%d) %d' % (angle_index, SUPPORTED_ANGLES[angle_index])

    
if __name__ == '__main__':
    try:
        rospy.init_node(NODE_NAME, anonymous=True)
        run_analysis()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)


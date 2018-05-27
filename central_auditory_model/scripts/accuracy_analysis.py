#!/usr/bin/env python
import numpy as np
import collections
import threading
import time
import os

import rospy, rospkg
from std_msgs.msg import UInt8
from std_msgs.msg import String
from std_msgs.msg import Header
from central_auditory_model.msg import AngleEstimation


NODE_NAME = 'accuracy_analysis'
ANGLE_INDEX_TOPIC_NAME = '/binaural_audio/wave_stereo/angle_index'
FILE_PATH_NAME_TOPIC_NAME = '/binaural_audio/wave_stereo/file_path_name'
SUB_TOPIC_NAME = '/central_auditory_model/ic_stream/angle_estimation'

N_CAPTURE_SAMPLES = 400
SUPPORTED_ANGLES = [90, 60, 30, 0, 330, 300, 270]


def run_analysis():
    ang_idx_pub = rospy.Publisher(ANGLE_INDEX_TOPIC_NAME, UInt8, queue_size=1)
    file_pn_pub = rospy.Publisher(FILE_PATH_NAME_TOPIC_NAME, String, queue_size=1)

    capture_q = collections.deque(maxlen=N_CAPTURE_SAMPLES)
    capture_event = threading.Event()
    capture_event.set()
    capture_start = time.time()

    def angle_estim_cb(data):
        if rospy.is_shutdown():
            capture_event.set()

        if not capture_event.is_set():
            if data.timecode.to_sec() > capture_start:
                capture_q.append(data)
                if len(capture_q) >= N_CAPTURE_SAMPLES:
                    capture_event.set()
            else:
                pass
                # print 'skip old msg.'
        else:
            pass
            # print 'capture disabled.'

    rospy.Subscriber(SUB_TOPIC_NAME, AngleEstimation, angle_estim_cb)

    time.sleep(1.)

    rospy.loginfo('start subscribing to %s' % SUB_TOPIC_NAME)

    
    for (angle_index, angle) in enumerate(SUPPORTED_ANGLES):
        print 'analyzing (%d) %d' % (angle_index, angle)
        ang_idx_pub.publish(angle_index)
        capture_start = time.time() + 1.0
        capture_q.clear()
        capture_event.clear()

        capture_event.wait(60.)

        n_msg = len(capture_q)

        data_np = np.array([capture_q.popleft().angle_index for _ in range(n_msg)], dtype=np.int8)

        stat_result = np.histogram(data_np, bins=np.arange(len(SUPPORTED_ANGLES) + 1))[0].astype(np.float) / n_msg

        print n_msg, stat_result

        if rospy.is_shutdown():
            break



if __name__ == '__main__':
    try:
        rospy.init_node(NODE_NAME, anonymous=True)
        run_analysis()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)


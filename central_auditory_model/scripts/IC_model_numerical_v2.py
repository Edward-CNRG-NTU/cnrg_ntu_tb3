#!/usr/bin/env python
import numpy as np
import timeit, time
import threading
import nengo, nengo_dl
from delay_line import DelayLine
from collections import deque

import rospy
from std_msgs.msg import String
from std_msgs.msg import Header
from std_msgs.msg import UInt8
from ipem_module.msg import AuditoryNerveImage
from central_auditory_model.msg import AngleEstimation

# ROS
NODE_NAME = 'nengo_ic_model'
ANGLE_ESTIMATION_TOPIC_NAME = '/central_auditory_model/ic_stream/angle_estimation'
ANGLE_INDEX_TOPIC_NAME = '/central_auditory_model/ic_stream/angle_index'
SUB_MSO_TOPIC_NAME = '/central_auditory_model/mso_stream'
SUB_LSO_TOPIC_NAME = '/central_auditory_model/lso_stream'

# SIGNAL
MAXPOOLING_STEP = 256
SAMPLE_RATE = 11025 // MAXPOOLING_STEP
CHUNK_SIZE = 1024 / MAXPOOLING_STEP
N_SUBCHANNELS = 40

# SIMULATION
MAX_DELAY = 5.

SUPPORTED_ANGLES = [90, 60, 30, 0, 330, 300, 270]

vote_weight = np.repeat(np.repeat([[3,5,3,1]], CHUNK_SIZE, 0), [1,21,12,6], 1)
vote_stat_bin = np.arange(len(SUPPORTED_ANGLES) + 1)


def run_IC_model():

    dq_mso = deque(maxlen=MAX_DELAY * SAMPLE_RATE / CHUNK_SIZE)
    dq_lso = deque(maxlen=MAX_DELAY * SAMPLE_RATE / CHUNK_SIZE)
    event = threading.Event()
    event.clear()

    def mso_cb(data):
        if data.n_subchannels != N_SUBCHANNELS or data.sample_rate != SAMPLE_RATE:            
            rospy.logwarn('NOT IMPLEMENT YET: dynamic CHUNK_SIZE, N_SUBCHANNELS and SAMPLE_RATE not supported!')
            return
        dq_mso.append(data)
        event.set()
        # print 'mso_cb receive %f' % data.timecode.to_sec()

    def lso_cb(data):
        if data.n_subchannels != N_SUBCHANNELS or data.sample_rate != SAMPLE_RATE:            
            rospy.logwarn('NOT IMPLEMENT YET: dynamic CHUNK_SIZE, N_SUBCHANNELS and SAMPLE_RATE not supported!')
            return
        dq_lso.append(data)
        event.set()
        # print 'lso_cb receive %f' % data.timecode.to_sec()        

    rospy.Subscriber(SUB_MSO_TOPIC_NAME, AuditoryNerveImage, mso_cb)
    rospy.Subscriber(SUB_LSO_TOPIC_NAME, AuditoryNerveImage, lso_cb)

    index_pub = rospy.Publisher(ANGLE_INDEX_TOPIC_NAME, UInt8, queue_size=1)
    ang_est_pub = rospy.Publisher(ANGLE_ESTIMATION_TOPIC_NAME, AngleEstimation, queue_size=1)

    rospy.loginfo('"%s" starts subscribing to "%s" and "%s".' % (NODE_NAME, SUB_MSO_TOPIC_NAME, SUB_LSO_TOPIC_NAME))

    while not rospy.is_shutdown() and event.wait(1.0):
        event.clear()

        try:
            mso_msg = dq_mso.popleft()
        except IndexError:
            continue

        try:
            lso_msg = dq_lso.popleft()
        except IndexError:            
            dq_mso.appendleft(mso_msg)
            continue

        if mso_msg.timecode.to_sec() < lso_msg.timecode.to_sec():
            rospy.logwarn('skip 1 mso_msg: %d, %d' % (len(dq_mso), len(dq_lso)))
            dq_lso.appendleft(lso_msg)
            event.set()
            continue
        elif mso_msg.timecode.to_sec() > lso_msg.timecode.to_sec():
            rospy.logwarn('skip 1 lso_msg: %d, %d' % (len(dq_mso), len(dq_lso)))
            dq_mso.appendleft(mso_msg)
            event.set()
            continue

        if mso_msg.timecode.to_sec() == lso_msg.timecode.to_sec():
            mso_data = np.array(mso_msg.left_channel).reshape(mso_msg.shape)
            lso_data = np.array(lso_msg.left_channel).reshape(lso_msg.shape)

            # n_steps = mso_msg.shape[1]

            ic_data = np.empty_like(mso_data)

            ic_data[:, :, :18] = mso_data[:, :, :18]
            ic_data[:, :, 18:] = mso_data[:, :, 18:] * lso_data[:, :, 18:]
            # ic_data[:, :, 18:] = 5. * mso_data[:, :, 18:] * np.min(lso_data[:, :, 18:], axis=1, keepdims=True)
            # high_freq = 5. * mso_data[:, :n_steps, 18:] * np.min(lso_data[:, :, 18:], axis=1, keepdims=True)
            # ic_data = np.concatenate([low_freq, high_freq], axis=2)

            # ic_result_reduced = np.max(ic_data, axis=1)

            argmax_of_every_ch = np.argmax(ic_data, axis=0)

            print argmax_of_every_ch

            vote_result = np.histogram(argmax_of_every_ch, bins=vote_stat_bin, weights=vote_weight)[0]  # NOTICE how th bin set up.

            print vote_result

            angle_index = np.argmax(vote_result)

            ang_est_pub.publish(
                AngleEstimation(
                    header=Header(
                        frame_id=NODE_NAME,
                        stamp=rospy.Time.now()
                    ),
                    timecode=mso_msg.timecode,
                    angle_estimastion=SUPPORTED_ANGLES[angle_index],
                    angle_index=angle_index,
                    votes=vote_result,
                    supported_angles=SUPPORTED_ANGLES
                )
            )
            
            index_pub.publish(angle_index)

            rospy.loginfo('<%f> angle_estimate: (%d)%d' % (mso_msg.timecode.to_sec(), angle_index, SUPPORTED_ANGLES[angle_index]))


if __name__ == '__main__':
    try:
        rospy.init_node(NODE_NAME, anonymous=True)
        run_IC_model()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)
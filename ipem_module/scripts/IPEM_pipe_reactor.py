#!/usr/bin/env python
import numpy as np
from collections import deque
from threading import Event
from IPEM_pipe import IPEM_pipe

import rospy
from std_msgs.msg import String
from std_msgs.msg import Header
from binaural_microphone.msg import BinauralAudio
from ipem_module.msg import AuditoryNerveImageMultiDim


SAMPLE_RATE = 22050
N_SUBCHANNELS = 40
CHUNK_SIZE= 1024
FIFO_PATH = '/tmp/IPEM'
NODE_NAME = 'IPEM_pipe_reactor'
SUB_TOPIC_NAME = 'source_stream'
PUB_TOPIC_NAME = 'apm_stream'


def reactor():
    rospy.init_node(NODE_NAME, anonymous=False)

    ani_pub = rospy.Publisher(PUB_TOPIC_NAME, AuditoryNerveImageMultiDim, queue_size=1)
    ipem_L_results = deque()
    ipem_L_ready = Event()

    def ipem_L_cb(data):
        np_data = np.fromstring(data, dtype=np.float, count=N_SUBCHANNELS, sep=' ')
        ipem_L_results.append(np_data)
        if len(ipem_L_results) > CHUNK_SIZE:
            ipem_L_ready.set()

    ipem_R_results = deque()
    ipem_R_ready = Event()

    def ipem_R_cb(data):
        np_data = np.fromstring(data, dtype=np.float, count=N_SUBCHANNELS, sep=' ')
        ipem_R_results.append(np_data)
        if len(ipem_R_results) > CHUNK_SIZE:
            ipem_R_ready.set()

    ipem_L = IPEM_pipe(new_ani_callback=ipem_L_cb, sample_frequency=SAMPLE_RATE, fifo_path=FIFO_PATH + '/L')
    ipem_R = IPEM_pipe(new_ani_callback=ipem_R_cb, sample_frequency=SAMPLE_RATE, fifo_path=FIFO_PATH + '/R')

    def feed_ipem(data):
        rospy.loginfo(data.header.seq)
        if data.sample_rate != SAMPLE_RATE:
            raise TypeError('sample_rate missmatch!')
        ipem_L.feed_pcm_samples(np.array(data.left_channel, dtype=np.int16).tobytes())
        ipem_R.feed_pcm_samples(np.array(data.right_channel, dtype=np.int16).tobytes())

    rospy.Subscriber(SUB_TOPIC_NAME, BinauralAudio, feed_ipem)
    rospy.loginfo('"%s" starts subscribing to "%s".' % (NODE_NAME, SUB_TOPIC_NAME))

    try:
        # rospy.spin()
        while not rospy.is_shutdown():
            while ipem_L_ready.wait(1) and ipem_R_ready.wait(1):
                ipem_L_np = np.stack([ipem_L_results.popleft() for i in range(CHUNK_SIZE)], axis=0)
                ipem_R_np = np.stack([ipem_R_results.popleft() for i in range(CHUNK_SIZE)], axis=0)

                ipem_L_ready.clear()
                ipem_R_ready.clear()

                ipem_np = np.stack([ipem_L_np, ipem_R_np], axis=0)

                ani = AuditoryNerveImageMultiDim(
                    header=Header(
                        stamp=rospy.Time.now()
                    ),
                    sample_rate=SAMPLE_RATE / 2,
                    chunk_size=CHUNK_SIZE,
                    info='(stereo, chunk_size, n_subchannels)',
                    shape=ipem_np.shape,
                    data=ipem_np.ravel()
                )
                ani_pub.publish(ani)

            rospy.loginfo('main loop time out!')
            break
    finally:
        ipem_L.close()
        ipem_R.close()


if __name__ == '__main__':
    try:
        reactor()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)

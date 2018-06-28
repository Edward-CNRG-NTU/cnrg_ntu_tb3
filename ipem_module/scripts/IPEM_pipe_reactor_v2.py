#!/usr/bin/env python
import numpy as np
from collections import deque
from threading import Event
from IPEM_pipe import IPEM_pipe
import atexit

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

SKIP_FIRST_L = 32
SKIP_FIRST_R = 32


def reactor():
    rospy.init_node(NODE_NAME, anonymous=False)

    ani_pub = rospy.Publisher(PUB_TOPIC_NAME, AuditoryNerveImageMultiDim, queue_size=1)
    
    ipem_L_ready = Event()
    ipem_L_q = deque()

    def ipem_L_cb(data):
        global SKIP_FIRST_L
        if SKIP_FIRST_L:
            SKIP_FIRST_L -= 1
            return

        ipem_L_q.append(np.fromstring(data, dtype=np.float, count=N_SUBCHANNELS, sep=' '))

        if (not ipem_L_ready.is_set()) and len(ipem_L_q) >= CHUNK_SIZE:
            ipem_L_ready.set()
    
    ipem_R_ready = Event()
    ipem_R_q = deque()

    def ipem_R_cb(data):
        global SKIP_FIRST_R
        if SKIP_FIRST_R:
            SKIP_FIRST_R -= 1
            return

        ipem_R_q.append(np.fromstring(data, dtype=np.float, count=N_SUBCHANNELS, sep=' '))

        if (not ipem_R_ready.is_set()) and len(ipem_R_q) >= CHUNK_SIZE:
            ipem_R_ready.set()

    ipem_L = IPEM_pipe(new_ani_callback=ipem_L_cb, sample_frequency=SAMPLE_RATE, fifo_path=FIFO_PATH + '/L')
    ipem_R = IPEM_pipe(new_ani_callback=ipem_R_cb, sample_frequency=SAMPLE_RATE, fifo_path=FIFO_PATH + '/R')

    timecode_q = deque()

    def feed_ipem(data):
        if data.sample_rate != SAMPLE_RATE:
            raise TypeError('sample_rate missmatch!')
        timecode_q.append(data.header.stamp)
        ipem_L.feed_pcm_samples(np.array(data.left_channel, dtype=np.int16).tobytes())
        ipem_R.feed_pcm_samples(np.array(data.right_channel, dtype=np.int16).tobytes())

    rospy.Subscriber(SUB_TOPIC_NAME, BinauralAudio, feed_ipem)
    rospy.loginfo('"%s" starts subscribing to "%s".' % (NODE_NAME, SUB_TOPIC_NAME))

    atexit.register(ipem_L.close)
    atexit.register(ipem_R.close)

    while not rospy.is_shutdown():
        while ipem_L_ready.wait(1) and ipem_R_ready.wait(1):
            if len(ipem_L_q) >= CHUNK_SIZE and len(ipem_R_q) >= CHUNK_SIZE:
                ipem_L_np = np.stack([ipem_L_q.popleft() for i in range(CHUNK_SIZE)], axis=0)
                ipem_R_np = np.stack([ipem_R_q.popleft() for i in range(CHUNK_SIZE)], axis=0)

                ipem_L_ready.clear()
                ipem_R_ready.clear()

                ipem_np = np.stack([ipem_L_np, ipem_R_np], axis=0)

                ani = AuditoryNerveImageMultiDim(
                    header=Header(
                        stamp=rospy.Time.now()
                    ),
                    timecode=timecode_q.popleft(),
                    sample_rate=SAMPLE_RATE / 2,
                    chunk_size=CHUNK_SIZE,
                    info='(stereo, chunk_size, n_subchannels)',
                    shape=ipem_np.shape,
                    data=ipem_np.ravel()
                )
                timecode_q.popleft()            
                ani_pub.publish(ani)

                rospy.loginfo('len(timecode_q): %d' % len(timecode_q))
                
            else:
                rospy.loginfo('ipem_L_ready or ipem_L_ready false alarm!')

        rospy.logwarn('main loop time out!')
        # break


if __name__ == '__main__':
    try:
        reactor()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)

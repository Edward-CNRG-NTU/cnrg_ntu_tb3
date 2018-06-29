#!/usr/bin/env python
import pyaudio
# import wave
import numpy as np
import time

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import String
from std_msgs.msg import Header
from binaural_microphone.msg import BinauralAudio


SAMPLE_RATE = 22050
CHUNK_SIZE= 1024
CHANNELS = 2
FORMAT = pyaudio.paInt16

TOPIC_NAME = 'source_stream'
NODE_NAME = 'pyaudio_stereo'

RMS_TOPIC_NAME = 'source_rms'


if __name__ == '__main__':

    audio = pyaudio.PyAudio()
    stream = audio.open(format=FORMAT, channels=CHANNELS,
                        rate=SAMPLE_RATE, input=True,
                        frames_per_buffer=CHUNK_SIZE)

    try:
        rospy.init_node(NODE_NAME, anonymous=False)

        raw_pub = rospy.Publisher(TOPIC_NAME, BinauralAudio, queue_size=10)
        # raw_str_pub = rospy.Publisher('/audio_stream_raw', String, queue_size=1)  # for backward compability.

        rms_L_pub = rospy.Publisher(RMS_TOPIC_NAME + '/L', Float32, queue_size=1)
        rms_R_pub = rospy.Publisher(RMS_TOPIC_NAME + '/R', Float32, queue_size=1)

        rospy.loginfo('"%s" starts publishing to "%s".' % (NODE_NAME, TOPIC_NAME))
        
        while not rospy.is_shutdown():
            raw_str = stream.read(CHUNK_SIZE)
            np_data = np.frombuffer(raw_str, dtype=np.int16).reshape([-1, 2])
            ba = BinauralAudio(
                header=Header(
                    frame_id=NODE_NAME,
                    stamp=rospy.Time.now()
                    ),
                type='PCM Int16',
                sample_rate=SAMPLE_RATE,
                chunk_size=CHUNK_SIZE,
                left_channel=np_data[:, 0],
                right_channel=np_data[:, 1]
            )
            raw_pub.publish(ba)
            # raw_str_pub.publish(raw_str)

            rms_L_pub.publish(np.sqrt(np.mean(np.square(np_data[:, 0].astype(np.float)))))
            rms_R_pub.publish(np.sqrt(np.mean(np.square(np_data[:, 1].astype(np.float)))))
    except rospy.ROSInterruptException:
        pass
    finally:
        stream.stop_stream()
        stream.close()
        audio.terminate()
        rospy.loginfo('stream.stop_stream()')
        rospy.loginfo('stream.close()')
        rospy.loginfo('audio.terminate()')

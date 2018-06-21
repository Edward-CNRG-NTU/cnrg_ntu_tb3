#!/usr/bin/env python
import wave
import numpy as np
import os

import rospy, rospkg
from std_msgs.msg import Float32
from std_msgs.msg import UInt8
from std_msgs.msg import String
from std_msgs.msg import Header
from dynamic_reconfigure.server import Server
from binaural_microphone.cfg import waveStereoConfig
from binaural_microphone.msg import BinauralAudio


NODE_NAME = 'wave_stereo'
TOPIC_NAME = 'source_stream'
RMS_TOPIC_NAME = 'source_rms'

PACKAGE_DIR_PATH = rospkg.RosPack().get_path('binaural_microphone')

ANGLE_INDEX = 3
SUPPORTED_ANGLES = [90, 60, 30, 0, 330, 300, 270]

# FILE_PATH_NAME = 'wave_stereo_db/pink_noise'
FILE_PATH_NAME = 'wave_stereo_db/coffee'
FILE_SAMPLE_RATE = 44100
DOWN_SAMPLE_FACTOR = 2

SAMPLE_RATE = 22050
CHUNK_SIZE= 1024


def wave_preprocessing(wav, stereo=False):
    assert wav.getsampwidth() == 2
    assert wav.getframerate() == FILE_SAMPLE_RATE

    wav.rewind()
    wav_str = wav.readframes(wav.getnframes())

    if not stereo:
        assert wav.getnchannels() == 1
        wav_np = np.frombuffer(wav_str, dtype=np.int16)[::DOWN_SAMPLE_FACTOR]
        n_chunks = wav_np.shape[0] // CHUNK_SIZE + 1
        return np.pad(wav_np, (0, n_chunks * CHUNK_SIZE - wav_np.shape[0],), mode='constant')
    else:
        assert wav.getnchannels() == 2    
        wav_np = np.frombuffer(wav_str, dtype=np.int16).reshape([-1, 2])[::DOWN_SAMPLE_FACTOR, :]
        n_chunks = wav_np.shape[0] // CHUNK_SIZE + 1
        data_stereo = np.pad(wav_np, ((0, n_chunks * CHUNK_SIZE - wav_np.shape[0]), (0, 0)), mode='constant')
        return data_stereo[:, 0], data_stereo[:, 1]


def load_data(file_path_name=FILE_PATH_NAME, angle_index=ANGLE_INDEX):
    global data_L, data_R, data_start, ANGLE_INDEX, FILE_PATH_NAME

    angle_index = int(angle_index)

    if 0 <= angle_index < len(SUPPORTED_ANGLES):
        ANGLE_INDEX = angle_index
        rospy.loginfo('angle_index changed to: %d (%d degree)' % (angle_index, SUPPORTED_ANGLES[angle_index]))
    else:
        rospy.logwarn('unsupported angle_index: %d' % angle_index)
        return

    fn_L = '%s_%dL.wav' % (PACKAGE_DIR_PATH + '/' + file_path_name, SUPPORTED_ANGLES[angle_index])
    fn_R = '%s_%dR.wav' % (PACKAGE_DIR_PATH + '/' + file_path_name, SUPPORTED_ANGLES[angle_index])
    fn_stereo = '%s_%d.wav' % (PACKAGE_DIR_PATH + '/' + file_path_name, SUPPORTED_ANGLES[angle_index])
    
    if os.path.isfile(fn_L) and os.path.isfile(fn_R):
        if file_path_name is not FILE_PATH_NAME:
            FILE_PATH_NAME = file_path_name
            rospy.loginfo('file changed to: %s' % file_path_name)
        data_L = wave_preprocessing(wave.open(fn_L, 'rb'))
        data_R = wave_preprocessing(wave.open(fn_R, 'rb'))
    elif os.path.isfile(fn_stereo):
        if file_path_name is not FILE_PATH_NAME:
            FILE_PATH_NAME = file_path_name
            rospy.loginfo('file changed to: %s' % file_path_name)
        (data_L, data_R) = wave_preprocessing(wave.open(fn_stereo, 'rb'), stereo=True)
    else:
        rospy.logwarn('files not found: %s, %s or %s' % (fn_L, fn_R, fn_stereo))
        return
            
    data_start = 0


def dynamic_reconfig_cb(config, level):
    rospy.loginfo('Reconfiugre Request: {angle_index}, {file_path_name}.'.format(**config))
    load_data(config['file_path_name'], config['angle_index'])
    (config['file_path_name'], config['angle_index']) = (FILE_PATH_NAME, ANGLE_INDEX)
    return config


if __name__ == '__main__':

    global data_L, data_R, data_start

    load_data()

    try:
        rospy.init_node(NODE_NAME, anonymous=False)

        reconfig_server = Server(waveStereoConfig, dynamic_reconfig_cb)

        raw_pub = rospy.Publisher(TOPIC_NAME, BinauralAudio, queue_size=1)
        # raw_str_pub = rospy.Publisher('/audio_stream_raw', String, queue_size=1)  # for backward compability.

        rms_L_pub = rospy.Publisher(RMS_TOPIC_NAME + '/L', Float32, queue_size=1)
        rms_R_pub = rospy.Publisher(RMS_TOPIC_NAME + '/R', Float32, queue_size=1)

        rospy.Subscriber('~angle_index', UInt8, lambda data: load_data(file_path_name=FILE_PATH_NAME, angle_index=data.data))
        rospy.Subscriber('~file_path_name', String, lambda data: load_data(file_path_name=data.data, angle_index=ANGLE_INDEX))

        rospy.loginfo('"%s" starts publishing to "%s".' % (NODE_NAME, TOPIC_NAME))

        rate = rospy.Rate(1.0 * SAMPLE_RATE / CHUNK_SIZE)
        
        while not rospy.is_shutdown():
            data_stop = data_start + CHUNK_SIZE

            ba = BinauralAudio(
                header=Header(
                    frame_id=NODE_NAME,
                    stamp=rospy.Time.now()
                    ),
                type='PCM Int16',
                sample_rate=SAMPLE_RATE,
                chunk_size=CHUNK_SIZE,
                left_channel=data_L[data_start:data_stop],
                right_channel=data_R[data_start:data_stop]
            )
            raw_pub.publish(ba)
            # raw_str_pub.publish(raw_str)

            rms_L_pub.publish(np.sqrt(np.mean(np.square(data_L[data_start:data_stop].astype(np.float)))))
            rms_R_pub.publish(np.sqrt(np.mean(np.square(data_R[data_start:data_stop].astype(np.float)))))

            data_start = data_stop if data_stop < data_L.shape[0] else 0
            rate.sleep()

    except rospy.ROSInterruptException as e:
        rospy.logerr(e)
    finally:
        pass

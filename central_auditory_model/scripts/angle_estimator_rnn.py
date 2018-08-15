#!/usr/bin/env python
import numpy as np
import collections
import threading
import tensorflow
import keras.backend as K
config = tensorflow.ConfigProto()
config.gpu_options.allow_growth = True 
K.tensorflow_backend.set_session(tensorflow.Session(config=config))
from keras.models import load_model

import rospy
from std_msgs.msg import String
from std_msgs.msg import Header
from std_msgs.msg import UInt8
from ipem_module.msg import AuditoryNerveImageMultiDim
from central_auditory_model.msg import AngleEstimation

# ROS
NODE_NAME = 'angle_estimator_rnn'
MSO_TOPIC_NAME = '/central_auditory_model/mso_stream'
LSO_TOPIC_NAME = '/central_auditory_model/lso_stream'
IC_TOPIC_NAME = '/central_auditory_model/ic_stream'
ANGLE_ESTIMATION_TOPIC_NAME = '/central_auditory_model/angle_estimation'
ANGLE_INDEX_TOPIC_NAME = '/central_auditory_model/angle_index'

# SIGNAL
MAXPOOLING_STEP = 64
SAMPLE_RATE = 11025 // MAXPOOLING_STEP
CHUNK_SIZE = 1024 / MAXPOOLING_STEP
N_SUBCHANNELS = 40

SUPPORTED_ANGLES = [90, 60, 30, 0, 330, 300, 270]
N_SUPPORTED_ANGLES = len(SUPPORTED_ANGLES)

model_dir = '/home/cnrg-ntu/catkin_ws/src/cnrg_ntu_tb3/central_auditory_model/auditory_model/rnn_best/'


def search_msg(queue, timecode):
    while True:
        try:
            msg = queue.popleft()
        except IndexError:
            return None

        if msg.timecode.to_sec() == timecode.to_sec():
            return msg
        elif msg.timecode.to_sec() < timecode.to_sec():
            continue
        else:
            queue.appendleft(msg)
            return None


def run_angle_estimator():    
    index_pub = rospy.Publisher(ANGLE_INDEX_TOPIC_NAME, UInt8, queue_size=10)
    ang_est_pub = rospy.Publisher(ANGLE_ESTIMATION_TOPIC_NAME, AngleEstimation, queue_size=10)

    mso_q = collections.deque(maxlen=30)
    lso_q = collections.deque(maxlen=30)
    ic_q = collections.deque(maxlen=30)
    event = threading.Event()
    event.clear()

    def generic_cb_factory(queue, trigger=False):        
        def generic_cb(data):
            queue.append(data)
            if trigger:
                event.set()
        return generic_cb

    rospy.Subscriber(MSO_TOPIC_NAME, AuditoryNerveImageMultiDim, generic_cb_factory(mso_q))
    rospy.Subscriber(LSO_TOPIC_NAME, AuditoryNerveImageMultiDim, generic_cb_factory(lso_q))
    rospy.Subscriber(IC_TOPIC_NAME, AuditoryNerveImageMultiDim, generic_cb_factory(ic_q, trigger=True))

    model = load_model(model_dir + '20180716163414_simple_rnn_model_0.85.hdf5')
    param = np.load(model_dir + '20180716163414_simple_rnn_param.npz')

    rospy.loginfo('starting "%s"' % NODE_NAME)

    while not rospy.is_shutdown():        
        if not event.wait(1.):
            rospy.logwarn('main loop timeout!')
            continue
        
        if len(ic_q) >= 3:
            ic_msg = ic_q.popleft()
        else:
            event.clear()
            continue

        mso_msg = search_msg(mso_q, ic_msg.timecode)
        lso_msg = search_msg(lso_q, ic_msg.timecode)

        if mso_msg is None or lso_msg is None:
            # ic_q.appendleft(ic_msg)
            # if mso_msg is not None:
            #     mso_q.appendleft(mso_msg)
            # if lso_msg is not None:
            #     lso_q.appendleft(lso_msg)            
            continue

        mso_data = np.swapaxes(np.array(mso_msg.data, dtype=np.float32).reshape(mso_msg.shape), 0, 1)
        lso_data = np.swapaxes(np.array(lso_msg.data, dtype=np.float32).reshape(lso_msg.shape), 0, 1)
        ic_data = np.swapaxes(np.array(ic_msg.data, dtype=np.float32).reshape(ic_msg.shape), 0, 1)
        acm_data = np.stack([mso_data, lso_data, ic_data], axis=-1).reshape(1, ic_data.shape[0], -1)

        # print mso_data.shape, lso_data.shape, ic_data.shape, acm_data.shape

        # print ic_data.shape, acm_data.shape

        x_test = (acm_data - param['X_mean']) / param['X_std']

        vote_result = model.predict(x_test, batch_size=1)[0]
        angle_index = np.argmax(vote_result)

        ae_msg = AngleEstimation(
            header=Header(
                stamp=rospy.Time.now()
            ),
            timecode=ic_msg.timecode,
            angle_estimastion=SUPPORTED_ANGLES[angle_index],
            angle_index=int(angle_index),
            votes=vote_result * 255,
            supported_angles=SUPPORTED_ANGLES
        )

        ang_est_pub.publish(ae_msg)        
        index_pub.publish(angle_index)

        rospy.loginfo('<%f> angle_estimate: (%d)%d' % (ic_msg.timecode.to_sec(), angle_index, SUPPORTED_ANGLES[angle_index]))


if __name__ == '__main__':
    try:
        rospy.init_node(NODE_NAME, anonymous=False)
        run_angle_estimator()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)

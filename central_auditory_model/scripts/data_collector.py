#!/usr/bin/env python
import numpy as np
import collections
import threading
import progressbar

import rospy, rospkg
from std_msgs.msg import UInt8
from std_msgs.msg import String
from ipem_module.msg import AuditoryNerveImageMultiDim
from central_auditory_model.msg import AngleEstimation


NODE_NAME = 'data_collector'
ANGLE_INDEX_TOPIC_NAME = '/binaural_audio/wave_stereo/angle_index'
FILE_PATH_NAME_TOPIC_NAME = '/binaural_audio/wave_stereo/file_path_name'

APM_TOPIC_NAME = '/ipem_module/apm_stream'
MSO_TOPIC_NAME = '/central_auditory_model/mso_stream'
LSO_TOPIC_NAME = '/central_auditory_model/lso_stream'
IC_TOPIC_NAME = '/central_auditory_model/ic_stream'
ANG_EST_TOPIC_NAME = '/central_auditory_model/angle_estimation'

FILE_PATH_NAME = 'wave_stereo_db/cor'
DB_FILENAME = 'db_cor'

N_CAPTURE_SAMPLES = 2000
SUPPORTED_ANGLES = [90, 60, 30, 0, 330, 300, 270]


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


def run_collector():
    ang_idx_pub = rospy.Publisher(ANGLE_INDEX_TOPIC_NAME, UInt8, queue_size=1)
    file_pn_pub = rospy.Publisher(FILE_PATH_NAME_TOPIC_NAME, String, queue_size=1)

    apm_q = collections.deque(maxlen=50)
    mso_q = collections.deque(maxlen=50)
    lso_q = collections.deque(maxlen=50)
    ic_q = collections.deque(maxlen=50)
    ang_q = collections.deque(maxlen=30)
    event = threading.Event()
    event.clear()

    def generic_cb_factory(queue, trigger=False):        
        def generic_cb(data):
            queue.append(data)
            if trigger:
                event.set()
        return generic_cb

    rospy.Subscriber(APM_TOPIC_NAME, AuditoryNerveImageMultiDim, generic_cb_factory(apm_q))
    rospy.Subscriber(MSO_TOPIC_NAME, AuditoryNerveImageMultiDim, generic_cb_factory(mso_q))
    rospy.Subscriber(LSO_TOPIC_NAME, AuditoryNerveImageMultiDim, generic_cb_factory(lso_q))
    rospy.Subscriber(IC_TOPIC_NAME, AuditoryNerveImageMultiDim, generic_cb_factory(ic_q))
    rospy.Subscriber(ANG_EST_TOPIC_NAME, AngleEstimation, generic_cb_factory(ang_q, trigger=True))
    
    rospy.sleep(1.)

    apm_list = []
    acm_list = []
    votes_list = []
    label_list = []
    timecode_list = []

    rospy.loginfo('starting %s' % NODE_NAME)

    angle_index = 0
    file_pn_pub.publish(FILE_PATH_NAME)    
    rospy.sleep(0.5)

    ang_idx_pub.publish(angle_index)
    n_msg = 0
    capture_start = rospy.get_time() + 1.
    bar = progressbar.ProgressBar(max_value=N_CAPTURE_SAMPLES)
    rospy.loginfo('collecting %s: (%d) %d' % (FILE_PATH_NAME, angle_index, SUPPORTED_ANGLES[angle_index]))

    while not rospy.is_shutdown():        
        if not event.wait(1.):
            rospy.logwarn('main loop timeout!')
            continue

        if len(ang_q) >= 5:
            ang_msg = ang_q.popleft()
        else:
            event.clear()
            continue
        
        if ang_msg.timecode.to_sec() < capture_start:
            continue
            
        label_list.append(angle_index)
        votes_list.append(np.array(ang_msg.votes, dtype=np.uint16))
        timecode_list.append(ang_msg.timecode.to_sec())

        # print len(apm_q), len(mso_q), len(lso_q), len(ic_q)

        apm_msg = search_msg(apm_q, ang_msg.timecode)
        mso_msg = search_msg(mso_q, ang_msg.timecode)
        lso_msg = search_msg(lso_q, ang_msg.timecode)
        ic_msg = search_msg(ic_q, ang_msg.timecode)

        if apm_msg is not None:
            apm_data = np.swapaxes(np.array(apm_msg.data, dtype=np.float32).reshape(apm_msg.shape), 0, 1)
            apm_list.append(apm_data)
        else:
            rospy.logerr('lost apm_msg!!!')

        if mso_msg is not None and lso_msg is not None and ic_msg is not None:
            mso_data = np.swapaxes(np.array(mso_msg.data, dtype=np.float32).reshape(mso_msg.shape), 0, 1)
            lso_data = np.swapaxes(np.array(lso_msg.data, dtype=np.float32).reshape(lso_msg.shape), 0, 1)
            ic_data = np.swapaxes(np.array(ic_msg.data, dtype=np.float32).reshape(ic_msg.shape), 0, 1)
            acm_data = np.stack([mso_data, lso_data, ic_data], axis=-1)
            acm_list.append(acm_data)
        else:
            rospy.logerr('lost mso_msg or lso_msg or ic_msg!!!')

        n_msg += 1
        bar.update(n_msg)

        if n_msg < N_CAPTURE_SAMPLES:
            continue

        bar.finish()
        
        if angle_index < len(SUPPORTED_ANGLES) - 1:
            angle_index = angle_index + 1

            ang_idx_pub.publish(angle_index)
            n_msg = 0
            capture_start = rospy.get_time() + 1.
            bar = progressbar.ProgressBar(max_value=N_CAPTURE_SAMPLES)
            rospy.loginfo('collecting %s: (%d) %d' % (FILE_PATH_NAME, angle_index, SUPPORTED_ANGLES[angle_index]))
        else:
            # print len(apm_list), len(ani_list), len(votes_list), len(label_list)

            apm_db = np.stack(apm_list, axis=0)
            acm_db = np.stack(acm_list, axis=0)
            votes_db = np.stack(votes_list, axis=0)
            label_db = np.array(label_list, dtype=np.uint8)
            timecode_db = np.array(timecode_list, dtype=np.float64)

            print apm_db.shape, acm_db.shape, votes_db.shape, label_db.shape, timecode_db.shape

            np.savez_compressed(DB_FILENAME + '.npz', apm_db=apm_db, acm_db=acm_db, votes_db=votes_db, label_db=label_db, timecode_db=timecode_db)
            rospy.loginfo('collecting %s: done.' % FILE_PATH_NAME)
            
            break

    
if __name__ == '__main__':
    try:
        rospy.init_node(NODE_NAME, anonymous=False)
        run_collector()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)


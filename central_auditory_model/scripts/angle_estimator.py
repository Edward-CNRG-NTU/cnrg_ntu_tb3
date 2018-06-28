#!/usr/bin/env python
import numpy as np

import rospy
from std_msgs.msg import String
from std_msgs.msg import Header
from std_msgs.msg import UInt8
from ipem_module.msg import AuditoryNerveImageMultiDim
from central_auditory_model.msg import AngleEstimation

# ROS
NODE_NAME = 'angle_estimator'
SUB_IC_TOPIC_NAME = '/central_auditory_model/ic_stream'
ANGLE_ESTIMATION_TOPIC_NAME = '/central_auditory_model/angle_estimation'
ANGLE_INDEX_TOPIC_NAME = '/central_auditory_model/angle_index'

# SIGNAL
MAXPOOLING_STEP = 64
SAMPLE_RATE = 11025 // MAXPOOLING_STEP
CHUNK_SIZE = 1024 / MAXPOOLING_STEP
N_SUBCHANNELS = 40

SUPPORTED_ANGLES = [90, 60, 30, 0, 330, 300, 270]
N_SUPPORTED_ANGLES = len(SUPPORTED_ANGLES)

vote_weight = np.repeat(np.repeat([[3,5,3,1]], CHUNK_SIZE, 0), [1,21,12,6], 1)
vote_stat_bin = np.arange(len(SUPPORTED_ANGLES) + 1)


def run_angle_estimator():
    index_pub = rospy.Publisher(ANGLE_INDEX_TOPIC_NAME, UInt8, queue_size=1)
    ang_est_pub = rospy.Publisher(ANGLE_ESTIMATION_TOPIC_NAME, AngleEstimation, queue_size=1)

    def ic_cb(data):
        ic_data = np.array(data.data).reshape(data.shape)
        argmax_of_every_ch = np.argmax(ic_data, axis=0)
        print argmax_of_every_ch

        # amp_mean = np.mean(amp_data)
        # rospy.logwarn(amp_data > amp_mean)

        # vote_result = np.histogram(argmax_of_every_ch, bins=vote_stat_bin, weights=vote_weight * (amp_data > amp_mean))[0]  # NOTICE how th bin set up.
        vote_result = np.histogram(argmax_of_every_ch, bins=vote_stat_bin, weights=vote_weight)[0]

        angle_index = np.argmax(vote_result)
        # rospy.logwarn(angle_index)

        ae_msg = AngleEstimation(
            header=Header(
                stamp=rospy.Time.now()
            ),
            timecode=data.timecode,
            angle_estimastion=SUPPORTED_ANGLES[angle_index],
            angle_index=angle_index,
            votes=vote_result,
            supported_angles=SUPPORTED_ANGLES
        )

        ang_est_pub.publish(ae_msg)        
        index_pub.publish(angle_index)

        rospy.loginfo('<%f> angle_estimate: (%d)%d' % (data.timecode.to_sec(), angle_index, SUPPORTED_ANGLES[angle_index]))

    rospy.Subscriber(SUB_IC_TOPIC_NAME, AuditoryNerveImageMultiDim, ic_cb)

    rospy.loginfo('"%s" starts subscribing to "%s".' % (NODE_NAME, SUB_IC_TOPIC_NAME))

    rospy.spin()


if __name__ == '__main__':
    try:
        rospy.init_node(NODE_NAME, anonymous=True)
        run_angle_estimator()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)

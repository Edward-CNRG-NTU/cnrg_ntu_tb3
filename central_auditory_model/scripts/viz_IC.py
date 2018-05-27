#!/usr/bin/env python
import numpy as np
# import numba
import time

import rospy, tf
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, Float32, ColorRGBA, Int16MultiArray


SUPPORTED_ANGLES = [90, 60, 30, 0, 330, 300, 270]

NODE_NAME = 'viz_IC'
SUB_TOPIC_NAME = '/central_auditory_model/ic_stream/vote'
PUB_TOPIC_NAME = '/visualization_marker'

FRAME_ID = '/map'


def visualizer():
    rospy.init_node(NODE_NAME, anonymous=False)
    marker_publisher = rospy.Publisher(PUB_TOPIC_NAME, Marker, queue_size=10)

    def ic_cb(data):
        t1 = time.time()
        angle_estimate = np.argmax(data.data)
        # confidence = float(data.data[angle_estimate]) / sum(data.data)
        # marker = Marker(
        #     header=Header(frame_id=FRAME_ID),
        #     ns=NODE_NAME,
        #     id=2,
        #     type=Marker.ARROW,
        #     action=Marker.ADD,
        #     pose=Pose(Point(0.0, 0.0, 0.0), Quaternion(*tf.transformations.quaternion_from_euler(0., 0., SUPPORTED_ANGLES[angle_estimate] * np.pi / 180.))),
        #     scale=Vector3(confidence, 0.1, 0.1),
        #     color=ColorRGBA(1.0, 0.0, 0.0, 1.0),
        #     lifetime=rospy.Duration(0.1),
        #     frame_locked=True,
        # )
        # marker_publisher.publish(marker)

        all_votes = sum(data.data)

        for (i, vote) in enumerate(data.data):
            confidence = max(float(vote) / all_votes, 0.001)
            # confidence = 1
            marker = Marker(
                header=Header(frame_id=FRAME_ID),
                ns=NODE_NAME,
                id=2 + i,
                type=Marker.ARROW,
                action=Marker.ADD,
                pose=Pose(Point(0.0, 0.0, 0.0), Quaternion(*tf.transformations.quaternion_from_euler(0., 0., SUPPORTED_ANGLES[i] * np.pi / 180.))),
                scale=Vector3(1. * confidence, 0.1 * confidence, 0.1 * confidence),
                color=ColorRGBA(0.0, 0.0, 1.0, 0.3),
                lifetime=rospy.Duration(0.1),
                frame_locked=True,
            )

            if i == angle_estimate:
                marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)

            marker_publisher.publish(marker)

        rospy.loginfo(time.time() - t1)
        

    rospy.Subscriber(SUB_TOPIC_NAME, Int16MultiArray, ic_cb)

    rospy.loginfo('start subscribing to %s' % SUB_TOPIC_NAME)

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        marker = Marker(
            header=Header(frame_id=FRAME_ID),
            ns=NODE_NAME,
            id=0,
            type=Marker.TEXT_VIEW_FACING,
            action=Marker.ADD,
            pose=Pose(Point(0.0, 0, -0.05), Quaternion(0., 0., 0., 1.)),
            scale=Vector3(0.1, 0.1, 0.1),
            color=ColorRGBA(1.0, 1.0, 1.0, 1.0),
            lifetime=rospy.Duration(1.0),
            frame_locked=True,
            text=NODE_NAME
        )
        marker_publisher.publish(marker)

        marker = Marker(
            header=Header(frame_id=FRAME_ID),
            ns=NODE_NAME,
            id=1,
            type=Marker.ARROW,
            action=Marker.ADD,
            pose=Pose(Point(0.0, 0.0, 0.0), Quaternion(
                *tf.transformations.quaternion_from_euler(0., 0., 0.))),
            scale=Vector3(1, 0.1, 0.1),
            color=ColorRGBA(0.0, 1.0, 0.0, 0.3),
            lifetime=rospy.Duration(1.0),
            frame_locked=True,
        )
        marker_publisher.publish(marker)
        rate.sleep()        


if __name__ == '__main__':
    try:
        visualizer()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)

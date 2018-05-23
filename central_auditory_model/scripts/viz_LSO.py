#!/usr/bin/env python
import numpy as np
# import numba
import time

import rospy, tf
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, Float32, ColorRGBA, Int16MultiArray
from ipem_module.msg import AuditoryNerveImage


SUPPORTED_ANGLES = [90, 60, 30, 0, 330, 300, 270]

NODE_NAME = 'viz_LSO'
SUB_TOPIC_NAME = '/central_auditory_model/lso_stream'
PUB_TOPIC_NAME = '/visualization_marker'

FRAME_ID = '/map'

X_SPACING = 0.2
Y_SPACING = 0.1
Z_SPACING = 0.05
LIFETIME = 0.1


def hsva_to_rgba(h, s, v, a=1.0):
    if s == 0.0:
        return v, v, v, a
    i = int(h*6.0) # XXX assume int() truncates!
    f = (h*6.0) - i
    p = v*(1.0 - s)
    q = v*(1.0 - s*f)
    t = v*(1.0 - s*(1.0-f))
    i = i%6
    if i == 0:
        return v, t, p, a
    if i == 1:
        return q, v, p, a
    if i == 2:
        return p, v, t, a
    if i == 3:
        return p, q, v, a
    if i == 4:
        return t, p, v, a
    if i == 5:
        return v, p, q, a


def visualizer():
    rospy.init_node(NODE_NAME, anonymous=False)
    marker_publisher = rospy.Publisher(PUB_TOPIC_NAME, Marker, queue_size=1)

    def mso_cb(data):
        t1 = time.time()
        data_np = np.array(data.left_channel).reshape(data.shape)

        points = [Point(t * X_SPACING, 0.85 - i * Y_SPACING, ch * Z_SPACING) for i in range(data_np.shape[0]) for t in range(data_np.shape[1]) for ch in range(data_np.shape[2])]
        colors = [ColorRGBA(*hsva_to_rgba(1., 0., 1., np.clip(data_np[i, t, ch], 0, 0.3))) for i in range(data_np.shape[0]) for t in range(data_np.shape[1]) for ch in range(data_np.shape[2])]

        marker = Marker(
            header=Header(frame_id=FRAME_ID),
            ns=NODE_NAME,
            id=2,
            type=Marker.POINTS,
            action=Marker.ADD,
            pose=Pose(Point(0.0, 2.0, 0.0), Quaternion(0, 0, 0, 1)),
            scale=Vector3(0.05, 0.05, 0.0),
            # color=None,
            lifetime=rospy.Duration(LIFETIME),
            frame_locked=True,
            points = points,
            colors = colors
            )
        marker_publisher.publish(marker)

        rospy.loginfo(time.time() - t1)

    rospy.Subscriber(SUB_TOPIC_NAME, AuditoryNerveImage, mso_cb)

    rospy.loginfo('start subscribing to %s' % SUB_TOPIC_NAME)

    # rospy.spin()

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        marker = Marker(
            header=Header(frame_id=FRAME_ID),
            ns=NODE_NAME,
            id=1,
            type=Marker.TEXT_VIEW_FACING,
            action=Marker.ADD,
            pose=Pose(Point(0.0, 2.5, -0.05), Quaternion(0., 0., 0., 1.)),
            scale=Vector3(0.1, 0.1, 0.1),
            color=ColorRGBA(1.0, 1.0, 1.0, 1.0),
            lifetime=rospy.Duration(1.0),
            frame_locked=True,
            text=NODE_NAME
        )
        marker_publisher.publish(marker)
        rate.sleep()


if __name__ == '__main__':
    try:
        visualizer()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)

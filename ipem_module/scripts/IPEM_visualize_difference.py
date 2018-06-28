#!/usr/bin/env python
import numpy as np
# import numba
import time

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from binaural_microphone.msg import BinauralAudio
from ipem_module.msg import AuditoryNerveImageMultiDim


NODE_NAME = 'ipem_visualize_difference'
SUB_TOPIC_NAME = 'apm_stream'
PUB_TOPIC_NAME = '/visualization_marker'
CHUNK_SIZE = 1024
RENDER_STRIDE = 8
N_SUBCHANNELS = 40
SAMPLE_RATE = 11025
X_SPACING = 0.01
Y_SPACING = 0.1
Y_SPLIT_SPACING = 1.0
Z_SCALING = 5


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
    
    def setup_objects():
        global pos, clr, history, points, colors
        t1 = time.time()
        pos = np.zeros([2, CHUNK_SIZE, N_SUBCHANNELS, 3])        
        pos[:, :, :, 0] = np.arange(CHUNK_SIZE).reshape([CHUNK_SIZE, 1]) * X_SPACING + np.zeros(N_SUBCHANNELS) - CHUNK_SIZE * X_SPACING / 2
        pos[:, :, :, 1] = np.arange(N_SUBCHANNELS) * Y_SPACING + Y_SPLIT_SPACING / 2
        pos[0, :, :, 1] += (- Y_SPLIT_SPACING - N_SUBCHANNELS * Y_SPACING)

        history = np.zeros([2, CHUNK_SIZE, N_SUBCHANNELS])

        clr = np.zeros([2, CHUNK_SIZE, N_SUBCHANNELS, 4])
        for j in range(N_SUBCHANNELS):
            clr[:, :, j, :] = np.array(hsva_to_rgba(0.67 - 0.67 * j / N_SUBCHANNELS, 1.0, 1.0, 1.0))        

        points = [Point(*pos[s, i, j]) for s in range(2) for i in range(0, CHUNK_SIZE, RENDER_STRIDE) for j in range(N_SUBCHANNELS)]
        colors = [ColorRGBA(*clr[s, i, j]) for s in range(2) for i in range(0, CHUNK_SIZE, RENDER_STRIDE) for j in range(N_SUBCHANNELS)]
        
        rospy.loginfo('setup_objects() done it %f sec.' % (time.time() - t1))

    # @numba.jit(parallel=True)
    def update_height(z_0, z_1):
        pos[0, :, :, 2] = z_0 * Z_SCALING
        pos[1, :, :, 2] = z_1 * Z_SCALING
        clr[0, :, :, 3] = np.clip(np.abs(z_0) * 3, 0., 1.)
        clr[1, :, :, 3] = np.clip(np.abs(z_1) * 3, 0., 1.)

    setup_objects()

    def ani_cb(data):        
        global CHUNK_SIZE, N_SUBCHANNELS, SAMPLE_RATE
        t1 = time.time()
        if data.shape[1] != CHUNK_SIZE or data.shape[2] != N_SUBCHANNELS or data.sample_rate != SAMPLE_RATE:
            CHUNK_SIZE = data.shape[1]
            N_SUBCHANNELS = data.shape[2]
            SAMPLE_RATE = data.sample_rate
            setup_objects()
            return

        ani_np = np.array(data.data).reshape(data.shape)

        global history
        history = np.roll(history, 1, axis=1)
        # history[:, 1:, :] = history[:, :-1, :]
        history[0, 0] = np.mean(ani_np[0] + ani_np[1], axis=0)
        history[1, 0] = np.mean(ani_np[0] - ani_np[1], axis=0) * 10

        update_height(history[0], history[1])

        for s in range(2):        
            for i in range(0, CHUNK_SIZE, RENDER_STRIDE):
                for j in range(N_SUBCHANNELS):                
                    points[(s * CHUNK_SIZE + i) * N_SUBCHANNELS / RENDER_STRIDE + j].z = pos[s, i, j, 2]
                    colors[(s * CHUNK_SIZE + i) * N_SUBCHANNELS / RENDER_STRIDE + j].a = clr[s, i, j, 3]

        marker = Marker(
            header=Header(frame_id='/map'),
            ns=NODE_NAME,
            id=1,
            type=Marker.POINTS,
            action=Marker.ADD,
            pose=Pose(Point(0.0, 0.0, 0.0), Quaternion(0, 0, 0, 1)),
            scale=Vector3(0.05, 0.05, 0.05),
            color=None,
            lifetime=rospy.Duration(CHUNK_SIZE / SAMPLE_RATE),
            frame_locked=False,
            points = points,
            colors = colors
            )
        marker_publisher.publish(marker)
        rospy.loginfo('loading: %2f%%' % (100 * (time.time() - t1) * SAMPLE_RATE / CHUNK_SIZE))
    
    rospy.Subscriber(SUB_TOPIC_NAME, AuditoryNerveImageMultiDim, ani_cb)

    r = rospy.Rate(1.)

    while not rospy.is_shutdown():        
        marker = Marker(
                header=Header(frame_id='/map'),
                ns=NODE_NAME,
                id=0,
                type=Marker.TEXT_VIEW_FACING,                
                lifetime=rospy.Duration(0.5),
                pose=Pose(Point(0, 0, 1), Quaternion(0, 0, 0, 1)),
                scale=Vector3(0.2, 0.2, 0.2),
                color=ColorRGBA(1.0, 0.0, 0.0, 1.0),
                text=NODE_NAME)
        marker_publisher.publish(marker)
        r.sleep()

if __name__ == '__main__':
    try:
        visualizer()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)

#!/usr/bin/env python
import numpy as np
# import numba
import time

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, Float32, ColorRGBA
from binaural_microphone.msg import BinauralAudio
from ipem_module.msg import AuditoryNerveImageMultiDim


NODE_NAME = 'ipem_visualize_stat'
SUB_TOPIC_NAME = 'apm_stream'
PUB_TOPIC_NAME = '/visualization_marker'
CHUNK_SIZE = 1024
RENDER_STRIDE = 1
N_SUBCHANNELS = 40
SAMPLE_RATE = 11025
X_SPACING = 0.1
Y_SPACING = 0.1
Y_SPLIT_SPACING = 1.0
Z_SCALING = 5

AMPL_START = 0.001
AMPL_STOP = 1.0
FREQ_START = 100
FREQ_STOP = 10000
N_STEP = 50
T_STEP = 1.2

SAVE_DIR = '/home/cnrg-ntu/catkin_ws/temp/IPEM_response_analysis/N50_2/'


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
    freq_publisher = rospy.Publisher('/binaural_audio/sine_sweep/frequency', Float32, queue_size=1)
    ampl_publisher = rospy.Publisher('/binaural_audio/sine_sweep/amplitude', Float32, queue_size=1)

    freq_publisher.publish(FREQ_START)
    ampl_publisher.publish(AMPL_START)

    rospy.sleep(1.0)
    
    def setup_objects():
        global pos, clr, statistic, points, colors
        t1 = time.time()
        pos = np.zeros([2, N_STEP, N_SUBCHANNELS, 3])        
        pos[:, :, :, 0] = np.arange(N_STEP).reshape([N_STEP, 1]) * X_SPACING + np.zeros(N_SUBCHANNELS) - N_STEP * X_SPACING / 2
        pos[:, :, :, 1] = np.arange(N_SUBCHANNELS) * Y_SPACING + Y_SPLIT_SPACING / 2
        pos[0, :, :, 1] += (- Y_SPLIT_SPACING - N_SUBCHANNELS * Y_SPACING)

        statistic = np.zeros([2, N_STEP, N_STEP, N_SUBCHANNELS, 3])

        clr = np.zeros([2, N_STEP, N_SUBCHANNELS, 4])
        for j in range(N_SUBCHANNELS):
            clr[:, :, j, :] = np.array(hsva_to_rgba(0.67 - 0.67 * j / N_SUBCHANNELS, 1.0, 1.0, 1.0))        

        points = [Point(*pos[s, i, j]) for s in range(2) for i in range(0, N_STEP, RENDER_STRIDE) for j in range(N_SUBCHANNELS)]
        colors = [ColorRGBA(*clr[s, i, j]) for s in range(2) for i in range(0, N_STEP, RENDER_STRIDE) for j in range(N_SUBCHANNELS)]
        
        rospy.loginfo('setup_objects() done in %f sec.' % (time.time() - t1))

    setup_objects()

    all_L = []
    all_R = []
    
    freq_index = 0
    freq = np.geomspace(FREQ_START, FREQ_STOP, N_STEP)

    ampl_index = 0
    ampl = np.geomspace(AMPL_START, AMPL_STOP, N_STEP)

    def ani_cb(data):
        t1 = time.time()
        if data.shape[1] != CHUNK_SIZE or data.shape[2] != N_SUBCHANNELS or data.sample_rate != SAMPLE_RATE:
            rospy.logerr('not supported setup.')
            return

        ani_np = np.array(data.data).reshape(data.shape)

        all_L.append(ani_np[0])
        all_R.append(ani_np[1])

        try:
            all_L_np = np.concatenate(all_L, axis=0)
            all_R_np = np.concatenate(all_R, axis=0)
        except ValueError:
            return

        statistic[0, ampl_index, freq_index, :, 0] = np.mean(all_L_np, axis=0)
        statistic[1, ampl_index, freq_index, :, 0] = np.mean(all_R_np, axis=0)
        statistic[0, ampl_index, freq_index, :, 1] = np.std(all_L_np, axis=0)
        statistic[1, ampl_index, freq_index, :, 1] = np.std(all_R_np, axis=0)
        statistic[0, ampl_index, freq_index, :, 2] = np.max(all_L_np, axis=0)
        statistic[1, ampl_index, freq_index, :, 2] = np.max(all_R_np, axis=0)

        pos[0, :, :, 2] = statistic[0, ampl_index, :, :, 0] * Z_SCALING
        pos[1, :, :, 2] = statistic[1, ampl_index, :, :, 0] * Z_SCALING
        clr[0, :, :, 3] = np.clip(statistic[0, ampl_index, :, :, 0] * 3, 0., 1.)
        clr[1, :, :, 3] = np.clip(statistic[1, ampl_index, :, :, 0] * 3, 0., 1.)        

        for s in range(2):        
            for i in range(0, N_STEP, RENDER_STRIDE):
                for j in range(N_SUBCHANNELS):                
                    points[(s * N_STEP + i) * N_SUBCHANNELS / RENDER_STRIDE + j].z = pos[s, i, j, 2]
                    colors[(s * N_STEP + i) * N_SUBCHANNELS / RENDER_STRIDE + j].a = clr[s, i, j, 3]

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

    rate = rospy.Rate(1./T_STEP)

    while not rospy.is_shutdown():
        freq_publisher.publish(freq[freq_index])
        ampl_publisher.publish(ampl[ampl_index])

        rospy.sleep(0.2)
        
        all_L = []
        all_R = []

        marker = Marker(
                header=Header(frame_id='/map'),
                ns=NODE_NAME,
                id=0,
                type=Marker.TEXT_VIEW_FACING,                
                lifetime=rospy.Duration(T_STEP),
                pose=Pose(Point(0, 0, 1), Quaternion(0, 0, 0, 1)),
                scale=Vector3(0.2, 0.2, 0.2),
                color=ColorRGBA(1.0, 0.0, 0.0, 1.0),
                text='%f, %f' % (ampl[ampl_index], freq[freq_index]))
        marker_publisher.publish(marker)

        rate.sleep()

        if freq_index < N_STEP - 1:
            freq_index += 1
        else:
            freq_index = 0
            if ampl_index < N_STEP - 1:
                ampl_index += 1
            else:
                ampl_index = 0
                np.save(SAVE_DIR + 'freq.npy', freq)
                np.save(SAVE_DIR + 'ampl.npy', ampl)
                np.save(SAVE_DIR + 'statistic.npy', statistic)
                exit(0)


if __name__ == '__main__':
    try:        
        visualizer()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)

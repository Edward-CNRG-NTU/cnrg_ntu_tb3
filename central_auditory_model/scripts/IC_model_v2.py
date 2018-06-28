#!/usr/bin/env python
import numpy as np
import timeit, time
import threading
import nengo, nengo_dl
from collections import deque

import rospy
from std_msgs.msg import Header
from ipem_module.msg import AuditoryNerveImageMultiDim

# ROS
NODE_NAME = 'nengo_ic_model'
SUB_MSO_TOPIC_NAME = '/central_auditory_model/mso_stream'
SUB_LSO_TOPIC_NAME = '/central_auditory_model/lso_stream'
PUB_IC_TOPIC_NAME = '/central_auditory_model/ic_stream'

# SIGNAL
MAXPOOLING_STEP = 64
SAMPLE_RATE = 11025 // MAXPOOLING_STEP
CHUNK_SIZE = 1024 / MAXPOOLING_STEP
N_SUBCHANNELS = 40

# SIMULATION
MAX_DELAY = 5.

SUPPORTED_ANGLES = [90, 60, 30, 0, 330, 300, 270]
N_SUPPORTED_ANGLES = len(SUPPORTED_ANGLES)

synapse_node_ens = 0
synapse_ens_node = 0
synapse_probe = 0

seed = 6666


def build_nengo_model():
    lifrate_model = nengo.LIFRate(tau_rc=0.002, tau_ref=0.0002)
    max_r = nengo.dists.Uniform(1000, 2000)

    def output_function_factory(channel):        
        def output_function(x):
            if channel < 18:
                return x[0]
            else:
                return x[0] * x[1]
        return output_function

    with nengo.Network(label="IC_Jeffress_Model") as model:
        n_neurons = 64

        input_mso = nengo.Node([0] * 40, label='input_node_mso')
        input_lso = nengo.Node([0] * 40, label='input_node_lso')

        ens_combine_list = [nengo.Ensemble(n_neurons, 2, radius=3., neuron_type=lifrate_model, max_rates=max_r, label='ens_combine_%d' % i, seed=seed) for i in range(N_SUBCHANNELS)]
        
        output_node = nengo.Node(size_in=N_SUBCHANNELS, label='output_node')
        
        for i in range(N_SUBCHANNELS):
            nengo.Connection(input_mso[i], ens_combine_list[i][0], synapse=synapse_node_ens)
            nengo.Connection(input_lso[i], ens_combine_list[i][1], synapse=synapse_node_ens)

            nengo.Connection(ens_combine_list[i], output_node[i], synapse=synapse_ens_node, function=output_function_factory(channel=i))
        
        output_probe = nengo.Probe(output_node, label='output_probe', synapse=synapse_probe)  # , sample_every=0.01

        nengo_dl.configure_settings(session_config={"gpu_options.allow_growth": True})
        simulator = nengo_dl.Simulator(model, dt=(1. / SAMPLE_RATE), unroll_simulation=CHUNK_SIZE, minibatch_size=N_SUPPORTED_ANGLES)

    return simulator, input_mso, input_lso, output_probe   


def run_IC_model():
    
    sim, in_mso, in_lso, out_probe = build_nengo_model()

    dq_mso = deque(maxlen=MAX_DELAY * SAMPLE_RATE / CHUNK_SIZE)
    dq_lso = deque(maxlen=MAX_DELAY * SAMPLE_RATE / CHUNK_SIZE)
    event = threading.Event()
    event.clear()

    def mso_cb(data):
        if data.shape[2] != N_SUBCHANNELS or data.sample_rate != SAMPLE_RATE:            
            rospy.logwarn('NOT IMPLEMENT YET: dynamic CHUNK_SIZE, N_SUBCHANNELS and SAMPLE_RATE not supported!')
            return
        dq_mso.append(data)
        event.set()
        # print 'mso_cb receive %f' % data.timecode.to_sec()

    def lso_cb(data):
        if data.shape[2] != N_SUBCHANNELS or data.sample_rate != SAMPLE_RATE:            
            rospy.logwarn('NOT IMPLEMENT YET: dynamic CHUNK_SIZE, N_SUBCHANNELS and SAMPLE_RATE not supported!')
            return
        dq_lso.append(data)
        event.set()
        # print 'lso_cb receive %f' % data.timecode.to_sec()        

    rospy.Subscriber(SUB_MSO_TOPIC_NAME, AuditoryNerveImageMultiDim, mso_cb)
    rospy.Subscriber(SUB_LSO_TOPIC_NAME, AuditoryNerveImageMultiDim, lso_cb)

    ic_pub = rospy.Publisher(PUB_IC_TOPIC_NAME, AuditoryNerveImageMultiDim, queue_size=1)

    rospy.loginfo('"%s" starts subscribing to "%s" and "%s".' % (NODE_NAME, SUB_MSO_TOPIC_NAME, SUB_LSO_TOPIC_NAME))

    while not rospy.is_shutdown() and event.wait(1.0):
        event.clear()
        t2 = timeit.default_timer()

        try:
            mso_msg = dq_mso.popleft()
        except IndexError:
            continue

        try:
            lso_msg = dq_lso.popleft()
        except IndexError:            
            dq_mso.appendleft(mso_msg)
            continue

        if mso_msg.timecode.to_sec() < lso_msg.timecode.to_sec():
            rospy.logwarn('skip 1 mso_msg: %d, %d' % (len(dq_mso), len(dq_lso)))
            dq_lso.appendleft(lso_msg)
            event.set()
            continue
        elif mso_msg.timecode.to_sec() > lso_msg.timecode.to_sec():
            rospy.logwarn('skip 1 lso_msg: %d, %d' % (len(dq_mso), len(dq_lso)))
            dq_mso.appendleft(mso_msg)
            event.set()
            continue

        if mso_msg.timecode.to_sec() == lso_msg.timecode.to_sec():
            mso_data = np.array(mso_msg.data).reshape(mso_msg.shape)
            lso_data = np.array(lso_msg.data).reshape(lso_msg.shape)
            # amp_data = np.array(lso_msg.right_channel).reshape((16, 40))

            sim.run_steps(CHUNK_SIZE, progress_bar=False, input_feeds={in_mso: mso_data, in_lso: lso_data})

            ic_data = sim.model.params[out_probe][-1]
            del sim.model.params[out_probe][-1]

            ic_msg = AuditoryNerveImageMultiDim(
                header=Header(
                    stamp=rospy.Time.now()
                ),
                timecode=mso_msg.timecode,
                sample_rate=SAMPLE_RATE,
                chunk_size=CHUNK_SIZE,
                shape=ic_data.shape,
                info='(direction, chunk_size, n_subchannels)',
                data=ic_data.ravel(),
            )

            ic_pub.publish(ic_msg)
            rospy.loginfo('[%f] ran %d steps in %5.3f sec.' % (mso_msg.timecode.to_sec(), CHUNK_SIZE, timeit.default_timer() - t2))


if __name__ == '__main__':
    try:
        rospy.init_node(NODE_NAME, anonymous=True)
        run_IC_model()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)

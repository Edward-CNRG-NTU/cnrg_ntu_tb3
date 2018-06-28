#!/usr/bin/env python
import numpy as np
import time
import threading
import nengo
from delay_line import DelayLine
from signal_collector import SignalCollector

import rospy
from std_msgs.msg import String
from std_msgs.msg import Header
# from binaural_microphone.msg import BinauralAudio
from ipem_module.msg import AuditoryNerveImageMultiDim


NODE_NAME = 'nengo_mso_model'
PUB_TOPIC_NAME = '/central_auditory_model/mso_stream'
SUB_TOPIC_NAME = '/ipem_module/apm_stream'
SAMPLE_RATE = 11025
CHUNK_SIZE = 1024
N_SUBCHANNELS = 40
MAX_DELAY = 5.
MAX_STEPS = int(MAX_DELAY * SAMPLE_RATE)

DELAY_VALUES = [0., 9.e-5, 26e-5, 41e-5, 56e-5, 73e-5, 82e-5]
N_DELAY_VAL = len(DELAY_VALUES)
MAXPOOLING = 100
# DELAY_VALUES = [0., 41e-5, 82e-5]


synapse1 = None
synapse2 = 0
synapse3 = 0
synapse4 = None

radius_1 = 1
radius_2 = 2        


def run_MSO_model():

    dl_L = DelayLine((N_SUBCHANNELS,), SAMPLE_RATE, initial_value=0.05, max_delay=MAX_DELAY)
    dl_R = DelayLine((N_SUBCHANNELS,), SAMPLE_RATE, initial_value=0.05, max_delay=MAX_DELAY)

    mso_pub = rospy.Publisher(PUB_TOPIC_NAME, AuditoryNerveImageMultiDim, queue_size=1)

    ani_L = np.zeros([CHUNK_SIZE, N_SUBCHANNELS])
    ani_R = np.zeros([CHUNK_SIZE, N_SUBCHANNELS])
    ani_L_1d = ani_L.ravel() # create an 1-D view into ani_L, must use ravel().
    ani_R_1d = ani_R.ravel() # create an 1-D view into ani_L, must use ravel().

    event = threading.Event()

    def ani_cb(data):        
        if data.shape[1] != CHUNK_SIZE or data.shape[2] != N_SUBCHANNELS or data.sample_rate != SAMPLE_RATE:            
            rospy.logwarn('NOT IMPLEMENT YET: dynamic SRC_CHUNK_SIZE, N_SUBCHANNELS and SAMPLE_RATE not supported!')
            return
        
        ani_data = np.array(data.data).reshape(data.shape)

        dl_L.update(ani_data[0], timecode=data.timecode)
        dl_R.update(ani_data[1])
        event.set()

    mso_msg = AuditoryNerveImageMultiDim(sample_rate=SAMPLE_RATE / MAXPOOLING, chunk_size=N_DELAY_VAL)

    def mso_output_cb(t, x):
        mso_msg.header = Header(stamp=rospy.Time.now())
        mso_msg.data = x.ravel()
        mso_pub.publish(mso_msg)

    sc = SignalCollector((N_DELAY_VAL, N_SUBCHANNELS), mso_output_cb, initial_value=0.05, maxpooling=100)

    def build_model():
        lifrate_model = nengo.LIFRate(tau_rc=0.002, tau_ref=0.0002)
        max_r = nengo.dists.Uniform(1000, 2000)

        def MSO_ensemble_array(dim, radius, label='ens_arr'):
            return nengo.networks.EnsembleArray(n_neurons=32, n_ensembles=dim, ens_dimensions=1, label=label,
                                                radius=radius, neuron_type=lifrate_model, max_rates=max_r, seed=None)

        model = nengo.Network(label="MSO_Jeffress_Model_Multi_Channel")
        with model:
            input_node_L = []
            input_node_R = []
            ens_arr_L = []
            ens_arr_R = []
            ens_arr_add = []
            output_node = []

            dim = N_SUBCHANNELS

            for i in range(N_DELAY_VAL):                
                input_node_L.append(nengo.Node(dl_L.delayed_view(DELAY_VALUES[i]), 0, dim, 'input_node_L[%d]' % i))
                input_node_R.append(nengo.Node(dl_R.delayed_view(DELAY_VALUES[i]), 0, dim, 'input_node_R[%d]' % i))
                ens_arr_L.append(MSO_ensemble_array(dim, radius_1, 'ens_arr_L[%d]' % i))
                ens_arr_R.append(MSO_ensemble_array(dim, radius_1, 'ens_arr_R[%d]' % i))
                ens_arr_add.append(MSO_ensemble_array(dim, radius_2, 'ens_arr_add[%d]' % i))
                output_node.append(nengo.Node(sc.updater_factory(i), N_SUBCHANNELS, 0, 'output_node[%d]' % i))

            for i in range(N_DELAY_VAL):
                nengo.Connection(input_node_L[i],   ens_arr_L[i].input, synapse=synapse2)
                nengo.Connection(input_node_R[i],   ens_arr_R[i].input, synapse=synapse2)
                nengo.Connection(ens_arr_L[i].output, ens_arr_add[i].input, synapse=synapse3)
                nengo.Connection(ens_arr_R[-1-i].output, ens_arr_add[i].input, synapse=synapse3)
                nengo.Connection(ens_arr_add[i].output, output_node[i], synapse=synapse4)


        dt = 1. / SAMPLE_RATE

        # sim = nengo.Simulator(model, dt=dt)

        import nengo_dl
        sim = nengo_dl.Simulator(model, dt=dt, unroll_simulation=32)

        # import os; os.environ['PYOPENCL_CTX'] = '0'
        # import pyopencl, nengo_ocl
        # ctx = pyopencl.create_some_context()
        # sim = nengo_ocl.Simulator(model, context=ctx, dt=dt)

        return sim

    sim = build_model()
    print sim.time, sim.n_steps
    
    rospy.Subscriber(SUB_TOPIC_NAME, AuditoryNerveImageMultiDim, ani_cb)
    rospy.loginfo('"%s" starts subscribing to "%s".' % (NODE_NAME, SUB_TOPIC_NAME))

    while not rospy.is_shutdown() and event.wait(1.0):

        yet_to_run = dl_R.n_steps - sim.n_steps - CHUNK_SIZE

        if yet_to_run == 0:                
            event.clear()
        elif yet_to_run > MAX_STEPS:
            print 'delay too much!'
            break
        elif yet_to_run < 0:
            print 'yet_to_run < 0?'
            break

        t2 = time.time()
        sim.run_steps(CHUNK_SIZE, progress_bar=False)
        print 'ran %d steps in %5.3f sec, %d steps yet to run.' % (CHUNK_SIZE, time.time() - t2, yet_to_run)


if __name__ == '__main__':
    try:
        rospy.init_node(NODE_NAME, anonymous=False)
        run_MSO_model()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)
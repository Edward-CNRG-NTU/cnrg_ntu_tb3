#!/usr/bin/env python
import numpy as np
import timeit
import threading
import nengo, nengo_dl
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

DELAY_STEPS = [0, 1, 3, 5, 7, 9, 10]
DELAY_STEPS_R = list(reversed(DELAY_STEPS))
N_DELAY_VAL = len(DELAY_STEPS)
MAXPOOLING = 100


synapse1 = 0
synapse2 = 0
synapse3 = 0

radius_1 = 1
radius_2 = 2


def build_nengo_model():
    lifrate_model = nengo.LIFRate(tau_rc=0.002, tau_ref=0.0002)
    max_r = nengo.dists.Uniform(1000, 2000)

    def MSO_ensemble_array(dim, radius, label='ens_arr'):
        return nengo.networks.EnsembleArray(n_neurons=32, n_ensembles=dim, ens_dimensions=1, label=label,
                                            radius=radius, neuron_type=lifrate_model, max_rates=max_r, seed=None)

    with nengo.Network(label="MSO_Jeffress_Model_Multi_Channel") as model:
        input_node_L = nengo.Node([0]*N_SUBCHANNELS, label='input_node_L')
        input_node_R = nengo.Node([0]*N_SUBCHANNELS, label='input_node_R')

        ens_arr_L = MSO_ensemble_array(N_SUBCHANNELS, radius_1, 'ens_arr_L')
        ens_arr_R = MSO_ensemble_array(N_SUBCHANNELS, radius_1, 'ens_arr_R')
        ens_arr_add = MSO_ensemble_array(N_SUBCHANNELS, radius_2, 'ens_arr_add')            

        nengo.Connection(input_node_L,   ens_arr_L.input, synapse=synapse1)
        nengo.Connection(input_node_R,   ens_arr_R.input, synapse=synapse1)
        nengo.Connection(ens_arr_L.output, ens_arr_add.input, synapse=synapse2)
        nengo.Connection(ens_arr_R.output, ens_arr_add.input, synapse=synapse2)

        ens_arr_add_probe = nengo.Probe(ens_arr_add.output, label='ens_arr_add_probe', synapse=synapse3, sample_every=0.01)

        simulator = nengo_dl.Simulator(model, dt=(1. / SAMPLE_RATE), unroll_simulation=64, minibatch_size=N_DELAY_VAL)

    return simulator, input_node_L, input_node_R, ens_arr_add_probe   


def run_MSO_model():
    dl_L = DelayLine((N_SUBCHANNELS,), SAMPLE_RATE, initial_value=0.05, max_delay=MAX_DELAY)
    dl_R = DelayLine((N_SUBCHANNELS,), SAMPLE_RATE, initial_value=0.05, max_delay=MAX_DELAY)
    in_L_data = np.zeros((N_DELAY_VAL, CHUNK_SIZE, N_SUBCHANNELS))
    in_R_data = np.zeros((N_DELAY_VAL, CHUNK_SIZE, N_SUBCHANNELS))

    event = threading.Event()

    def ani_cb(data):        
        if data.shape[1] != CHUNK_SIZE or data.shape[2] != N_SUBCHANNELS or data.sample_rate != SAMPLE_RATE:            
            rospy.logwarn('NOT IMPLEMENT YET: dynamic SRC_CHUNK_SIZE, N_SUBCHANNELS and SAMPLE_RATE not supported!')
            return
        
        ani_data = np.array(data.data).reshape(data.shape)

        dl_L.update(ani_data[0], timecode=data.timecode)
        dl_R.update(ani_data[1])
        event.set()

    sim, in_L, in_R, out_probe = build_nengo_model()
    
    rospy.Subscriber(SUB_TOPIC_NAME, AuditoryNerveImageMultiDim, ani_cb)
    rospy.loginfo('"%s" starts subscribing to "%s".' % (NODE_NAME, SUB_TOPIC_NAME))

    while not rospy.is_shutdown() and event.wait(1.0):

        yet_to_run = dl_R.n_steps - sim.n_steps - CHUNK_SIZE

        if yet_to_run == 0:
            event.clear()
        elif yet_to_run > MAX_STEPS:
            rospy.logwarn('delay too much!')
            break
        elif yet_to_run < 0:
            rospy.logwarn('skip')
            event.clear()
            continue
        # elif yet_to_run < 0:
        #     print 'yet_to_run < 0?'
        #     break

        t2 = timeit.default_timer()
        in_L_data[:] = dl_L.batch_view_chunk(sim.n_steps, CHUNK_SIZE, delay_steps=DELAY_STEPS)
        in_R_data[:] = dl_R.batch_view_chunk(sim.n_steps, CHUNK_SIZE, delay_steps=DELAY_STEPS_R)        
        if (in_L_data is not None) and (in_R_data is not None):
            print 'batch_view_chunk: ', timeit.default_timer() - t2
            sim.run_steps(CHUNK_SIZE, progress_bar=False, input_feeds={in_L: in_L_data, in_R: in_R_data})
            print sim.model.params[out_probe][-1].shape
            del sim.model.params[out_probe][-1]
            print 'ran %d steps in %5.3f sec, %d steps yet to run.' % (CHUNK_SIZE, timeit.default_timer() - t2, yet_to_run)


if __name__ == '__main__':
    try:
        rospy.init_node(NODE_NAME, anonymous=False)
        run_MSO_model()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)
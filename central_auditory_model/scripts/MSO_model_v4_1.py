#!/usr/bin/env python
import numpy as np
import timeit
import threading
import nengo, nengo_dl
from delay_line import DelayLine

import rospy
from std_msgs.msg import String
from std_msgs.msg import Header
# from binaural_microphone.msg import BinauralAudio
from ipem_module.msg import AuditoryNerveImage


NODE_NAME = 'nengo_mso_model'
PUB_TOPIC_NAME = '/central_auditory_model/mso_stream'
SUB_TOPIC_NAME = '/ipem_module/apm_stream'
SAMPLE_RATE = 11025
CHUNK_SIZE = 1024
SIM_CHUNK_SIZE = CHUNK_SIZE
N_SUBCHANNELS = 40
MAX_DELAY = 10.
MAX_STEPS = int(MAX_DELAY * SAMPLE_RATE)

# DELAY_STEPS = [10, 9, 7, 5, 3, 1, 0]  # [0, 1, 3, 5, 7, 9, 10]
# DELAY_STEPS = [38, 34, 26, 19, 12, 4, 0] 
# DELAY_STEPS = [20, 18, 14, 10, 6, 2, 0]
DELAY_STEPS = [9, 8, 6, 4, 3, 1, 0]
DELAY_STEPS_R = list(reversed(DELAY_STEPS))
N_DELAY_VAL = len(DELAY_STEPS)
MAXPOOLING = 100

SIM_SKIP_FACTOR = 4
SIM_OUTPUT_SIZE = SIM_CHUNK_SIZE / SIM_SKIP_FACTOR

MAXPOOLING_WINDOW = 256
MAXPOOLING_STEP = 256
OUTPUT_RATE = SAMPLE_RATE / MAXPOOLING_STEP


synapse_node_ens = 0
synapse_ens_node = 0
synapse_probe = 0

radius_ens = 2


def maxpooling(a, window=MAXPOOLING_WINDOW, step=MAXPOOLING_STEP, axis=-1):
    # print a.shape, a.strides
    shape = a.shape[:axis] + ((a.shape[axis] - window) // step + 1, window) + a.shape[axis + 1:]
    strides = a.strides[:axis] + (a.strides[axis] * step, a.strides[axis]) + a.strides[axis + 1:]
    # print shape, strides
    return np.lib.stride_tricks.as_strided(a, shape=shape, strides=strides).max(axis=axis + 1)



def build_nengo_model():
    lifrate_model = nengo.LIFRate(tau_rc=0.002, tau_ref=0.0002)
    max_r = nengo.dists.Uniform(1000, 2000)

    with nengo.Network(label="MSO_Jeffress_Model") as model:
        n_neurons = 32

        input_node_L = nengo.Node([0], label='input_node_L')
        input_node_R = nengo.Node([0], label='input_node_R')

        ens_L = nengo.Ensemble(n_neurons, 1, radius=radius_ens, neuron_type=lifrate_model, max_rates=max_r, label='ens_L')
        ens_R = nengo.Ensemble(n_neurons, 1, radius=radius_ens, neuron_type=lifrate_model, max_rates=max_r, label='ens_R')

        output_node = nengo.Node(size_in=1, size_out=1, label='output_node')

        nengo.Connection(input_node_L, ens_L, synapse=synapse_node_ens)
        nengo.Connection(input_node_R, ens_R, synapse=synapse_node_ens)
        nengo.Connection(ens_L, output_node, synapse=synapse_ens_node)
        nengo.Connection(ens_R, output_node, synapse=synapse_ens_node)
        
        output_probe = nengo.Probe(output_node, label='output_node_probe', synapse=synapse_probe)  # , sample_every=0.01

        nengo_dl.configure_settings(session_config={"gpu_options.allow_growth": True})
        simulator = nengo_dl.Simulator(model, dt=(1. / SAMPLE_RATE), unroll_simulation=32, minibatch_size=N_DELAY_VAL * N_SUBCHANNELS)

    return simulator, input_node_L, input_node_R, output_probe   


def run_MSO_model():    
    ani_L = np.zeros([CHUNK_SIZE, N_SUBCHANNELS])
    ani_R = np.zeros([CHUNK_SIZE, N_SUBCHANNELS])    
    ani_L_1d = ani_L.ravel() # create an 1-D view into ani_L, must use ravel().
    ani_R_1d = ani_R.ravel() # create an 1-D view into ani_L, must use ravel().

    dl_L = DelayLine((N_SUBCHANNELS,), SAMPLE_RATE, initial_value=0.05, max_delay=MAX_DELAY)
    dl_R = DelayLine((N_SUBCHANNELS,), SAMPLE_RATE, initial_value=0.05, max_delay=MAX_DELAY)    

    sim, in_L, in_R, out_probe = build_nengo_model()

    event = threading.Event()

    def ani_cb(data):        
        if data.chunk_size != CHUNK_SIZE or data.n_subchannels != N_SUBCHANNELS or data.sample_rate != SAMPLE_RATE:            
            rospy.logwarn('NOT IMPLEMENT YET: dynamic CHUNK_SIZE, N_SUBCHANNELS and SAMPLE_RATE not supported!')
            return
        try:
            ani_L_1d[:] = data.left_channel
            ani_R_1d[:] = data.right_channel
        except ValueError:
            rospy.logwarn('shape mismatch: %d -> %d %d' % (len(data.left_channel), data.chunk_size, data.n_subchannels))
            return
        else:
            dl_L.update(ani_L, timecode=data.timecode)
            dl_R.update(ani_R)
            event.set()

    mso_pub = rospy.Publisher(PUB_TOPIC_NAME, AuditoryNerveImage, queue_size=1)
    
    rospy.Subscriber(SUB_TOPIC_NAME, AuditoryNerveImage, ani_cb)

    rospy.loginfo('"%s" starts subscribing to "%s".' % (NODE_NAME, SUB_TOPIC_NAME))

    while not rospy.is_shutdown() and event.wait(1.):
        yet_to_run = dl_R.n_steps - sim.n_steps * SIM_SKIP_FACTOR - SIM_CHUNK_SIZE

        if yet_to_run == 0:
            event.clear()
        elif yet_to_run > MAX_STEPS:
            rospy.logwarn('delay too much!')
            break
        elif yet_to_run < 0:
            rospy.logwarn('skip')
            event.clear()
            continue

        t2 = timeit.default_timer()

        try:
            view_start = sim.n_steps * SIM_SKIP_FACTOR
            timecode = dl_L.get_timecode(view_start)
            assert timecode == dl_L.get_timecode(view_start + SIM_OUTPUT_SIZE - 1), 'Timecode out of sync!'            
            in_L_data = dl_L.batch_view_chunk(view_start, SIM_OUTPUT_SIZE, delay_steps=DELAY_STEPS)
            in_R_data = dl_R.batch_view_chunk(view_start, SIM_OUTPUT_SIZE, delay_steps=DELAY_STEPS_R)
        except ValueError:
            print 'ValueError occur, skipping...'
            continue
        else:
            # print in_L_data.shape
            reformed_L_data = np.swapaxes(in_L_data, 1, 2).reshape((N_DELAY_VAL * N_SUBCHANNELS, SIM_OUTPUT_SIZE, 1))
            reformed_R_data = np.swapaxes(in_R_data, 1, 2).reshape((N_DELAY_VAL * N_SUBCHANNELS, SIM_OUTPUT_SIZE, 1))
            # print reformed_L_data.shape
            sim.run_steps(SIM_OUTPUT_SIZE, progress_bar=False, input_feeds={in_L: reformed_L_data, in_R: reformed_R_data})
            # print sim.model.params[out_probe][-1].shape

            mso_data = sim.model.params[out_probe][-1].reshape((N_DELAY_VAL, N_SUBCHANNELS, SIM_OUTPUT_SIZE))
            mso_data = maxpooling(mso_data, axis=2)
            # print mso_data.shape
            mso_data = np.swapaxes(mso_data, 1, 2)
            # print mso_data.shape

            mso_msg = AuditoryNerveImage(header=Header(
                                            stamp=rospy.Time.now()
                                        ),
                                        timecode=timecode,
                                        sample_rate=OUTPUT_RATE,
                                        chunk_size=SIM_OUTPUT_SIZE,
                                        n_subchannels=N_SUBCHANNELS,
                                        shape=mso_data.shape,
                                        info='(direction, chunk_size, n_subchannels)',
                                        left_channel=mso_data.reshape(-1),
                                        right_channel=[])
            mso_pub.publish(mso_msg)
            rospy.logwarn('[%f] ran %d steps in %5.3f sec, %d steps yet to run.' % (timecode.to_sec(), SIM_OUTPUT_SIZE, timeit.default_timer() - t2, yet_to_run))
        # TODO: handle timeout and not directly exit.


if __name__ == '__main__':
    try:
        rospy.init_node(NODE_NAME, anonymous=True)
        run_MSO_model()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)
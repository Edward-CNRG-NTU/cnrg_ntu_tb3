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

# ROS
NODE_NAME = 'nengo_lso_model'
PUB_TOPIC_NAME = '/central_auditory_model/lso_stream'
SUB_TOPIC_NAME = '/ipem_module/apm_stream'

# SIGNAL
SAMPLE_RATE = 11025
CHUNK_SIZE = 1024
N_SUBCHANNELS = 40

# MODEL
# DECAY_VALUE = [-0.45, -0.4, -0.35, -0.25, -0.1, -0.1, 0.]   # 0.5425
# DECAY_VALUE = [-0.45, -0.4, -0.35, -0.25, -0.15, -0.1, -0.05]  # 0.61
DECAY_VALUE = [-0.45, -0.4, -0.35, -0.25, -0.15, -0.05, -0.00]  # 0.7775
DECAY_VALUE_R = list(reversed(DECAY_VALUE))
N_DECAY_VAL = len(DECAY_VALUE)

# MAXPOOLING
MAXPOOLING_WINDOW = 256
MAXPOOLING_STEP = 256
MAXPOOLING_OVERLAP = MAXPOOLING_WINDOW - MAXPOOLING_STEP


# SIMULATION
MAX_DELAY = 1.
MAX_STEPS = int(MAX_DELAY * SAMPLE_RATE)
SIM_CHUNK_SIZE = CHUNK_SIZE
SIM_OUTPUT_SIZE = SIM_CHUNK_SIZE / MAXPOOLING_STEP
OUTPUT_RATE = SAMPLE_RATE / MAXPOOLING_STEP


synapse_node_ens = 0
synapse_ens_ILD = 0
synapse_ILD_node = 0
synapse_probe = 0

radius_ens = 1.5
radius_ILD = 3.0

seed = 6666


def maxpooling(a, window=MAXPOOLING_WINDOW, step=MAXPOOLING_STEP, axis=-1):
    # print a.shape, a.strides
    shape = a.shape[:axis] + ((a.shape[axis] - window) // step + 1, window) + a.shape[axis + 1:]
    strides = a.strides[:axis] + (a.strides[axis] * step, a.strides[axis]) + a.strides[axis + 1:]
    # print shape, strides
    return np.lib.stride_tricks.as_strided(a, shape=shape, strides=strides).max(axis=axis + 1)


def build_nengo_model():
    lifrate_model = nengo.LIFRate(tau_rc=0.002, tau_ref=0.0002)
    max_r = nengo.dists.Uniform(1000, 2000)

    def decibel_function_factory(level_offset, sign):

        def decibel_function(x):
            return sign * (2 * np.log10(np.max([x, 5e-2])) + level_offset)

        return decibel_function

    def output_function(x):
        if np.abs(x) <= 0.2:
            return 1.0
        else:
            return 0.

    with nengo.Network(label="LSO_Jeffress_Model") as model:
        n_neurons = 64

        input_L = nengo.Node([0], label='input_node_L')
        input_R = nengo.Node([0], label='input_node_R')

        ens_L = nengo.Ensemble(n_neurons, 1, radius=radius_ens, neuron_type=lifrate_model, max_rates=max_r, label='ens_L', seed=seed)
        ens_R = nengo.Ensemble(n_neurons, 1, radius=radius_ens, neuron_type=lifrate_model, max_rates=max_r, label='ens_R', seed=seed)

        ens_arr_ILD = [nengo.Ensemble(n_neurons, 1, radius=radius_ILD, neuron_type=lifrate_model, max_rates=max_r, label='ens_arr_ILD', seed=seed) for i in range(N_DECAY_VAL)]

        output_node = nengo.Node(size_in=N_DECAY_VAL, size_out=N_DECAY_VAL, label='output_node')

        nengo.Connection(input_L, ens_L, synapse=synapse_node_ens)
        nengo.Connection(input_R, ens_R, synapse=synapse_node_ens)

        for i in range(N_DECAY_VAL):
            nengo.Connection(ens_L, ens_arr_ILD[i], synapse=synapse_ens_ILD,
                             function=decibel_function_factory(level_offset=DECAY_VALUE[i], sign=1.))
            nengo.Connection(ens_R, ens_arr_ILD[i], synapse=synapse_ens_ILD,
                             function=decibel_function_factory(level_offset=DECAY_VALUE_R[i], sign=-1.))
            nengo.Connection(ens_arr_ILD[i], output_node[i], synapse=synapse_ILD_node, function=output_function)
        
        output_probe = nengo.Probe(output_node, label='output_probe', synapse=synapse_probe)  # , sample_every=0.01

        nengo_dl.configure_settings(session_config={"gpu_options.allow_growth": True})
        simulator = nengo_dl.Simulator(model, dt=(1. / OUTPUT_RATE), unroll_simulation=SIM_OUTPUT_SIZE, minibatch_size=N_SUBCHANNELS)

    return simulator, input_L, input_R, output_probe   


def run_LSO_model():    
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
            # rospy.logwarn('%f %f' % (ani_L.min(), ani_L.max()))
            event.set()

    lso_pub = rospy.Publisher(PUB_TOPIC_NAME, AuditoryNerveImage, queue_size=1)
    rospy.Subscriber(SUB_TOPIC_NAME, AuditoryNerveImage, ani_cb)

    rospy.loginfo('"%s" starts subscribing to "%s".' % (NODE_NAME, SUB_TOPIC_NAME))

    while not rospy.is_shutdown() and event.wait(1.0):
        yet_to_run = dl_R.n_steps - sim.n_steps * MAXPOOLING_STEP - SIM_CHUNK_SIZE

        if yet_to_run == 0:
            event.clear()
        elif yet_to_run > MAX_STEPS:
            rospy.logwarn('delay too much!')
            break
        elif yet_to_run < 0:
            rospy.logwarn('skipping...')
            event.clear()
            continue

        t2 = timeit.default_timer()

        try:
            view_start = sim.n_steps * MAXPOOLING_STEP - MAXPOOLING_OVERLAP
            view_len = SIM_CHUNK_SIZE + MAXPOOLING_OVERLAP
            timecode = dl_L.get_timecode(view_start + MAXPOOLING_OVERLAP)
            timecode_2 = dl_L.get_timecode(view_start + view_len - 1)
            
            if timecode != timecode_2:
                print 'Timecode out of sync! %f, %f' % (timecode.to_sec(), timecode_2.to_sec())
                for i in range(view_len):
                    print i, dl_L.get_timecode(view_start + i)

            in_L_data = dl_L.batch_view_chunk(view_start, view_len)
            in_R_data = dl_R.batch_view_chunk(view_start, view_len)
        except ValueError:
            print 'ValueError occur, skipping...'
            continue
        else:
            in_L_data = maxpooling(in_L_data, axis=1)
            in_R_data = maxpooling(in_R_data, axis=1)
            # print in_L_data.shape, in_R_data.shape
            reformed_L_data = np.swapaxes(in_L_data, 0, 2)
            reformed_R_data = np.swapaxes(in_R_data, 0, 2)
            # print reformed_L_data.shape, reformed_R_data.shape
            sim.run_steps(SIM_OUTPUT_SIZE, progress_bar=False, input_feeds={in_L: reformed_L_data, in_R: reformed_R_data})

            # rospy.logwarn(sim.model.params[out_probe][-1].shape)
            lso_data = np.swapaxes(sim.model.params[out_probe][-1], 0, 2)
            # rospy.logwarn(lso_data)
            lso_msg = AuditoryNerveImage(header=Header(
                                            stamp=rospy.Time.now()
                                        ),
                                        timecode=timecode,
                                        sample_rate=OUTPUT_RATE,
                                        chunk_size=SIM_OUTPUT_SIZE,
                                        n_subchannels=N_SUBCHANNELS,
                                        shape=lso_data.shape,
                                        info='(direction, chunk_size, n_subchannels)',
                                        left_channel=lso_data.reshape(-1),
                                        right_channel=[])
            lso_pub.publish(lso_msg)
            print '[%f] ran %d steps in %5.3f sec, %d steps yet to run.' % (timecode.to_sec(), SIM_OUTPUT_SIZE, timeit.default_timer() - t2, yet_to_run)


if __name__ == '__main__':
    try:
        rospy.init_node(NODE_NAME, anonymous=True)
        run_LSO_model()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)
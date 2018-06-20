# coding: utf-8
#!/usr/bin/env python
import numpy as np
# import itertools
import matplotlib.pyplot as plt

SUPPORTED_ANGLES = [90, 60, 30, 0, 330, 300, 270]

ear_distance = 0.15
speed_of_sound = 340.
ear_delay = ear_distance / speed_of_sound

print ear_distance, speed_of_sound, ear_delay

exact_delay = ear_delay * np.sin(np.pi * np.array(SUPPORTED_ANGLES) / 180.)
# exact_position = exact_position - np.min(exact_position)

print exact_delay


def get_delay_and_tick(delay_steps, dt):
    delay_steps = np.sort(delay_steps)
    tick = np.arange(delay_steps[0], delay_steps[-1] + 1)
    center = np.mean(tick)
    tick = (tick - center) * dt
    delay = (delay_steps - center) * dt
    return delay, tick


dt_1 = 0.00001
delay_steps_1 = [0, 9, 9+17, 9+17+15, 9+17+15+15, 9+17+15+15+17, 9+17+15+15+17+9]
delay_1, tick_1 = get_delay_and_tick(delay_steps_1, dt_1)

dt_2 = 1. / 11025.
# delay_steps_2 = [10, 9, 7, 5, 3, 1, 0]
delay_steps_2 = [9, 8, 6, 4, 3, 1, 0]
delay_2, tick_2 = get_delay_and_tick(delay_steps_2, dt_2)

dt_3 = 1. / 11025.
delay_steps_3 = [7, 6, 5, 4, 2, 1, 0]
delay_3, tick_3 = get_delay_and_tick(delay_steps_3, dt_3)


ax = plt.gca()
ax.set_aspect('equal')

c = plt.Circle((0., 0.), radius=ear_delay, fill=False, color='r', alpha=1.0)
ax.add_artist(c)

plt.vlines(exact_delay, -1. * 0.0005 * np.ones_like(exact_delay), 1. * 0.0005 * np.ones_like(exact_delay), color='r', alpha=0.3, linestyles=':')


def draw_delay_both_way(y_partition, delay, tick, color):
    plt.scatter(delay, y_partition * 0.0005 * np.ones_like(delay), color=color)
    plt.scatter(tick, y_partition * 0.0005 * np.ones_like(tick), color=color, marker='|', s=200, alpha=0.3)
    delay *= -1
    tick *= -1
    y_partition -= 0.1
    plt.scatter(delay, y_partition * 0.0005 * np.ones_like(delay), color=color)
    plt.scatter(tick, y_partition * 0.0005 * np.ones_like(tick), color=color, marker='|', s=200, alpha=0.3)
    

draw_delay_both_way(0.2, delay_1, tick_1, 'g')

draw_delay_both_way(-0.1, delay_2, tick_2, 'b')

draw_delay_both_way(-0.4, delay_3, tick_3, 'c')


plt.ylim((-0.0005, 0.0005))
plt.xlim((-0.0005, 0.0005))

plt.tight_layout()
plt.show()
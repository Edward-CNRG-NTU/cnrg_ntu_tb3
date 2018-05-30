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


dt_1 = 0.00001
delay_steps_1 = list(reversed([0, 9, 9+17, 9+17+15, 9+17+15+15, 9+17+15+15+17, 9+17+15+15+17+9]))
delay_1 = np.array(delay_steps_1)
tick_1 = (np.arange(delay_1.min(), delay_1.max() + 1) - np.mean(delay_1)) * dt_1
delay_1 = (delay_1 - np.mean(delay_1)) * dt_1

print dt_1, delay_1


dt_2 = 1. / 11025.
delay_steps_2 = [9, 8, 6, 4, 3, 1, 0]
# delay_steps_2 = [10, 9, 7, 5, 3, 1, 0]

delay_2 = np.array(delay_steps_2)
tick_2 = (np.arange(delay_2.min(), delay_2.max() + 1) - np.mean(delay_2)) * dt_2
delay_2 = (delay_2 - np.mean(delay_2)) * dt_2


print dt_2, delay_2

ax = plt.gca()
ax.set_aspect('equal')

c = plt.Circle((0., 0.), radius=ear_delay, fill=False, color='r', alpha=1.0)
ax.add_artist(c)

plt.vlines(exact_delay, -1. * 0.0005 * np.ones_like(exact_delay), 1. * 0.0005 * np.ones_like(exact_delay), color='r', alpha=0.3, linestyles=':')

plt.scatter(delay_1, 0.1 * 0.0005 * np.ones_like(delay_1), color='g')
plt.scatter(tick_1, 0.1 * 0.0005 * np.ones_like(tick_1), color='g', marker='|', s=200, alpha=0.3)

plt.scatter(delay_2, -0.1 * 0.0005 * np.ones_like(delay_2), color='b')
plt.scatter(tick_2, -0.1 * 0.0005 * np.ones_like(tick_2), color='b', marker='|', s=200, alpha=0.3)

plt.ylim((-0.0005, 0.0005))
plt.xlim((-0.0005, 0.0005))

plt.tight_layout()
plt.show()
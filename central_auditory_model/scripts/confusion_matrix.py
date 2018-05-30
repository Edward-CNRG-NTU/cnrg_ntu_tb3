# coding: utf-8
#!/usr/bin/env python
import numpy as np
import itertools
import matplotlib.pyplot as plt

SUPPORTED_ANGLES = [90, 60, 30, 0, 330, 300, 270]

def draw_confusion_matrix(confusion_matrix, classes=SUPPORTED_ANGLES, show=True):
    confusion_matrix = np.asarray(confusion_matrix)

    plt.imshow(confusion_matrix, interpolation='nearest', cmap=plt.cm.Blues)
    # plt.scatter(t*np.ones(7),angle_array,out[:,t]+1,color='r')

    thresh = confusion_matrix.max() / 2.
    for i, j in itertools.product(range(confusion_matrix.shape[0]), range(confusion_matrix.shape[1])):
        plt.text(j, i, '%.2f' % confusion_matrix[i, j],
                horizontalalignment="center",
                color='white' if confusion_matrix[i, j] > thresh else 'black')

    plt.title('angle confusion matrix')
    plt.colorbar()
    tick_marks = np.arange(len(classes))
    plt.xticks(tick_marks, classes, rotation=45)
    plt.yticks(tick_marks, classes)

    plt.xlabel('(Simulated) Source Angle (degree)')
    plt.ylabel('Estimated Angle (degree)')
    plt.tight_layout()
    if show:
        plt.show()


if __name__ == '__main__':
    confusion_matrix = [[0.845, 0.155, 0.,    0.,    0.,    0.,    0.   ],
                        [0., 1., 0., 0., 0., 0., 0.],
                        [0., 0., 1., 0., 0., 0., 0.],
                        [0., 0., 0., 1., 0., 0., 0.],
                        [0., 0., 0., 0., 1., 0., 0.],
                        [0.,   0.,   0.,   0.,   0.,   0.92, 0.08],
                        [0., 0., 0., 0., 0., 0., 1.]
    ]
    draw_confusion_matrix(confusion_matrix)
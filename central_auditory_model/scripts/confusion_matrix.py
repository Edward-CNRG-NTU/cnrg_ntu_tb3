# coding: utf-8
#!/usr/bin/env python
import numpy as np
import itertools
import matplotlib.pyplot as plt

SUPPORTED_ANGLES = [90, 60, 30, 0, 330, 300, 270]

def draw_confusion_matrix(confusion_matrix, classes=SUPPORTED_ANGLES, show=True):
    confusion_matrix = np.asarray(confusion_matrix)

    confusion_matrix = confusion_matrix.T[::-1, :]

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
    plt.yticks(tick_marks, reversed(classes))

    plt.xlabel('(Simulated) Source Angle (degree)')
    plt.ylabel('Estimated Angle (degree)')
    plt.tight_layout()
    if show:
        plt.show()


if __name__ == '__main__':
    confusion_matrix = [[0.465,  0.0425, 0.015,  0.365,  0.,     0.,     0.1125],
                        [0.305,  0.215,  0.0325, 0.29,   0.,     0.,     0.1575],
                        [0.1325, 0.0125, 0.3325, 0.3125, 0.005,  0.0025, 0.2025],
                        [0.1275, 0.015,  0.0025, 0.69,   0.,     0.,     0.165 ],
                        [0.065,  0.0025, 0.0025, 0.4575, 0.365,  0.,     0.1075],
                        [0.0675, 0.01,   0.0075, 0.3925, 0.035,  0.2175, 0.27  ],
                        [0.1025, 0.015,  0.,     0.31,   0.0175, 0.015,  0.54  ]
    ]
    draw_confusion_matrix(confusion_matrix)

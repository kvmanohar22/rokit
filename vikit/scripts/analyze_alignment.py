#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rc

rc('font', **{'family':'serif','serif':['Cardo']})
rc('text', usetex=True)

def add_subplot(ax1, ax2, ax3, data, label):
    ax1.plot(data[:, 1], data[:, 2], label=label)
    ax1.set_ylabel("error in x [m]")
    ax1.grid(linestyle='dotted')
    ax1.legend()

    ax2.plot(data[:, 1], data[:, 3], label=label)
    ax2.set_ylabel("error in y [m]")
    ax2.grid(linestyle='dotted')
    ax2.legend()

    ax3.plot(data[:, 1], data[:, 4], label=label)
    ax3.set_ylabel("error in z [m]")
    ax3.grid(linestyle='dotted')
    ax3.legend()

def single_multi_scales():
    d1 = np.genfromtxt("../data/alignment.csv", delimiter=',')
    d2 = np.genfromtxt("../data/alignment_multi_scale.csv", delimiter=',')
    add_subplot(ax1, ax2, ax3, d1, "single scale")
    add_subplot(ax1, ax2, ax3, d2, "multi  scale")

def multi_scale_comparisons():
    d1 = np.genfromtxt("../data/alignment_pyramidal_30_pyr4.csv", delimiter=',')
    d2 = np.genfromtxt("../data/alignment_pyramidal_30_pyr6.csv", delimiter=',')
    add_subplot(ax1, ax2, ax3, d1, "pyramid size = 4")
    add_subplot(ax1, ax2, ax3, d2, "pyramid size = 6")

fig = plt.figure()
gs = fig.add_gridspec(3, 1, hspace=0)
(ax1, ax2, ax3) = gs.subplots(sharex='col')


multi_scale_comparisons()
plt.xlabel('current image index')
plt.show()


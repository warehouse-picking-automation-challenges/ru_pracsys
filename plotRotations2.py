# coding: utf-8
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def plot3d(c1,c2,c3):
    fig = plt.figure(figsize=(10,10))
    ax = fig.add_subplot(111,projection='3d')
    p3d = ax.scatter(c1,c2,c3,s=5)
    plt.show()

execfile('/home/alberto/RUTGERS/apc_hg/object_models/out2.txt')

plot3d(Template[:,0],Template[:,1],Template[:,2])

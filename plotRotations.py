# coding: utf-8
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def plot3d(c1,c2,c3):
    fig = plt.figure(figsize=(10,10))
    ax = fig.add_subplot(111,projection='3d')
    p3d = ax.scatter(c1,c2,c3,s=20)
    plt.show()

execfile('/home/alberto/RUTGERS/apc_hg/object_models/out.txt')

Temp = np.reshape(Template,(Template.shape[0],3,3))
    
vec = np.array([1,0,0])
pts = np.ndarray((Temp.shape[0],3))
for i in range(0,Temp.shape[0]):
    pts[i] = np.dot(Temp[i],vec)
    
plot3d(pts[:,0],pts[:,1],pts[:,2])

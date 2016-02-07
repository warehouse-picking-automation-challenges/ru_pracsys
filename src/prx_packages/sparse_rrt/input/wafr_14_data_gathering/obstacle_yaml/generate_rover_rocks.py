import sys
import random

j = open("rover_obstacles.yaml",'w');
j.write("building1:\n");
j.write("  type: obstacle\n");
j.write("  geometries:\n");

for x in xrange(0,10000):
    x_val = random.uniform(-200,200);
    y_val = random.uniform(-200,200);
    j.write("    -\n");
    j.write("      name: rock"+str(x)+"\n");
    j.write("      collision_geometry:\n");
    j.write("        type: box\n");
    j.write("        dims: [.2,.2,.2]\n");
    j.write("        material: black\n");
    j.write("      config:\n");
    j.write("        position: ["+str(x_val)+","+str(y_val)+",0]\n");
    j.write("        orientation: [0,0,0,1]\n");
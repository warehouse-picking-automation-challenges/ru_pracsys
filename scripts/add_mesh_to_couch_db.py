import os
import sys

object_id_file = "/home/colinmatthew/RUTGERS/apc_main/scripts/object_id.txt"
object_list_file = "/home/colinmatthew/RUTGERS/apc_main/scripts/object_list.txt"
mesh_status_file = "/home/colinmatthew/RUTGERS/apc_main/scripts/mesh_add_status.txt"

objectlist = open(object_list_file,"r")
lines = objectlist.readlines()

for line in lines:
	line = line.strip()
	print os.system("cd $PRACSYS_PATH/../object_models; rosrun object_recognition_core object_add.py -n \""+line+"\" -d \""+line+"\" --commit >> "+object_id_file)
	print line

objectid = open(object_id_file,"r")
ids = objectid.readlines()

for i,line in enumerate(ids):
	id_ = line.split()[-1]
	id_ = id_.strip()
	object_name = lines[i]
	object_name =  object_name.strip()
	print object_name, id_
	os.system("cd $PRACSYS_PATH/../object_models; rosrun object_recognition_core mesh_add.py " + id_ + " " + object_name + ".obj --commit >> " + mesh_status_file)
	
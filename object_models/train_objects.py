import couchdb
import os

couch = couchdb.Server('http://localhost:5984')
db = couch['object_recognition']

# for id in db:
#     doc = db[id]
#     if 'Type' in doc:
#         if doc['Type'] == 'Object':
#         	cmd_delete_db = "rosrun object_recognition_core object_delete.py %s --commit" % (doc.id)


# Loop through objects in object_models directory
# and for each .obj file, insert an object
for file in os.listdir("."):
	if file.endswith(".obj"):

		obj_name = file[0:-4]
		cmd_str = "rosrun object_recognition_core object_add.py -n %s -d %s --commit" % (obj_name, obj_name) 
		os.system(cmd_str)

		# Now loop through db objects to find corresponding object_id
		for id in db:
		    doc = db[id]
		    if 'Type' in doc:
		        if doc['Type'] == 'Object':
		            if doc['object_name'] == obj_name:
		            	obj_id = doc.id
		            	obj_name_ext = obj_name + ".obj"
		            	cmd_str_2 = "rosrun object_recognition_core mesh_add.py %s %s --commit" % (doc.id, obj_name_ext)
		            	os.system(cmd_str_2)

		            	# Write individual detection config files for each object
		            	# in src/linemod/conf/ -- format:
		            	# detection.object_name.ros.ork
		            	# file_to_open = "detection_template.txt"
		            	# file_to_write = "../src/linemod/conf/detection.%s.ros.ork" % (obj_name)
		            	# cmd_cp = "cp %s %s" % (file_to_open, file_to_write)
		            	# os.system(cmd_cp)
		            	# with open(file_to_write, "a+") as cmd_open_file:
		            	# 	cmd_open_file.seek(0)
			            # 	for line in cmd_open_file:
			            # 		if "insert_object_id_here" in line:
			            # 			print line
			            # 			cmd_open_file.write(line.replace("insert_object_id_here", obj_id))
			            # 			print line
		            	# # os.write(cmd_open_file, write_string)
		            	# cmd_open_file.close()

		            	# Write individual training config files for each object
		            	# in src/linemod/conf/ -- format:
		            	# training.object_name.ork
		       #      	file_to_open = "training_template.txt"
		       #      	file_to_write = "../src/linemod/conf/training.%s.ork" % (obj_name)
		       #      	cmd_cp = "cp %s %s" % (file_to_open, file_to_write)
		       #      	os.system(cmd_cp)
		       #      	with open(file_to_write, "a+") as cmd_open_file:
		       #      		cmd_open_file.seek(0)
			      #       	for line in cmd_open_file:
			      #       		if "insert_object_id_here" in line:
									# cmd_open_file.write(line.replace("insert_object_id_here", obj_id))
		       #      	# os.write(cmd_open_file, write_string)
		       #      	cmd_open_file.close()

cmd_train_obj = "rosrun object_recognition_core training -c ../src/linemod/conf/training.ros.ork"
os.system(cmd_train_obj)



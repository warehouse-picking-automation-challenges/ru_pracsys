from cv2 import cv
import numpy as np
import yaml
import os, sys
import re
import pprint

############################################################################################################
# Header Functions
##################

# Custom Definition For yaml Package Working With opencv-matrix Type
def opencv_matrix(loader, node):
    mapping = loader.construct_mapping(node, deep=True)
    rows = mapping["rows"]
    cols = mapping["cols"]
    dt = mapping["dt"]
    mat = np.array(mapping["data"])
    mat.resize(mapping["rows"], mapping["cols"])
    return CVMat(rows, cols, dt, mat)
yaml.add_constructor(u"tag:yaml.org,2002:opencv-matrix", opencv_matrix)

# def opencv_matrix_representer(dumper, dat)

class CVMat(yaml.YAMLObject):
	yaml_tag = u'tag:yaml.org,2002:opencv-matrix'
	def __init__(self, rows, cols, dt, data):
		self.name = self.__class__.__name__
		self.rows = rows
		self.cols = cols
		self.dt = dt
		self.data = data

	def __repr__(self):
		return "rows=%r, cols=%r, dt=%r, data=%r" % (self.rows, self.cols, self.dt, self.data.tolist())

# Class Def For Working Easily With Pose Objects
class Pose(yaml.YAMLObject):
	def __init__(self):
		self.object_translation_wrt_camera = np.array(object_translation_wrt_camera)
		self.object_rotation_wrt_camera = np.array(object_rotation_wrt_camera)
		
		self.object_translation_wrt_base = np.array(object_translation_wrt_base)
		self.object_rotation_wrt_base = np.array(object_rotation_wrt_base)
		
		self.camera_translation_wrt_base = np.array(camera_translation_wrt_base)
		self.camera_rotation_wrt_base = np.array(camera_rotation_wrt_base)

		self.template_id = template_id
		print "LOADED POSE OBJECT"

	def __repr__(self):
		print "USING OUR REP"
		return """object_translation_wrt_camera=%r, object_rotation_wrt_camera=%r,
			object_translation_wrt_base=%r, object_rotation_wrt_base=%r,
			camera_translation_wrt_base=%r, camera_rotation_wrt_base=%r,
			template_id=%r""" % (
			self.object_translation_wrt_camera, self.object_rotation_wrt_camera,
			self.object_translation_wrt_base, self.object_rotation_wrt_base,
			self.camera_translation_wrt_base, self.camera_rotation_wrt_base,
			self.template_id)


# Inclusive Range Function Def
irange = lambda start, end: range(start, end+1)

############################################################################################################
# Main
######

if __name__ == "__main__":

	# Validate Proper Command Line Usage
	if len(sys.argv) != 2:
		print "ERROR: Command Line args missing! Program requires one argument of the form [run_x] where x indicates which directory to copy ground truth from"
		raise SystemExit
	elif sys.argv[1][:4] != "run_":
		print "ERROR: Program requires one argument of the form [run_x] where x indicates which directory to copy ground truth from"
		print sys.argv[1][:4]
		raise SystemExit

	data_home = "/home/colinmatthew/RUTGERS/apc_videos/data_gathered/"
	curr_dir = sys.argv[1]

	# Obtain Iterating Variables 
	## objects = {object names}, dirs = {folders}
	objects = []
	files = os.listdir("%s" % (data_home+curr_dir))
	apcname_format = re.compile(r"._.")
	for i in range(0,len(files)):
	    candidate = re.split("-",files[i])[0]
	    if candidate not in objects and re.search(apcname_format, candidate) and candidate != "work_order.json":
	        objects.append(candidate)

	dirs = []
	# data_home = re.split("run_",os.getcwd())[0]
	# curr_dir = re.split("/",os.getcwd())[-1]

	x = int(re.split("_",curr_dir)[-1])

	for i in irange(x,x+2):
		dirs.append("run_"+str(i))

	# Main Loop To Do The Copying
	for i, directory in enumerate(dirs):
		
		print "\n\n Directory: %s \n" % directory
		
		for j, objname in enumerate(objects):

			gtdir = data_home + dirs[0] + "/"
			workingdir = data_home + directory + "/"

			gtpattern = re.compile(r"%s.*-1-0.yml$" % objname)
			gtfilename = [k for k in os.listdir("%s" % gtdir) if re.search(gtpattern, k)][0]
			gtfilepath = gtdir + gtfilename

			destpattern = re.compile(r"%s.*.yml$" % objname)
			destfilenames = [k for k in os.listdir("%s" % workingdir) if re.search(destpattern, k)]

			with open(gtfilepath, 'r') as f:
			    pose_yaml = f.read()
			pose_yaml = pose_yaml.replace("%YAML:1.0", "!!python/object:__main__.Pose")
			gt_pose = yaml.load("%s" % pose_yaml)

			for m, destfilename in enumerate(destfilenames):
				destfilepath = workingdir + destfilename
				with open(destfilepath, 'r') as f:
				    destpose_yaml = f.read()
				destpose_yaml = destpose_yaml.replace("%YAML:1.0", "!!python/object:__main__.Pose")
				destpose = yaml.load("%s" % destpose_yaml)

				print "Copying %s Pose into %s" % (gtfilename, destfilename)

				# Paste Obj wrt Base
				destpose.object_rotation_wrt_base = gt_pose.object_rotation_wrt_base
				destpose.object_translation_wrt_base = gt_pose.object_translation_wrt_base
				
				# Calculate Obj wrt Camera
				translation = np.array(destpose.object_translation_wrt_base) - np.array(destpose.camera_translation_wrt_base)
				np_object_rotation_wrt_base = np.array([gt_pose.object_rotation_wrt_base.data[0:3],
					gt_pose.object_rotation_wrt_base.data[3:6],
					gt_pose.object_rotation_wrt_base.data[6:9]])
				np_camera_rotation_wrt_base = np.array([gt_pose.camera_rotation_wrt_base.data[0:3],
					gt_pose.camera_rotation_wrt_base.data[3:6],
					gt_pose.camera_rotation_wrt_base.data[6:9]])
				destpose.object_translation_wrt_camera = np.dot(translation, np_camera_rotation_wrt_base).tolist()
				destpose.object_rotation_wrt_camera.data = np.dot(np_camera_rotation_wrt_base.transpose(), np_object_rotation_wrt_base).reshape((1,9))[0].tolist()
				
				with open(destfilepath, 'w+') as fout:
					yaml_out = yaml.dump(destpose)
					fout.write(yaml_out.replace("!!python/object:__main__.Pose", "%YAML:1.0"))

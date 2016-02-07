import json
import copy
import sys
import os
import re
from pprint import pprint

if len(sys.argv) != 2:
	print "ERROR: Command Line args missing! {clutter, position}"
	raise SystemExit
elif sys.argv[1] not in {"clutter", "position"}:
	print "ERROR: Command Line args missing! {clutter, position}"
	raise SystemExit

with open('/home/pracsys/RUTGERS/apc_hg2/apc_data_gathering.json') as data_file:
	data = json.load(data_file)
data_file.close()

# Move collected data to new folder and check count of items
items = []
home = "/home/pracsys/RUTGERS/apc_hg2/object_models/data_gathered"
files = [f for f in os.listdir('%s' % home) if os.path.isfile(home+"/"+f)]
dirs = [d for d in os.listdir('%s' % home) if not os.path.isfile(home+"/"+d)]

for i in range(0,12):
	items.append(data['work_order'][i]['item'])

failed = False
for i,item in enumerate(items):
	r = re.compile(r'%s.+png$' % item)
	matches = [f for f in files if r.match(f)]
	print i, item, '\t', len(matches)
	if len(matches) != 36:
		print "ERROR: Did not collect 36 png files for %s. Instead only have %i." % (item, len(matches))
		failed = True

if failed:
	raise SystemExit

# Check for erroneous input in command line args
if ((len(dirs)+1) % 3 == 0) & (sys.argv[1] != "position"):
	warn = raw_input("Number of folders in data_gathered suggests changing the position. Are you sure you want to change clutter [y/n]?")
	if warn.lower() != 'y':
		raise SystemExit

if ((len(dirs)+1) % 3 != 0) & (sys.argv[1] != "clutter"):
	warn = raw_input("Number of folders in data_gathered suggests changing the clutter. Are you sure you want to change position [y/n]?")
	if warn.lower() != 'y':
		raise SystemExit


os.system("mkdir %s" % home+"/"+"run_"+str((len(dirs)+1)))
os.system("cp %s/* %s" % (home, home+"/"+"run_"+str((len(dirs)+1))))
os.system("rm %s" % home+"/*")


bin_contents = {}

# Changing clutter routine
if sys.argv[1] == "clutter":

	bin_contents['bin_A'] = copy.deepcopy(data['bin_contents']['bin_C'])
	bin_contents['bin_A'].remove(data['work_order'][2]['item'])
	bin_contents['bin_A'].append(data['work_order'][0]['item'])

	bin_contents['bin_B'] = copy.deepcopy(data['bin_contents']['bin_A'])
	bin_contents['bin_B'].remove(data['work_order'][0]['item'])
	bin_contents['bin_B'].append(data['work_order'][1]['item'])

	bin_contents['bin_C'] = copy.deepcopy(data['bin_contents']['bin_B'])
	bin_contents['bin_C'].remove(data['work_order'][1]['item'])
	bin_contents['bin_C'].append(data['work_order'][2]['item'])

	bin_contents['bin_D'] = copy.deepcopy(data['bin_contents']['bin_F'])
	bin_contents['bin_D'].remove(data['work_order'][5]['item'])
	bin_contents['bin_D'].append(data['work_order'][3]['item'])

	bin_contents['bin_E'] = copy.deepcopy(data['bin_contents']['bin_D'])
	bin_contents['bin_E'].remove(data['work_order'][3]['item'])
	bin_contents['bin_E'].append(data['work_order'][4]['item'])

	bin_contents['bin_F'] = copy.deepcopy(data['bin_contents']['bin_E'])
	bin_contents['bin_F'].remove(data['work_order'][4]['item'])
	bin_contents['bin_F'].append(data['work_order'][5]['item'])

	bin_contents['bin_G'] = copy.deepcopy(data['bin_contents']['bin_I'])
	bin_contents['bin_G'].remove(data['work_order'][8]['item'])
	bin_contents['bin_G'].append(data['work_order'][6]['item'])

	bin_contents['bin_H'] = copy.deepcopy(data['bin_contents']['bin_G'])
	bin_contents['bin_H'].remove(data['work_order'][6]['item'])
	bin_contents['bin_H'].append(data['work_order'][7]['item'])

	bin_contents['bin_I'] = copy.deepcopy(data['bin_contents']['bin_H'])
	bin_contents['bin_I'].remove(data['work_order'][7]['item'])
	bin_contents['bin_I'].append(data['work_order'][8]['item'])

	bin_contents['bin_J'] = copy.deepcopy(data['bin_contents']['bin_L'])
	bin_contents['bin_J'].remove(data['work_order'][11]['item'])
	bin_contents['bin_J'].append(data['work_order'][9]['item'])

	bin_contents['bin_K'] = copy.deepcopy(data['bin_contents']['bin_J'])
	bin_contents['bin_K'].remove(data['work_order'][9]['item'])
	bin_contents['bin_K'].append(data['work_order'][10]['item'])

	bin_contents['bin_L'] = copy.deepcopy(data['bin_contents']['bin_K'])
	bin_contents['bin_L'].remove(data['work_order'][10]['item'])
	bin_contents['bin_L'].append(data['work_order'][11]['item'])

	work_order = copy.deepcopy(data['work_order'])

# Changing position routine
elif sys.argv[1] == "position":
	work_order = copy.deepcopy(data['work_order'])

	bin_contents['bin_A'] = copy.deepcopy(data['bin_contents']['bin_A'])
	bin_contents['bin_A'].remove(data['work_order'][0]['item'])
	bin_contents['bin_A'].append(data['work_order'][11]['item'])
	work_order[0]['item'] = copy.deepcopy(data['work_order'][11]['item'])

	bin_contents['bin_B'] = copy.deepcopy(data['bin_contents']['bin_B'])
	bin_contents['bin_B'].remove(data['work_order'][1]['item'])
	bin_contents['bin_B'].append(data['work_order'][0]['item'])
	work_order[1]['item'] = copy.deepcopy(data['work_order'][0]['item'])

	bin_contents['bin_C'] = copy.deepcopy(data['bin_contents']['bin_C'])
	bin_contents['bin_C'].remove(data['work_order'][2]['item'])
	bin_contents['bin_C'].append(data['work_order'][1]['item'])
	work_order[2]['item'] = copy.deepcopy(data['work_order'][1]['item'])

	bin_contents['bin_D'] = copy.deepcopy(data['bin_contents']['bin_D'])
	bin_contents['bin_D'].remove(data['work_order'][3]['item'])
	bin_contents['bin_D'].append(data['work_order'][2]['item'])
	work_order[3]['item'] = copy.deepcopy(data['work_order'][2]['item'])

	bin_contents['bin_E'] = copy.deepcopy(data['bin_contents']['bin_E'])
	bin_contents['bin_E'].remove(data['work_order'][4]['item'])
	bin_contents['bin_E'].append(data['work_order'][3]['item'])
	work_order[4]['item'] = copy.deepcopy(data['work_order'][3]['item'])

	bin_contents['bin_F'] = copy.deepcopy(data['bin_contents']['bin_F'])
	bin_contents['bin_F'].remove(data['work_order'][5]['item'])
	bin_contents['bin_F'].append(data['work_order'][4]['item'])
	work_order[5]['item'] = copy.deepcopy(data['work_order'][4]['item'])

	bin_contents['bin_G'] = copy.deepcopy(data['bin_contents']['bin_G'])
	bin_contents['bin_G'].remove(data['work_order'][6]['item'])
	bin_contents['bin_G'].append(data['work_order'][5]['item'])
	work_order[6]['item'] = copy.deepcopy(data['work_order'][5]['item'])

	bin_contents['bin_H'] = copy.deepcopy(data['bin_contents']['bin_H'])
	bin_contents['bin_H'].remove(data['work_order'][7]['item'])
	bin_contents['bin_H'].append(data['work_order'][6]['item'])
	work_order[7]['item'] = copy.deepcopy(data['work_order'][6]['item'])

	bin_contents['bin_I'] = copy.deepcopy(data['bin_contents']['bin_I'])
	bin_contents['bin_I'].remove(data['work_order'][8]['item'])
	bin_contents['bin_I'].append(data['work_order'][7]['item'])
	work_order[8]['item'] = copy.deepcopy(data['work_order'][7]['item'])

	bin_contents['bin_J'] = copy.deepcopy(data['bin_contents']['bin_J'])
	bin_contents['bin_J'].remove(data['work_order'][9]['item'])
	bin_contents['bin_J'].append(data['work_order'][8]['item'])
	work_order[9]['item'] = copy.deepcopy(data['work_order'][8]['item'])

	bin_contents['bin_K'] = copy.deepcopy(data['bin_contents']['bin_K'])
	bin_contents['bin_K'].remove(data['work_order'][10]['item'])
	bin_contents['bin_K'].append(data['work_order'][9]['item'])
	work_order[10]['item'] = copy.deepcopy(data['work_order'][9]['item'])

	bin_contents['bin_L'] = copy.deepcopy(data['bin_contents']['bin_L'])
	bin_contents['bin_L'].remove(data['work_order'][11]['item'])
	bin_contents['bin_L'].append(data['work_order'][10]['item'])
	work_order[11]['item'] = copy.deepcopy(data['work_order'][10]['item'])

else:
	print "ERROR: program takes 1 argument. Must be either 'clutter' or 'position'!"
	raise SystemExit

json_data = {'bin_contents' : bin_contents, 'work_order' : work_order}

with open('/home/pracsys/RUTGERS/apc_hg2/apc_data_gathering.json', 'wt') as data_file:
	#print (json.dumps(json_data, sort_keys=True, indent=4))
	json.dump(json_data, data_file, sort_keys=True, indent=4, separators=(',',':'))
	data_file.close()

with open('/home/pracsys/RUTGERS/apc_hg2/object_models/data_gathered/work_order.json', 'wt') as work_order:
	json.dump(json_data, work_order, sort_keys=True, indent=4, separators=(',',':'))
	work_order.close()


wmf = open('wm_obstacles.launch', 'w')
simf = open('sim_obstacles.launch', 'w')
waypointLists = [ 
	[(-2, 2), (-2, -2)], 
	[(0.8, -1.2), (0.8, 0.3)]
	]
index = 1
radius = 0.25
height = 1.7
maxVelocity = 0.75
z = 3.0


simf.write('<launch>')
wmf.write('<launch>')

for waypoints in waypointLists:
	simf.write('\n')
	wmf.write('\n')
	
	simf.write('  <rosparam command="load" ns="simulator/subsystems/waypoints' + str(index) + 
		'" file="$(find prx_input)/templates/controllers/waypoint_controller.yaml" />\n')
	simf.write('  <rosparam command="load" ns="simulator/subsystems/waypoints' + str(index) + 
		'/subsystems/disk' + str(index) + '" file="$(find prx_input)/templates/plants/disk.yaml" />\n')
	simf.write('  <rosparam command="load" ns="simulator/subsystems/waypoints' + str(index) + '">\n')
	simf.write('    waypoints:\n')
	for wp in waypoints:
		simf.write('      -\n')
		simf.write('        state: [' + ','.join(str(i) for i in wp) + ']\n')
	simf.write('  </rosparam>\n')
	simf.write('  <rosparam command="load" ns="simulator/subsystems/waypoints' + str(index) + 
		'/subsystems/disk' + str(index) + '">\n')
	simf.write('    initial_state: [' + ','.join(str(i) for i in waypoints[0]) + ']\n')
	simf.write('    z: ' + str(z) + '\n')
	simf.write('    input_control_space:\n')
	simf.write('      min: [0, -3.14]\n')
	simf.write('      max: [' + str(maxVelocity) + ',  3.14]\n')
	simf.write('    geometries:\n')
	simf.write('      -\n')
	simf.write('        name: body\n')
	simf.write('        collision_geometry:\n')
	simf.write('          type: cylinder\n')
	simf.write('          radius: ' + str(radius) + '\n')
	simf.write('          height: ' + str(height) + '\n')
	simf.write('          material: blue\n')
	simf.write('  </rosparam>\n')
	
	wmf.write('  <rosparam command="load" ns="world_model/simulator/subsystems/disk' + str(index) + 
		'" file="$(find prx_input)/templates/plants/disk.yaml" />\n')
	wmf.write('  <rosparam command="load" ns="world_model/simulator/subsystems/disk' + str(index) + '">\n')
	wmf.write('    initial_state: [' + ','.join(str(i) for i in waypoints[0]) + ']\n')
	wmf.write('    z: ' + str(z) + '\n')
	wmf.write('    input_control_space:\n')
	wmf.write('      min: [0, -3.14]\n')
	wmf.write('      max: [' + str(maxVelocity) + ',  3.14]\n')
	wmf.write('    embeddings:\n')
	wmf.write('      space1:\n')
	wmf.write('        type: obstacle_mapping\n')
	wmf.write('    geometries:\n')
	wmf.write('      -\n')
	wmf.write('        name: body\n')
	wmf.write('        collision_geometry:\n')
	wmf.write('          type: cylinder\n')
	wmf.write('          radius: ' + str(radius) + '\n')
	wmf.write('          height: ' + str(height) + '\n')
	wmf.write('          material: blue\n')
	wmf.write('  </rosparam>\n')
	
	index += 1
	

simf.write('</launch>')
wmf.write('</launch>')

wmf.close()
simf.close()
#Let's figure out some of the global params
NUM_SYSTEMS = 1

#A Mode for what benchmark we are running
MODE = 1 # Mode 0 is for Kiva Shelves, Mode 1 is for tables

#Whether or not to create a planning node
CREATE_PLANNING = true

#Random handling
SPECIFY_RANDOM = true #Whether or not to have a random seed specified in the generated file
RANDOM_RANDOM = true #When we are specifying a seed, this says whether or not we use the value below
RANDOM_SEED = 45096

#Number of collection bins.  This should be between 1 and 4, depending on the problem mode
# This also represents the number of different object destinations
NUM_BINS = 1

#The number of objects we want to generate per shelf/table
NUM_OBJECTS = 5

#What type of motion planner should be instantiated for each arm
PLANNER_TYPE = "rrtc"


#Additional parameters for other uses
TDK_GENERATE_POSES = true
TDK_NUM_POSES = 100
TDK_POSE_BUFFER = 0.05


# = = Some Additional Information here == #

# KIVA SHELF INFORMATION (For input model at Oct 31, 2014)
# This provides the relative offset of the center of each shelf surface (approximately)
# Bin names are tentative

#= - - - - - = - - - - - = - - - - - = - - - - - =
#=  -0.306   =  -0.102   =   0.102   =   0.306   =
#= - - - - - = - - - - - = - - - - - = - - - - - = - - - - - =
#=     A     =     B     =     C     =     D     =   2.215   =
#= - - - - - = - - - - - = - - - - - = - - - - - = - - - - - =
#=     E     =     F     =     G     =     H     =   1.985   =
#= - - - - - = - - - - - = - - - - - = - - - - - = - - - - - =
#=     I     =     J     =     K     =     L     =   1.720   =
#= - - - - - = - - - - - = - - - - - = - - - - - = - - - - - =
#=     M     =     N     =     O     =     P     =   1.490   =
#= - - - - - = - - - - - = - - - - - = - - - - - = - - - - - =
#=     Q     =     R     =     S     =     T     =   1.225   =
#= - - - - - = - - - - - = - - - - - = - - - - - = - - - - - =
#=     U     =     V     =     W     =     X     =   0.880   =
#= - - - - - = - - - - - = - - - - - = - - - - - = - - - - - =
#=     Y     =     Z     =    AA     =    AB     =   0.485   =
#= - - - - - = - - - - - = - - - - - = - - - - - = - - - - - =

#= Note: All shelf centers have a y-offset of: -0.320

#Constants for indexing into the center points of the shelves
COL_1 = -0.306
COL_2 = -0.102
COL_3 = 0.102
COL_4 = 0.306

ROW_1 = 2.215
ROW_2 = 1.985
ROW_3 = 1.720
ROW_4 = 1.490
ROW_5 = 1.225
ROW_6 = 0.880
ROW_7 = 0.485

Y_OFF = -0.320

# = = Methods = = #

# = Generate a bin at the target location = #
def generate_bin( f, x, y, theta, i )
    quat = "[0,0," << (Math::sin(theta/2.0)).to_s << "," << (Math::cos(theta/2.0)).to_s << "]"

    #print "Generating Bin[" << i.to_s << "]: " << x.to_s << " , " << y.to_s << "\n"

    #Put one in the simulator
    f << "  <rosparam ns=\"simulation/simulator/obstacles\">\n"
    f << "    bin" << i.to_s << ":\n"
    f << "      type: obstacle\n"
    f << "      geometries:\n"
    f << "        -\n"
    f << "          name: base\n"
    f << "          collision_geometry:\n"
    f << "            type: box\n"
    f << "            dims: [0.6,1.0,0.6]\n"
    f << "            material: yellow\n"
    f << "          config:\n"
    f << "            position: [" << x.to_s << "," << y.to_s << ",0.3]\n"
    f << "            orientation: " << quat << "\n"
    f << "  </rosparam>\n\n"

    #And one in the planning
    f << "  <rosparam ns=\"planning/world_model/simulator/obstacles\">\n"
    f << "    bin" << i.to_s << ":\n"
    f << "      type: obstacle\n"
    f << "      geometries:\n"
    f << "        -\n"
    f << "          name: base\n"
    f << "          collision_geometry:\n"
    f << "            type: box\n"
    f << "            dims: [0.6,1.0,0.6]\n"
    f << "            material: yellow\n"
    f << "          config:\n"
    f << "            position: [" << x.to_s << "," << y.to_s << ",0.3]\n"
    f << "            orientation: " << quat << "\n"
    f << "  </rosparam>\n\n"

end

# = Generate a baxter at the target location = #
def generate_baxter( f, x, y, theta, i )
    f << "  <!-- Baxter " << i.to_s << "-->\n"

    #SIMULATION
    #Load this dude's consumer controller
    f << "  <rosparam command=\"load\" ns=\"simulation/simulator/subsystems/consumer" << i.to_s << "_left\">\n"
    f << "    template: \"controller\"\n"
    f << "  </rosparam>\n\n"

    f << "  <rosparam command=\"load\" ns=\"simulation/simulator/subsystems/consumer" << i.to_s << "_right\">\n"
    f << "    template: \"controller\"\n"
    f << "  </rosparam>\n\n"

    #Set the plant parameters
    f << "  <rosparam command=\"load\" ns=\"simulation/simulator/subsystems/consumer" << i.to_s << "_left/subsystems/baxter" << i.to_s << "\">\n"
    f << "    template: \"plant\"\n"
    f << "    initial_state: [0.0,-1.34394,-0.42624,2.61800,-2.84100,2.09400,-0.06762,0]\n"
    f << "    hand: left\n"
    f << "    forward_transform_rotation: [" << (Math::cos(-theta)).to_s << "," << (Math::sin(-theta)).to_s << ",0," << (-1.0*Math::sin(-theta)).to_s << "," << (Math::cos(-theta)).to_s << ",0,0,0,1]\n"
    f << "    forward_transform_translation: [" << x.to_s << "," << y.to_s << ",0.785]\n"
    f << "    inverse_transform_rotation: [" << (Math::cos(theta)).to_s << "," << (Math::sin(theta)).to_s << ",0," << (-1.0*Math::sin(theta)).to_s << "," << (Math::cos(theta)).to_s << ",0,0,0,1]\n"
    f << "    inverse_transform_translation: [" << (-1.0*x).to_s << "," << (-1.0*y).to_s << ",-0.785]\n"
    f << "  </rosparam>\n\n"

    f << "  <rosparam command=\"load\" ns=\"simulation/simulator/subsystems/consumer" << i.to_s << "_right/subsystems/baxter" << i.to_s << "\">\n"
    f << "    template: \"plant\"\n"
    f << "    initial_state: [0.0,-1.34394,0.42624,2.61800,2.84100,2.09400,-0.06762,0]\n"
    f << "    hand: right\n"
    f << "    forward_transform_rotation: [" << (Math::cos(-theta)).to_s << "," << (Math::sin(-theta)).to_s << ",0," << (-1.0*Math::sin(-theta)).to_s << "," << (Math::cos(-theta)).to_s << ",0,0,0,1]\n"
    f << "    forward_transform_translation: [" << x.to_s << "," << y.to_s << ",0.785]\n"
    f << "    inverse_transform_rotation: [" << (Math::cos(theta)).to_s << "," << (Math::sin(theta)).to_s << ",0," << (-1.0*Math::sin(theta)).to_s << "," << (Math::cos(theta)).to_s << ",0,0,0,1]\n"
    f << "    inverse_transform_translation: [" << (-1.0*x).to_s << "," << (-1.0*y).to_s << ",-0.785]\n"
    f << "  </rosparam>\n\n"

    #Then, need to place a torso O_o
    f << "  <rosparam ns=\"simulation/simulator/obstacles\">\n"
    generate_torso( f, x, y, theta, i )
    f << "  </rosparam>\n\n"

    #PLANNING
    #Set the plant parameters
    f << "  <rosparam command=\"load\" ns=\"planning/world_model/simulator/subsystems/baxter_left" << i.to_s << "\">\n"
    f << "    template: \"plant\"\n"
    f << "    initial_state: [0.0,-1.34394,-0.42624,2.61800,-2.84100,2.09400,-0.06762,0]\n"
    f << "    hand: left\n"
    f << "    forward_transform_rotation: [" << (Math::cos(-theta)).to_s << "," << (Math::sin(-theta)).to_s << ",0," << (-1.0*Math::sin(-theta)).to_s << "," << (Math::cos(-theta)).to_s << ",0,0,0,1]\n"
    f << "    forward_transform_translation: [" << x.to_s << "," << y.to_s << ",0.785]\n"
    f << "    inverse_transform_rotation: [" << (Math::cos(theta)).to_s << "," << (Math::sin(theta)).to_s << ",0," << (-1.0*Math::sin(theta)).to_s << "," << (Math::cos(theta)).to_s << ",0,0,0,1]\n"
    f << "    inverse_transform_translation: [" << (-1.0*x).to_s << "," << (-1.0*y).to_s << ",-0.785]\n"
    f << "    planning_contexts:\n"
    f << "      space" << i.to_s << ":\n"
    f << "        type: full_mapping\n"
    f << "  </rosparam>\n\n"

    f << "  <rosparam command=\"load\" ns=\"planning/world_model/simulator/subsystems/baxter_right" << i.to_s << "\">\n"
    f << "    template: \"plant\"\n"
    f << "    initial_state: [0.0,-1.34394,0.42624,2.61800,2.84100,2.09400,-0.06762,0]\n"
    f << "    hand: right\n"
    f << "    forward_transform_rotation: [" << (Math::cos(-theta)).to_s << "," << (Math::sin(-theta)).to_s << ",0," << (-1.0*Math::sin(-theta)).to_s << "," << (Math::cos(-theta)).to_s << ",0,0,0,1]\n"
    f << "    forward_transform_translation: [" << x.to_s << "," << y.to_s << ",0.785]\n"
    f << "    inverse_transform_rotation: [" << (Math::cos(theta)).to_s << "," << (Math::sin(theta)).to_s << ",0," << (-1.0*Math::sin(theta)).to_s << "," << (Math::cos(theta)).to_s << ",0,0,0,1]\n"
    f << "    inverse_transform_translation: [" << (-1.0*x).to_s << "," << (-1.0*y).to_s << ",-0.785]\n"
    f << "    planning_contexts:\n"
    f << "      space" << i.to_s << ":\n"
    f << "        type: full_mapping\n"
    f << "  </rosparam>\n\n"

    #Create the motion planner responsible for this arm
    f << "  <rosparam ns=\"planning/task_planner/planners/\" >\n"
    f << "    planner" << i.to_s << ":\n"
    f << "      type: " << PLANNER_TYPE << "\n"
    f << "      heuristic_search:\n"
    f << "        type: prm_astar\n"
    f << "      stretch_factor: 1.5\n"
    f << "      space_name: \"full_space\"\n"
    f << "      visualization_body: \"simulator/baxter_left/end_effector\"\n"
    f << "      visualization_bodies: [\"simulator/baxter_left/end_effector\"]\n"
    f << "      visualize_graph: false\n"
    f << "      visualize_tree: false\n"
    f << "      visualize_solutions: false\n"
    f << "      sparse_delta: 25\n"
    f << "      dense_delta: 0.5\n"
    f << "      delta_prm: false\n"
    f << "      pno_mode: false\n"
    f << "      graph_color: \"green\"\n"
    f << "  </rosparam>\n\n"

    f << "  <rosparam ns=\"planning/task_planner/planners/\" >\n"
    f << "    planner" << i.to_s << ":\n"
    f << "      type: " << PLANNER_TYPE << "\n"
    f << "      heuristic_search:\n"
    f << "        type: prm_astar\n"
    f << "      stretch_factor: 1.5\n"
    f << "      space_name: \"full_space\"\n"
    f << "      visualization_body: \"simulator/baxter_right/end_effector\"\n"
    f << "      visualization_bodies: [\"simulator/baxter_right/end_effector\"]\n"
    f << "      visualize_graph: false\n"
    f << "      visualize_tree: false\n"
    f << "      visualize_solutions: false\n"
    f << "      sparse_delta: 25\n"
    f << "      dense_delta: 0.5\n"
    f << "      delta_prm: false\n"
    f << "      pno_mode: false\n"
    f << "      graph_color: \"green\"\n"
    f << "  </rosparam>\n\n"

    #Then, need to place a torso
    f << "  <rosparam ns=\"planning/world_model/simulator/obstacles\">\n"
    generate_torso( f, x, y, theta, i )
    f << "  </rosparam>\n\n"

end

# = Generate a Kiva shelf at the target location = #
def generate_shelf( f, x, y, i )
    #Put one in the simulator
    f << "  <rosparam ns=\"simulation/simulator/obstacles\">\n"
    f << "    kiva_shelf" << i.to_s << ":\n"
    f << "      type: obstacle\n"
    f << "      geometries:\n"
    f << "        -\n"
    f << "          name: shelf\n"
    f << "          collision_geometry:\n"
    f << "            type: mesh\n"
    f << "            filename: meshes/kiva_pod.stl\n"
    f << "            material: blue\n"
    f << "          visualization_geometry:\n"
    f << "            type: mesh\n"
    f << "            filename: meshes/kiva_pod.osg\n"
    f << "            material: black\n"
    f << "          config:\n"
    f << "            position: [" << x.to_s << "," << y.to_s << ",0.0]\n"
    f << "            orientation: [0.7071067811865476, 0, 0, 0.7071067811865476]\n"
    f << "  </rosparam>\n\n"

    #And one in planning
    f << "  <rosparam ns=\"planning/world_model/simulator/obstacles\">\n"
    f << "    kiva_shelf" << i.to_s << ":\n"
    f << "      type: obstacle\n"
    f << "      geometries:\n"
    f << "        -\n"
    f << "          name: shelf\n"
    f << "          collision_geometry:\n"
    f << "            type: mesh\n"
    f << "            filename: meshes/kiva_pod.stl\n"
    f << "            material: blue\n"
    f << "          visualization_geometry:\n"
    f << "            type: mesh\n"
    f << "            filename: meshes/kiva_pod.osg\n"
    f << "            material: black\n"
    f << "          config:\n"
    f << "            position: [" << x.to_s << "," << y.to_s << ",0.0]\n"
    f << "            orientation: [0.7071067811865476, 0, 0, 0.7071067811865476]\n"
    f << "  </rosparam>\n\n"

end

# = Generate a torso at the target location = #
def generate_torso( f, x, y, theta, i )
    #Simple Torso generation - Begin by figuring out where these thigns are
    quat = "[0,0," << (Math::sin(theta/2.0)).to_s << "," << (Math::cos(theta/2.0)).to_s << "]"
    ped_x = -0.04 * Math::cos(theta)
    ped_y = -0.04 * Math::sin(theta)

    tor_x = -0.065 * Math::cos(theta)
    tor_y = -0.065 * Math::sin(theta)

    hed_x = 0.04 * Math::cos(theta)
    hed_y = 0.04 * Math::sin(theta)

    #Now actually output the things
    f << "    baxter" << i.to_s << "_torso:\n"
    f << "      type: obstacle\n"
    f << "      geometries:\n"
    f << "        -\n"
    f << "          name: pedestal" << i.to_s << "\n"
    f << "          collision_geometry:\n"
    f << "            type: box\n"
    f << "            dims: [0.90, 0.80, 0.05]\n"
    f << "            material: black\n"
    f << "          config:\n"
    f << "            position: [" << (ped_x + x).to_s << ", " << (ped_y + y).to_s << ", 0.31]\n"
    f << "            orientation: " << quat << "\n"
    f << "          visualization_geometry:\n"
    f << "            type: mesh\n"
    f << "            filename: meshes/base/pedestal.osg\n"
    f << "            material: black\n"
    f << "        -\n"
    f << "          name: torso" << i.to_s << "\n"
    f << "          collision_geometry:\n"
    f << "            type: cylinder\n"
    f << "            radius: 0.15\n"
    f << "            height: 1.6\n"
    f << "            material: black\n"
    f << "          config:\n"
    f << "            position: [" << (tor_x + x).to_s << ", " << (tor_y + y).to_s << ", 1.085]\n"
    f << "            orientation: " << quat << "\n"
    f << "          visualization_geometry:\n"
    f << "            type: mesh\n"
    f << "            filename: meshes/torso/base.osg\n"
    f << "            material: black\n"
    f << "        -\n"
    f << "          name: head" << i.to_s << "\n"
    f << "          collision_geometry:\n"
    f << "            type: cylinder\n"
    f << "            radius: 0.1\n"
    f << "            height: 0.25\n"
    f << "            material: black\n"
    f << "          config:\n"
    f << "            position: [" << (hed_x + x).to_s << ", " << (hed_y + y).to_s << ", 1.585]\n"
    f << "            orientation: " << quat << "\n"
    f << "          visualization_geometry:\n"
    f << "            type: box\n"
    f << "            dims: [0.01, 0.01, 0.01]\n"
    f << "            material: black\n"
end

# = Generate a Table at the target location = #
def generate_table( f, x, y, i )
    f << "  <!-- Table " << i.to_s << "-->\n"
    #Simulation's copy
    f << "  <rosparam ns=\"simulation/simulator/obstacles\">\n"
    f << "    table" << i.to_s << ":\n"
    f << "      type: obstacle \n"
    f << "      geometries:\n"
    f << "        -\n"
    f << "          name: top" << i.to_s << "\n"
    f << "          collision_geometry:\n"
    f << "            type: box\n"
    f << "            dims: [1.0,1.0,0.06]\n"
    f << "            material: white\n"
    f << "          config:\n"
    f << "            position: [" << x.to_s << ", " << y.to_s << ", 0.77]\n"
    f << "            orientation: [0,0,0,1]\n"
    f << "        -\n"
    f << "          name: leg_1_" << i.to_s << "\n"
    f << "          collision_geometry:\n"
    f << "            type: box\n"
    f << "            dims: [0.06,0.06,0.78]\n"
    f << "            material: gold\n"
    f << "          config:\n"
    f << "            position: [" << (x+0.465).to_s << ", " << (y+0.465).to_s << ", 0.39]\n"
    f << "            orientation: [0,0,0,1]\n"
    f << "        -\n"
    f << "          name: leg_2_" << i.to_s << "\n"
    f << "          collision_geometry:\n"
    f << "            type: box\n"
    f << "            dims: [0.06,0.06,0.78]\n"
    f << "            material: gold\n"
    f << "          config:\n"
    f << "            position: [" << (x-0.465).to_s << ", " << (y+0.465).to_s << ", 0.39]\n"
    f << "            orientation: [0,0,0,1]\n"
    f << "        -\n"
    f << "          name: leg_3_" << i.to_s << "\n"
    f << "          collision_geometry:\n"
    f << "            type: box\n"
    f << "            dims: [0.06,0.06,0.78]\n"
    f << "            material: gold\n"
    f << "          config:\n"
    f << "            position: [" << (x-0.465).to_s << ", " << (y-0.465).to_s << ", 0.39]\n"
    f << "            orientation: [0,0,0,1]\n"
    f << "        -\n"
    f << "          name: leg_4_" << i.to_s << "\n"
    f << "          collision_geometry:\n"
    f << "            type: box\n"
    f << "            dims: [0.06,0.06,0.78]\n"
    f << "            material: gold\n"
    f << "          config:\n"
    f << "            position: [" << (x+0.465).to_s << ", " << (y-0.465).to_s << ", 0.39]\n"
    f << "            orientation: [0,0,0,1]\n"
    f << "  </rosparam>\n\n"
    #Planning's Copy
    f << "  <rosparam ns=\"planning/world_model/simulator/obstacles\">\n"
    f << "    table" << i.to_s << ":\n"
    f << "      type: obstacle \n"
    f << "      geometries:\n"
    f << "        -\n"
    f << "          name: top" << i.to_s << "\n"
    f << "          collision_geometry:\n"
    f << "            type: box\n"
    f << "            dims: [1.0,1.0,0.06]\n"
    f << "            material: white\n"
    f << "          config:\n"
    f << "            position: [" << x.to_s << ", " << y.to_s << ", 0.77]\n"
    f << "            orientation: [0,0,0,1]\n"
    f << "        -\n"
    f << "          name: leg_1_" << i.to_s << "\n"
    f << "          collision_geometry:\n"
    f << "            type: box\n"
    f << "            dims: [0.06,0.06,0.78]\n"
    f << "            material: gold\n"
    f << "          config:\n"
    f << "            position: [" << (x+0.465).to_s << ", " << (y+0.465).to_s << ", 0.39]\n"
    f << "            orientation: [0,0,0,1]\n"
    f << "        -\n"
    f << "          name: leg_2_" << i.to_s << "\n"
    f << "          collision_geometry:\n"
    f << "            type: box\n"
    f << "            dims: [0.06,0.06,0.78]\n"
    f << "            material: gold\n"
    f << "          config:\n"
    f << "            position: [" << (x-0.465).to_s << ", " << (y+0.465).to_s << ", 0.39]\n"
    f << "            orientation: [0,0,0,1]\n"
    f << "        -\n"
    f << "          name: leg_3_" << i.to_s << "\n"
    f << "          collision_geometry:\n"
    f << "            type: box\n"
    f << "            dims: [0.06,0.06,0.78]\n"
    f << "            material: gold\n"
    f << "          config:\n"
    f << "            position: [" << (x-0.465).to_s << ", " << (y-0.465).to_s << ", 0.39]\n"
    f << "            orientation: [0,0,0,1]\n"
    f << "        -\n"
    f << "          name: leg_4_" << i.to_s << "\n"
    f << "          collision_geometry:\n"
    f << "            type: box\n"
    f << "            dims: [0.06,0.06,0.78]\n"
    f << "            material: gold\n"
    f << "          config:\n"
    f << "            position: [" << (x+0.465).to_s << ", " << (y-0.465).to_s << ", 0.39]\n"
    f << "            orientation: [0,0,0,1]\n"
    f << "  </rosparam>\n\n"
end

# = = = = = = #
# = Driver  = #
def main()
    f = File.open("benchmark.launch", "w")

    #Output basic info and tag
    f << "<!-- Auto generated launch file for Baxter: " << NUM_SYSTEMS.to_s << " " << NUM_BINS.to_s << " " << MODE.to_s << " -->\n"
    f << "<launch>\n"

    #Also output the space names xml include
    f << "  <rosparam command=\"load\" file=\"$(find prx_input)/templates/spaces/space_types.yaml\"/>\n\n"

    # = = = = = = = = = = = = =
    #Parameters: Simulation
    # = = = = = = = = = = = = =
    f << "  <!-- = = = = = = = -->\n"
    f << "  <!--   Simulation  -->\n"
    f << "  <!-- = = = = = = = -->\n"

    f << "  <!-- Load independent files -->\n"
    f << "  <rosparam command=\"load\" ns=\"simulation\" file=\"$(find prx_input)/templates/applications/empty_application.yaml\"/>\n"
    f << "  <rosparam command=\"load\" ns=\"simulation\" file=\"$(find prx_input)/templates/simulators/collision_stop_simulator.yaml\"/>\n\n"

    f << "  <!-- Load template files -->\n"
    f << "  <rosparam command=\"load\" ns=\"simulation/controller\" file=\"$(find prx_input)/templates/controllers/consumer.yaml\"/>\n"
    f << "  <rosparam command=\"load\" ns=\"simulation/plant\" file=\"$(find baxter)input/urdf/baxter.yaml\"/>\n\n"

    # = = = = = = = = = = = = =
    #Parameters: Planning
    # = = = = = = = = = = = = =
    f << "  <!-- = = = = = = = -->\n"
    f << "  <!--    Planning   -->\n"
    f << "  <!-- = = = = = = = -->\n\n"

    #Planning contexts need to be set up
    f << "  <!-- Planning Contexts... We will need to make a list of these... -->\n"
    f << "  <rosparam ns=\"planning/world_model/planning_contexts\">\n"
    for i in 1 .. NUM_SYSTEMS
        f << "    space" << i.to_s << ":\n"
        f << "      default:\n"
        f << "        type: hide_mapping\n"
    end
    f << "  </rosparam>\n\n"

    #Now, create the consumer mappings as well
    f << "  <!-- Consumer Mappings -->\n"
    f << "  <rosparam ns=\"planning\">\n"
    f << "  system_mapping:\n"
    for i in 1 .. NUM_SYSTEMS
        f << "  -\n"
        f << "    pair: [simulator/consumer" << i.to_s << "/baxter" << i.to_s << ", world_model/simulator/baxter" << i.to_s << "]\n"
    end
    f << "  consumer: simulator/consumer1\n"
    f << "  </rosparam>\n\n"

    f << "  <rosparam command=\"load\" ns=\"planning\" file=\"$(find prx_input)/templates/spaces/space_types.yaml\"/>\n"
    f << "  <rosparam command=\"load\" ns=\"planning/world_model\" file=\"$(find prx_input)/templates/simulators/collision_stop_simulator.yaml\"/>\n"
    f << "  <rosparam command=\"load\" ns=\"planning\" file=\"$(find prx_input)/templates/planning_applications/single_query_application.yaml\"/>\n"
    f << "  <rosparam command=\"load\" ns=\"planning/task_planner/\" file=\"$(find prx_input)/templates/task_planners/single_shot.yaml\"/>\n\n"

    f << "  <!-- Load template files -->\n"
    f << "  <rosparam command=\"load\" ns=\"planning/plant\" file=\"$(find baxter)input/urdf/baxter.yaml\"/>\n\n"

    #Output planning seed stuff
    f << "  <rosparam ns=\"planning\">\n"
    if SPECIFY_RANDOM
        f << "    random_seed: "
        if(RANDOM_RANDOM)
            f << rand(65535).to_s
        else
            f << RANDOM_SEED.to_s
        end
    end
    f << "\n"
    f << "    consumer_mapping:\n"
    for i in 1 .. NUM_SYSTEMS
        f << "      -\n"
        f << "        pair: [simulator/consumer" << i.to_s << ", space" << i.to_s << "]\n"
    end
    f << "  </rosparam>\n\n"

    # = = = = = = = = = = = = =
    #Parameters: Environment
    # = = = = = = = = = = = = =
    f << "  <!-- = = = = = = = -->\n"
    f << "  <!--  Environment  -->\n"
    f << "  <!-- = = = = = = = -->\n\n"

    #Before we get into the nitty-gritty, there are objects to be manipulated!
    f << "  <rosparam command=\"load\" ns=\"simulation/cup\" file=\"$(find manipulation)/input/simulation/plants/cups.yaml\"/>\n"
    f << "  <rosparam ns=\"simulation/cup\">\n"
    f << "    state_space:\n"
    f << "        min: [-2, -3, -100, -1, -1, -1, -1]\n"
    f << "        max: [22, 3, 100, 1, 1, 1, 1]\n"
    f << "        scale: [0.025, 0.025, 0.05, 0.5, 0.5, 0.5, 0.5]\n"
    f << "  </rosparam>\n\n"

    #First of all, we need to figure out which benchmark we are running
    # = = = = = = =
    # = Shelves
    # = = = = = = =
    if MODE == 0 #Kiva Shelves
        #First, let's place some Kiva Shelves and the Baxters
        for i in 1 .. NUM_SYSTEMS
            #Generate the shelf and Baxter in front of it
            base_x = 1.7*i
            base_y = 1.25
            generate_shelf( f, base_x, base_y, i )
            generate_baxter( f, base_x, 0, Math::PI/2.0, i )
            # = Then, let's generate some "Objects" in the shelf =
            f << "  <!-- Objects -->\n"
            bins = Array.new(12,0)
            for j in 1 .. NUM_OBJECTS
                done = false
                #Select a bin
                while !done
                    b = rand(12)
                    if( bins[b] == 0 )
                        bins[b] = 1
                        done = true
                    end
                end
            end
            for j in 0 .. 11
                #If this bin was selected to have something
                if bins[j] == 1
                    #Compute the x_off depending on the bin
                    x_off = 0
                    xmod = j%4
                    if xmod == 0
                        x_off = COL_1
                    elsif xmod == 1
                        x_off = COL_2
                    elsif xmod == 2
                        x_off = COL_3
                    elsif xmod == 3
                        x_off = COL_4
                    end
                    #Then Compute the z_off based on the bin
                    z_off = 0
                    zmod = j/4
                    if zmod == 0
                        z_off = ROW_4
                    elsif zmod == 1
                        z_off = ROW_5
                    elsif zmod == 2
                        z_off = ROW_6
                    end
                    #Then output the stuff
                    f << "  <rosparam command=\"load\" ns=\"simulation/simulator/subsystems/cup" << i.to_s << "_" << j.to_s << "\">\n"
                    f << "    template: \"cup\"\n"
                    f << "    initial_state: [" << (x_off + base_x).to_s << ", " << (Y_OFF + base_y).to_s << ", " << (z_off + 0.07).to_s << ", 0,0,0,1]\n"
                    f << "  </rosparam>\n\n"

                    #print "Putting the object in bin: " << j.to_s << ": " << x_off.to_s << " , " << z_off.to_s << "\n"
                end
            end
            # = TDK: Generate some random poses
            if TDK_GENERATE_POSES
                tdk = File.open("kiva_poses_" << i.to_s << ".txt", "w")
                tdkout = File.open("kiva_front_poses_" << i.to_s << ".txt", "w")
                for j in 1 .. TDK_NUM_POSES
                    #Select a random bin to find its offsets
                    r = rand(12)
                    #Compute the x_off depending on the bin
                    x_off = 0
                    xmod = r%4
                    if xmod == 0
                        x_off = COL_1
                    elsif xmod == 1
                        x_off = COL_2
                    elsif xmod == 2
                        x_off = COL_3
                    elsif xmod == 3
                        x_off = COL_4
                    end
                    #Then Compute the z_off based on the bin
                    z_off = 0
                    zmod = r/4
                    if zmod == 0
                        z_off = ROW_4
                    elsif zmod == 1
                        z_off = ROW_5
                    elsif zmod == 2
                        z_off = ROW_6
                    end
                    #Compute a random offset from the center
                    # TODO: Should probably parameterize the shelf sizes
                    x_rand = (rand() * (0.195 - 2.0*(TDK_POSE_BUFFER))) - (0.0975 - TDK_POSE_BUFFER)
                    y_rand = (rand() * (0.190 - 2.0*(TDK_POSE_BUFFER))) - (0.1050 - TDK_POSE_BUFFER)
                    tdk << "-\n"
                    tdk << "  pose: [" << (x_off + x_rand).to_s << "," << (Y_OFF + base_y + y_rand + 0.05).to_s << "," << (z_off + 0.07 + 2.0).to_s << ",0,0,0,1]\n"
                    tdkout << "-\n"
                    tdkout << "  pose: [" << (x_off + x_rand).to_s << "," << (Y_OFF + base_y - 0.055).to_s << "," << (z_off + 2.102).to_s << ",0.00,0.00,0.7071067811800000324495841,0.7071067811800000324495841]\n"
                end
                tdk.close()
                tdkout.close()
            end

        end
        f << "  <!-- Bins -->\n"
        #Now, we need to place bins where appropriate
        if NUM_BINS >= 1
            generate_bin( f, 0.7, 0, 0, 1 )
        end
        if NUM_BINS >= 2
            generate_bin( f, 1.0 + 1.7*NUM_SYSTEMS, 0, Math::PI, 2 )
        end
        if NUM_BINS > 2
            print "Requesting more bins than this setup knows how to place!\n"
        end
    # = = = = = = =
    # = Long Table
    # = = = = = = =
    else #Zhe long table
        #First, let's make some tables
        for i in 1 .. (NUM_SYSTEMS+1)
            generate_table( f, i-1, 0, i )
        end
        #Then, place some Baxters around that loooong table
        for i in 1 .. NUM_SYSTEMS
            theta = (Math::PI/2.0) * ((-2*(i%2))+1)
            generate_baxter( f, i-0.5, (2*(i%2))-1, theta, i )
        end
        #Now, let's put some items on that long table
        nr_bins = (NUM_SYSTEMS+1) * 25
        bins = Array.new(nr_bins, 0)
        for j in 1 .. (NUM_OBJECTS * (NUM_SYSTEMS+1))
            done = false
            #Select bins
            while !done
                b = rand(nr_bins)
                if( bins[b] == 0 )
                    bins[b] = 1
                    done = true
                end
            end
        end
        #Now, for every one of those that was selected, go ahead and put the object
        for j in 0 .. (nr_bins-1)
            if bins[j] == 1
                #Place it heah!!
                x_pos = 0.2 * (j % (5*(NUM_SYSTEMS+1))) - 0.4
                y_pos = 0.2 * (j / (5*(NUM_SYSTEMS+1))) - 0.4
                #Then output the stuff
                f << "  <rosparam command=\"load\" ns=\"simulation/simulator/subsystems/cup" << j.to_s << "\">\n"
                f << "    template: \"cup\"\n"
                f << "    initial_state: [" << x_pos.to_s << ", " << y_pos.to_s << ", 0.87, 0,0,0,1]\n"
                f << "  </rosparam>\n\n"

                #print "Putting the object in bin: " << j.to_s << ": " << x_pos.to_s << " , " << y_pos.to_s << "\n"
            end
        end
        #Finally, let's place some order bins
        if NUM_BINS > 0
            #The first bin goes on the top side next to the first Baxter
            generate_bin( f, -0.5, 1, 0, 1 )
        end
        if NUM_BINS > 1
            #The second bin goes on the top side next to the (second-to-)last Baxter
            generate_bin( f, 2*(((NUM_SYSTEMS-1)/2.0).floor) + 1.5, 1, Math::PI, 2 )
        end
        if NUM_BINS > 2
            #The third bin goes on the bottom side next to the first Baxter
            generate_bin( f, 0.5, -1, 0, 3 )
        end
        if NUM_BINS > 3
            #The fourth bin goes on the bottom side next to the first Baxter
            generate_bin( f, 2*(( (NUM_SYSTEMS) /2.0).floor) + 0.5, -1, 0, 4 )
        end
        if NUM_BINS > 4
            #TOO MANY BINS
            print "Requesting more bins than this setup knows how to place!\n"
        end
    end

    # = = = = = = = = = = = = =
    #Parameters: Visualization
    # = = = = = = = = = = = = =
    f << "  <!-- = = = = = = = -->\n"
    f << "  <!-- Visualization -->\n"
    f << "  <!-- = = = = = = = -->\n"
    f << "  <include file=\"$(find baxter)input/visualization.launch\"/>\n\n"

    #Define the Nodes to launch
    if CREATE_PLANNING
        f << "  <!-- Define the planning node -->\n"
        f << "  <node name=\"planning\" pkg=\"prx_planning\" type=\"prx_planning\" required=\"true\" launch-prefix=\"\" output=\"screen\" args=\"planning\" />\n"
    end

    f << "  <!-- Define the simulation node -->\n"
    f << "  <node name=\"simulation\" pkg=\"prx_simulation\" type=\"prx_simulation\" required=\"true\" launch-prefix=\"\" output=\"screen\" args=\"simulation\" />\n"

    #Close out the launch tag: we're finally done!
    f << "\n</launch>\n\n"
end

main()

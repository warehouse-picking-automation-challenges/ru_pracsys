import sys

quadrotor = {'plant':'quadrotor',
             'planners':['rrt_star_shooting','drain_rrt'],
             'obstacles':'quadrotor_obstacles',
             'query':'quadrotor_query',
             'parameter_override':'quadrotor_override',
             'model_override':'quadrotor_override'
            }
point = {'plant':'point',
         'planners':['rrt_star','rrt','drain_rrt'],
         'obstacles':'recreate_rrt_star',
         'query':'point_query',
         'parameter_override':'point_override',
             'model_override':'point_override'
        }
pendulum = {'plant':'pendulum',
             'planners':['rrt','rrt_star_shooting','drain_rrt'],
             'obstacles':'NONE',
             'query':'pendulum_query',
             'parameter_override':'pendulum_override',
             'model_override':'pendulum_override'
            }
cart_pole = {'plant':'cart_pole',
         'planners':['rrt','rrt_star_shooting','drain_rrt'],
         'obstacles':'cart_pole_obstacles',
         'query':'cart_pole_query',
         'parameter_override':'cart_pole_override',
             'model_override':'cart_pole_override'
        }
acrobot = {'plant':'acrobot',
         'planners':['rrt','rrt_star_shooting','drain_rrt'],
         'obstacles':'acrobot_obstacles',
         'query':'acrobot_query',
         'parameter_override':'acrobot_override',
             'model_override':'acrobot_override'
        }
# double_integrator = {'plant':'double_integrator',
#          'planners':['rrt','drain_rrt'],
#          'obstacles':'double_integator_obstacles',
#          'query':'double_integrator_query',
#          'parameter_override':'double_integrator_override',
#              'model_override':'double_integrator_override'
#         }
physics_car = {'plant':'physics_car',
         'planners':['rrt','rrt_star_shooting','drain_rrt'],
         'obstacles':'car_obstacles',
         'query':'physics_car_query',
         'parameter_override':'physics_car',
             'model_override':'physics_car'
        }
rover = {'plant':'rover',
     'planners':['drain_rrt'],
     'obstacles':'rover_obstacles',
     'query':'rover_query',
     'parameter_override':'rover_override',
     'model_override':'rover_override'
    }

problems = [rover,point]
count = 0
dups = 1
h = open("all_names.txt",'w');
j = open("all_params.launch",'w');
j.write("<launch>\n");
for num in xrange(0,dups):
    for problem in problems:
        for planner in problem['planners']:
    	    count = count + 1;
            planner_name = problem['plant']+"_"+planner+"_"+str(num); #"n"+str(delta_near)+"_d"+str(delta_drain)+"_num"+str(num);
            h.write(planner_name+"\n")
            with open("launch_params.launch",'r') as g:
                data = g.readlines();
                for y in data:
                    if "PLANNER_NAME" in y:
                        y = y.replace("PLANNER_NAME",planner_name);
                    if "RANDOM_SEED" in y:
                        y = y.replace("RANDOM_SEED",str(num));
                    if "QUERY_FILE" in y:
                        y = y.replace("QUERY_FILE",problem['query']);
                    if "PLANNER_FILE" in y:
                        y = y.replace("PLANNER_FILE",planner);
                    if "PLANT_FILE" in y:
                        y = y.replace("PLANT_FILE",problem['plant']);
                    if "OBSTACLE_FILE" in y:
                        if problem['obstacles'] != 'NONE':
                            y = y.replace("OBSTACLE_FILE",problem['obstacles']);
                        else:
                            y = ''
                    if "MODEL_OVERRIDE_FILE" in y:
                        y = y.replace("MODEL_OVERRIDE_FILE",problem['model_override']);
                    if "OVERRIDE_FILE" in y:
                        y = y.replace("OVERRIDE_FILE",problem['parameter_override']);
                    j.write(y);
            j.write("\n");
j.write("<rosparam>\n spaces: {AirControl: EEE, Airplane: EEEREEE, DynamicRigidBody: EEEQQQQEEEEEE, EMPTY: '',HeliControl: EERR, ODE_B: E, ODE_J: E, OneLink: RE, R: R, RR: RR, RallyCar: EEEEREEE,RallyCarTask: EEER, Rd: E, Rdd: E, SE2: EER, SE3: EEEQQQQ, SOCar: EEREE, TWO_D_BODY: EEEE,ThreeLink: RRREEE, Time: E, TwoLink: RREE, Vector: ER, WheelControl: REE, X: E,XR: ER, XXd: EE, XXdRRd: EERE, XY: EE, XYZ: EEE, Xd: E, XdRd: EE, XdYd: EE, Xdd: E,XddRdd: EE, XddYdd: EE, bomb_state: EEE}</rosparam>\n");
j.write("</launch>");

print count;

#visualization creation

selected_system = point
planner = "drain_rrt";
random_seed = 0;

j = open("visualize.launch",'w');
j.write("<launch>\n");
num = random_seed;
problem = selected_system;
planner_name = problem['plant']+"_"+planner+"_"+str(num);
with open("vis_launch_params.launch",'r') as g:
    data = g.readlines();
    for y in data:
        if "PLANNER_NAME" in y:
            y = y.replace("PLANNER_NAME",planner_name);
        if "PLANT_FILE" in y:
            y = y.replace("PLANT_FILE",problem['plant']);
        if "OBSTACLE_FILE" in y:
            if problem['obstacles'] != 'NONE':
                y = y.replace("OBSTACLE_FILE",problem['obstacles']);
            else:
                y = ''
        if "MODEL_OVERRIDE_FILE" in y:
            y = y.replace("MODEL_OVERRIDE_FILE",problem['model_override']);
        j.write(y);
j.write("\n");
j.write("<rosparam>\n spaces: {AirControl: EEE, Airplane: EEEREEE, DynamicRigidBody: EEEQQQQEEEEEE, EMPTY: '',HeliControl: EERR, ODE_B: E, ODE_J: E, OneLink: RE, R: R, RR: RR, RallyCar: EEEEREEE,RallyCarTask: EEER, Rd: E, Rdd: E, SE2: EER, SE3: EEEQQQQ, SOCar: EEREE, TWO_D_BODY: EEEE,ThreeLink: RRREEE, Time: E, TwoLink: RREE, Vector: ER, WheelControl: REE, X: E,XR: ER, XXd: EE, XXdRRd: EERE, XY: EE, XYZ: EEE, Xd: E, XdRd: EE, XdYd: EE, Xdd: E,XddRdd: EE, XddYdd: EE, bomb_state: EEE}</rosparam>\n");
j.write("</launch>");



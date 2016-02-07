import sys

#delta_nears = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15];
#delta_drains =[1,2,3,4,5,6,7,8,9,10,11,12,13,14,15];

# done 1-5,1-5; 6-10,6-10; 11-15,11-15 ; 6-10, 2-5

delta_drains  = [0];
delta_nears = [0];
count = 0;
#  Near Drain Reduction
#   X    X     X 
#   3 -> 011 -> Drain and Reduction
dups = 1
h = open("generated_files/all_names.txt",'w');
j = open("generated_files/all_params.launch",'w');
j.write("<launch>");
for num in xrange(0,dups):
    for temp_delta in delta_nears:
        for temp_delta_d in delta_drains:
            delta_near = temp_delta;
            delta_drain = temp_delta_d;
            if delta_near < delta_drain and delta_near!=0:
    	       continue;
    	    count = count + 1;
            planner_name = "n"+str(delta_near)+"_d"+str(delta_drain)+"_num"+str(num);
            h.write(planner_name+"\n")
            f = open("generated_files/"+planner_name+".yaml",'w');
            with open("params.yaml",'r') as g:
                data = g.readlines();
                for y in data:
                    if "NNNNNN" in y:
                        y = y.replace("NNNNNN",planner_name);
                    elif "delta_drain" in y:
                        y = y.replace("0",str(delta_drain));
                    elif "delta_near" in y:
                        y = y.replace("0",str(delta_near));
                    elif "random_seed" in y:
                        y = y.replace("####",str(num));
                    f.write(y);
            j.write("<rosparam file=\""+planner_name+".yaml\" />\n")
            f = open("generated_files/"+planner_name+".launch",'w');
            with open("template_launch.launch",'r') as g:
                data = g.readlines();
                for y in data:
                    if "NNNNNN" in y:
                        y = y.replace("NNNNNN",planner_name);
                    f.write(y);
j.write("</launch>");

print count;
#!/bin/bash
for i in {1..10}
do
    roslaunch baxter.launch
    mv ~/repositories/pracsys/prx_output/single_shot_output/planning_planner1.txt ~/repositories/pracsys/prx_output/single_shot_output/baxter_irs_$i.txt
done



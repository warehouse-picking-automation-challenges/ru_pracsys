#!/bin/bash
for i in {1..500}
do
    roslaunch corridor.launch
    mv $PRACSYS_PATH/prx_output/pno_data/dim_2_planning.txt $PRACSYS_PATH/prx_output/pno_data/corridor_$i.txt
done



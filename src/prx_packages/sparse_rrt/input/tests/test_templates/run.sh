#!/bin/bash
while read line
do
#	echo $line
	rosrun prx_planning prx_planning $line
done < all_names.txt
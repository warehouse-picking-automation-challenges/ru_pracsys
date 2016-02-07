#!/bin/bash
echo -n Password: 
read -s password
ts=`sshpass -p rethink ssh ruser@drogon.cs.rutgers.edu date '+%s'`
ts="@"$ts
echo $'\nBelow you see the Baxter timestamp\n'$ts'  '
echo $password | sudo -S date +%s -s $ts
echo "Above you see your machine's local timestamp"

#!/bin/bash

envname=ubremen-r27-docks
instancesetname="ubremen-r27"
denvxml="d-envs/$envname.xml"
instancefolder="instances/$instancesetname"
maxtime=600000

radius=27
gridedgelen="65"
maxspeed="0.05"
timestep=`echo "import math;print(int(math.ceil($gridedgelen/(2*$maxspeed))))" | python`
ngoals="3"

echo "Will use timestep $timestep"

mkdir -p $instancefolder
rm $instancefolder/*
cp prepare.sh $instancefolder/

instance=0
for nagents in "1" "5" "10" "15" "20" "30" "35"
do
    for seed in {1..10}
    do
        let instance=instance+1
	    # create a problem instance file
	    instancename="$instance"
	    instancefile=$instancefolder/$instancename.xml

        ## ConflictGenerator
        java -XX:+UseSerialGC -cp solver.jar -Dlog4j.configuration="file:$PWD/log4j.custom" tt.jointeuclid2ni.probleminstance.generator.GenerateRTInstance -env $denvxml -nagents $nagents -radius $radius -ngoals $ngoals -maxspeed $maxspeed  -seed $seed -outfile $instancefile
               
        algs="BASE ORCA DFCFS"
        
        for alg in $algs
        do
		    summaryprefix="$envname;$instance;$nagents;$radius;$seed;$timestep;$maxtime;$alg;"
	        echo -method $alg -problemfile $instancefile -ntasks 4 -timestep $timestep -maxtime $maxtime -timeout $maxtime -seed $seed -summaryprefix "$summaryprefix" >> $instancefolder/data.in           
        done

	    echo Finished instance no $instance. Agents: $nagents. Seed: $seed.
    done        
done
echo "env;instance;nagents;radius;seed;timestep;maxtime;alg;status;time;simfinished" > $instancefolder/head
echo Done. Created $instance instances at $envname environment. Instances stored in $instancefolder.


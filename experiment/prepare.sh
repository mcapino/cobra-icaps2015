#!/bin/bash

#--ubremen
envname=ubremen-r27-docks
instancesetname="ubremen-r27"
radius=27
gridedgelen="65"
maxtime=600000
agents="1 5 10 15 20 30 35"

#--warehouse
#envname=warehouse-r25-docks
#instancesetname="warehouse-r25"
#radius=25
#gridedgelen="54"
#maxtime=600000
#agents="1 5 10 15 20 30 40 50"

#--empty-hall
#envname=empty-hall-r25-docks
#instancesetname="empty-hall-r25"
#radius=20
#gridedgelen="54"
#maxtime=600000
#agents="1 5 10 15 20 30 40 50"


denvxml="d-envs/$envname.xml"
instancefolder="instances/$instancesetname"
maxspeed="0.05"
timestep=`echo "import math;print(int(math.ceil($gridedgelen/(2*$maxspeed))))" | python`
ntasks="4"

echo "Preparing instanceset $instancesetname. Will use timestep $timestep."

mkdir -p $instancefolder
rm $instancefolder/*
cp prepare.sh $instancefolder/

instance=0
for nagents in $agents
do
    for seed in {1..10}
    do
        let instance=instance+1
	    # create a problem instance file
	    instancename="$instance"
	    instancefile=$instancefolder/$instancename.xml

        ## ConflictGenerator
        java -XX:+UseSerialGC -cp solver.jar -Dlog4j.configuration="file:$PWD/log4j.custom" tt.jointeuclid2ni.probleminstance.generator.GenerateRTInstance -env $denvxml -nagents $nagents -radius $radius -maxspeed $maxspeed  -seed $seed -outfile $instancefile
               
        algs="ORCA COBRA"
        
        for alg in $algs
        do
		    summaryprefix="$envname;$instance;$nagents;$radius;$seed;$timestep;$maxtime;$alg;"
	        echo -method $alg -problemfile $instancefile -ntasks $ntasks -timestep $timestep -maxtime $maxtime -timeout $maxtime -seed $seed -summaryprefix "$summaryprefix" >> $instancefolder/data.in           
        done

	    echo Finished instance no $instance. Agents: $nagents. Seed: $seed.
    done        
done
echo "env;instance;nagents;radius;seed;timestep;maxtime;alg;status;avgBase;varBase;avgWait;varWait;avgPlan;varPlan;avgPWindow;varPWindow;avgProlongT;varProlongT;avgProlongR;varProlongR;makespan" > $instancefolder/head
echo Done. Created $instance instances at $envname environment. Instances stored in $instancefolder.


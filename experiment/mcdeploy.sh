#!/bin/bash
dir="icaps2015"
instanceset=$1
tasks="8" # number of tasks to be run in parallel in one josb
ptime="600" # runtime limit for one task in seconds 
pmem="1" # max memory of one task in gb
frontend="tarkil.metacentrum.cz"
storage="storage-praha1.metacentrum.cz" #storage-praha1.metacentrum.cz 
storagedir="/auto/praha1/capino"

cpus=$(($tasks+1))
mem=$(($pmem*$tasks+1)) #in gb

cluster="linux" #cl_mandos cl_hildor cl_perian #cl_mandos cl_nympha cl_luna cl_hermes

if [ -f "instances/$instanceset/data.in" ]
then 
	cd instances/$instanceset	 
	walltime=3600 # what will be the maximum wallclock time of one job/chunk in metacentrum? 
    runs=$(cat data.in | wc -l)
    splitby=$((($walltime/$ptime)*$tasks)) # computes how many runs we can fit into one chunk 
    chunks=$(($runs/$splitby+1)) # how many chunks do we need?
	echo "There is total $runs runs in data.in. Spliting into $chunks chunks with maxmimum duration $walltime s. Each chunk will contain $splitby runs." 
	
	sleep 5
    
    rm data.*.in
	split -l $splitby --additional-suffix=".in" data.in data.
	
	echo "Copying files"
	ssh $frontend rm -r $storagedir/$dir/instances/$instanceset   
	ssh $frontend mkdir $storagedir/$dir/instances/$instanceset
	scp * $storage:$dir/instances/$instanceset
	
	for f in data.*.in
	do
		echo "Adding $f ("$(cat $f | wc -l) "runs) to the queue. Will run via parallel."	
    	ssh $frontend qsub -l walltime=$walltime -l nodes=1:ppn=$cpus:$cluster,mem=$mem"gb",scratch=500mb -v env=$instanceset,cpus=$tasks,in=$f,out=$f.out,mem=$pmem $storagedir/$dir/run.parallel.metacentrum.sh
	done	

    echo "Running the experiment at $frontend. Use \"ssh $frontend qstat -u capino\" to monitor the progress"
else	
    echo "instanceset identifier not valid"	 
fi



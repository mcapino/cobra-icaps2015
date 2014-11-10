#!/bin/bash
instanceset=$1
nresults=$(ls instances/$instanceset/*.out | wc -l)
echo "Joining results from $nresults files."

if [ -n "$nresults" ]
then
	cat "instances/$instanceset/head" > "instances/$instanceset/data.out.head"	
	cat instances/$instanceset/*.out >> instances/$instanceset/data.out.head
else
	echo "invalid instance set"
fi

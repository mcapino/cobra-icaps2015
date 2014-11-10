#!/bin/bash
instanceset=$1
if [ -d "instances/$instanceset" ]
then
	scp -r storage-praha1.metacentrum.cz:icaps2015/instances/$instanceset instances/
	./join.sh $instanceset      
else	
    echo "instanceset identifier is expected"	
fi


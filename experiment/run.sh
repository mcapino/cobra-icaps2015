#!/bin/bash
instanceset=$1
cpus=$2
datain=$3
dataout=$4
mem=$5

./parallel_experiments.sh -j solver.jar -c instances/$instanceset/$datain -o instances/$instanceset/$dataout -m $mem"g" -v -s $cpus/:  

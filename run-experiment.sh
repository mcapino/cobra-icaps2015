#!/bin/bash

# clean temporary files and previous results
echo -e "\n >>>> Cleaning\n"
mvn clean
rm -f experiment/instances/empty-hall-r25/*.out.*
rm -f experiment/instances/ubremen-r27/*.out.*
rm -f experiment/instances/warehouse-r25/*.out.*
rm -rf experiment/plots
rm -f expermient/solver.jar

# compile the experimental framework
echo -e "\n >>>> Compiling the codebase\n"
mvn package

# copy the binaries to the experiment folder
cp target/map4rt-1.0-SNAPSHOT-jar-with-dependencies.jar experiment/solver.jar

cd experiment

CPUS=4 # no of CPU to be used for the experiment (how many simulations to run in parallel)
MEM=4  # maximum memory used by one simulation run in GBs


# run the experiment in empty-hall environment
echo -e "\n >>>> Running the experiment in empty hall...\n"
./run.sh empty-hall-r25 $CPUS data.in data.out $MEM  
./addhead.sh empty-hall-r25


# run the experiment in ubremen environment
echo -e "\n >>>> Running the experiment in ubremen...\n"
./run.sh ubremen-r27 $CPUS data.in data.out $MEM
./addhead.sh ubremen-r27

# run the experiment in warehouse environment
echo -e "\n >>>> Running the experiment in warehouse...\n"  
./run.sh warehouse-r25 $CPUS data.in data.out $MEM
./addhead.sh warehouse-r25


# make plots from the data
mkdir -p plots
Rscript makeplots.r

echo -e "\n >>>> Done! The plots should be in experiment/plots directory. \n"

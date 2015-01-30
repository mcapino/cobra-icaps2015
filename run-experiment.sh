#!/bin/bash

# compile the both the experimental framework
echo -e "\nCompiling the codebase\n"
mvn package

# copy the resulting binaries to the experiment folder
cp target/map4rt-1.0-SNAPSHOT-jar-with-dependencies.jar experiment/solver.jar

cd experiment

CPUS=1 # no of CPU to be used for the experiment
MEM=1  # maximum memory used by one simulation run in GBs


# run the experiment in empty-hall environment
echo -e "\nRunning the experiment in empty hall...\n"
./run.sh empty-hall-r25 $CPUS data.in data.out $MEM  
./addhead.sh empty-hall-r25


# run the experiment in ubremen environment
echo -e "\nRunning the experiment in ubremen...\n"
./run.sh ubremen-r27 $CPUS data.in data.out $MEM
./addhead.sh ubremen-r27

# run the experiment in warehouse environment
echo -e "\nRunning the experiment in warehouse...\n"  
./run.sh warehouse-r25 $CPUS data.in data.out $MEM
./addhead.sh warehouse-r25


# make plots from the data
mkdir -p plots
Rscript makeplots.r

echo -e "Done! The new plots have been generated in /plots directory."










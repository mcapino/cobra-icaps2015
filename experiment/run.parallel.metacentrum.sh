#!/bin/bash
trap 'rm -r $SCRATCHDIR' TERM EXIT

# copy the data from storage to scratchdir
DATADIR="/storage/praha1/home/capino/icaps2015"
cp -r $DATADIR/* $SCRATCHDIR || exit 1
cd $SCRATCHDIR || exit 2
rm instances/$env/*.out

# run the computation
module add jdk-8
module add parallel

./run.sh $env $cpus $in $out $mem

# copy the data back to datadir
cp instances/$env/*.out $DATADIR/instances/$env || {  trap - TERM EXIT && echo "Copy output data failed. Copy them manualy from `hostname`" >&2 ; exit 1;}
cp instances/$env/*.log $DATADIR/instances/$env || {  trap - TERM EXIT && echo "Copy output data failed. Copy them manualy from `hostname`" >&2 ; exit 1;}


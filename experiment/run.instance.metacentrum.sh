#!/bin/bash

if [ -n "$1" ] 
then
  args=$1
fi	

if [ -n "$2" ] 
then
  outfile=$2
fi	

cd /storage/praha1/home/capino/icra2015
module add jdk-7

java -XX:+UseSerialGC -jar solver.jar $args > $outfile

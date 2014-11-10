#!/bin/bash

env=$1
instance=$2
method=$3

echo "using data file:" $env/data.in

args=$(cat $env/data.in | grep $env/$instance.xml | grep $method)

echo will use arguments: $args
 
java -XX:+UseSerialGC -jar solver.jar $args -showvis

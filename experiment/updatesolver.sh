#!/bin/bash
cd /home/capino/projects/trajectorytools
mvn deploy
cd /home/capino/projects/deconflictiontools
mvn deploy
cd /home/capino/projects/orca
mvn deploy
cd /home/capino/projects/map4rt
mvn package
cp /home/capino/projects/map4rt/target/map4rt-1.0-SNAPSHOT-jar-with-dependencies.jar /home/capino/projects/map4rt/experiment/solver.jar

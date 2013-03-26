package cz.agents.deconfliction.util;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Random;
import java.util.Set;

import org.jgrapht.Graph;

import cz.agents.alite.trajectorytools.graph.spatial.SpatialGraphs;
import cz.agents.alite.trajectorytools.util.OrientedPoint;
import cz.agents.alite.trajectorytools.util.SpatialPoint;
import cz.agents.alite.trajectorytools.util.Waypoint;

public class GraphAgentMissionGenerator {

    public static class Mission {
        public Waypoint start;
        public Waypoint end;

        public Mission(Waypoint start, Waypoint end) {
            super();
            this.start = start;
            this.end = end;
        }

        @Override
        public String toString() {
            return "Mission [start=" + start + ", end=" + end + "]";
        }


    }

    public static Mission[] generateSuperconflicts(Graph<Waypoint, ?>  maneuvers, int[] sizes, SpatialPoint[] centers, int[] radiuses) {

        ArrayList<Mission> resultList = new ArrayList<Mission>();

        for (int i = 0; i < sizes.length; i++) {
            Mission[] superconflict = generateSuperconflict(maneuvers, sizes[i], centers[i], radiuses[i]);
            resultList.addAll(Arrays.asList(superconflict));
        }

        return (Mission[]) resultList.toArray(new Mission[resultList.size()]);
    }


    public static Mission[] generateTest(Graph<Waypoint, ?> maneuvers) {
        Mission[] missions = new Mission[3];

        missions[0] = new Mission(SpatialGraphs.getNearestVertex(maneuvers, new SpatialPoint(2,0,0)), SpatialGraphs.getNearestVertex(maneuvers, new SpatialPoint(3,10,0)));

        missions[1] = new Mission(SpatialGraphs.getNearestVertex(maneuvers, new SpatialPoint(5,5,0)), SpatialGraphs.getNearestVertex(maneuvers, new SpatialPoint(10, 5, 0)));
        missions[2] = new Mission(SpatialGraphs.getNearestVertex(maneuvers, new SpatialPoint(7.5,7.5,0)), SpatialGraphs.getNearestVertex(maneuvers, new SpatialPoint(7.0,0,0)));

        return missions;
    }

    public static Mission[] generateRandom(Random random, Graph<Waypoint, ?>  maneuvers, int nAgents, double maxx, double maxy, double separation) {
        Mission[] missions = new Mission[nAgents];
        Set<Waypoint> startWaypoints = new HashSet<Waypoint>();
        Set<Waypoint> endWaypoints = new HashSet<Waypoint>();

        for (int i = 0; i < nAgents; i++) {
            Waypoint startWp;
            Waypoint endWp;

            SpatialPoint start;
            SpatialPoint end;
            do {
                start = new SpatialPoint(random.nextDouble() * maxx, random.nextDouble() * maxy, 0);
                startWp = SpatialGraphs.getNearestVertex(maneuvers,start);

                end = new SpatialPoint(random.nextDouble() * maxx, random.nextDouble() * maxy, 0);
                endWp = SpatialGraphs.getNearestVertex(maneuvers,end);
            }
            while (!separated(startWp, startWaypoints, 1.5*separation) || !separated(endWp, endWaypoints, 1.5*separation) || startWp.equals(endWp));
            startWaypoints.add(startWp);
            endWaypoints.add(endWp);

            missions[i] = new Mission(startWp, endWp);
        }
        return missions;
    }

    public static Mission[] generateRandomSimilarLength(Random random, Graph<Waypoint, ?>  maneuvers, int nAgents, double maxx, double maxy, double radius, double separation) {
        Mission[] missions = new Mission[nAgents];
        Set<Waypoint> startWaypoints = new HashSet<Waypoint>();
        Set<Waypoint> endWaypoints = new HashSet<Waypoint>();

        for (int i = 0; i < nAgents; i++ ) {
            Waypoint startWp;
            Waypoint endWp;

            SpatialPoint start;
            do {
                double r = radius * (0.5 + random.nextDouble());
                start = new SpatialPoint(random.nextDouble() * maxx, random.nextDouble() * maxy, 0);
                startWp = SpatialGraphs.getNearestVertex(maneuvers,start);

                double angle = (2*Math.PI*random.nextDouble());

                SpatialPoint end = new SpatialPoint(start.x + Math.cos(angle)*r, start.y + Math.sin(angle)*r,0);
                endWp = SpatialGraphs.getNearestVertex(maneuvers,end);
            }
            while (!separated(startWp, startWaypoints, 1.5*separation) || !separated(endWp, endWaypoints, 1.5*separation) || startWp.equals(endWp));
            startWaypoints.add(startWp);
            endWaypoints.add(endWp);

            missions[i] = new Mission(startWp, endWp);
        }
        return missions;
    }

    public static Mission[] generateSuperconflict(Graph<Waypoint, ?> maneuvers, int nAgents, SpatialPoint center, double circleRadius) {
        Mission[] startEnds = new Mission[nAgents];

        for (int i = 0; i < nAgents; i++ ) {

            double angle = i*(2*Math.PI/nAgents);
            OrientedPoint startPoint = new OrientedPoint(center.x + Math.cos(angle)*circleRadius, center.y + Math.sin(angle)*circleRadius,0,0,1,0);
            OrientedPoint endPoint = new OrientedPoint(center.x + Math.cos(angle+Math.PI)*circleRadius, center.y + Math.sin(angle+Math.PI)*circleRadius,0,0,1,0);

            Waypoint start = SpatialGraphs.getNearestVertex(maneuvers,startPoint);
            Waypoint end = SpatialGraphs.getNearestVertex(maneuvers,endPoint);

            startEnds[i] = new Mission(start, end);
        }
        return startEnds;

    }

    public static Mission[] generateSpiralSuperconflict(Graph<Waypoint, ?>  maneuvers, int nAgents, SpatialPoint center, double minRadius, double maxRadius, double angleOffset) {
        Mission[] startEnds = new Mission[nAgents];

        double radiusStep = (maxRadius - minRadius) / (nAgents-1);

        for (int i = 0; i < nAgents; i++ ) {

            double circleRadius = minRadius + i * radiusStep;
            double angle = i*(2*Math.PI/(double)nAgents) + angleOffset;
            OrientedPoint startPoint = new OrientedPoint(center.x + Math.cos(angle)*circleRadius, center.y + Math.sin(angle)*circleRadius,0,0,1,0);
            OrientedPoint endPoint = new OrientedPoint(center.x + Math.cos(angle+Math.PI)*circleRadius, center.y + Math.sin(angle+Math.PI)*circleRadius,0,0,1,0);

            Waypoint start = SpatialGraphs.getNearestVertex(maneuvers,startPoint);
            Waypoint end = SpatialGraphs.getNearestVertex(maneuvers,endPoint);

            startEnds[i] = new Mission(start, end);
        }
        return startEnds;

    }

    private static boolean separated(Waypoint candidate, Set<Waypoint> usedWaypoints, double separation) {
        for (Waypoint wp : usedWaypoints) {
            if (wp.distance(candidate) <= separation) {
                return false;
            }
        }
        return true;
    }
}

package cz.agents.deconfliction.util;

import static org.junit.Assert.*;

import java.util.LinkedList;
import java.util.List;

import org.jgrapht.SingleEdgeGraphPath;
import org.junit.Before;
import org.junit.Test;

import com.sun.org.apache.bcel.internal.generic.NEW;

import cz.agents.deconfliction.maneuvergraph.EightWayConstantSpeedGridGraph;
import cz.agents.deconfliction.maneuvergraph.FourWayConstantSpeedGridGraph;
import cz.agents.deconfliction.maneuvergraph.Maneuver;
import org.jgrapht.Graph;
import cz.agents.deconfliction.trajectory.PiecewiseLinearTrajectory;
import cz.agents.deconfliction.trajectory.ShortestManeuverTrajectory;
import cz.agents.alite.trajectorytools.trajectory.Trajectory;
import cz.agents.deconfliction.trajectory.TrajectoryApproximation;
import cz.agents.alite.trajectorytools.util.SeparationDetector;
import cz.agents.deconfliction.util.Point;
import cz.agents.alite.trajectorytools.util.Waypoint;

public class ConflictDetectorTest {

    Graph<Waypoint, SpatialManeuver> graph = new EightWayConstantSpeedGridGraph(1, 1, 4, 4, 1.0);
    double SEPARATION = 0.05;
    double MAXTIME = 50.0;

    @Test
    public void hasConflict() {
        Trajectory t1 = new FastestManeuverTrajectory(graph, 0.5, graph.getNearestWaypoint(new Point(0,0,0)), graph.getNearestWaypoint(new Point(1,1,0)), MAXTIME, SEPARATION/2);
        Trajectory t2 = new FastestManeuverTrajectory(graph, 0.5, graph.getNearestWaypoint(new Point(1,1,0)), graph.getNearestWaypoint(new Point(0,0,0)), MAXTIME, SEPARATION/2);

        List<Trajectory> collection = new LinkedList<Trajectory>();
        collection.add(t1);
        collection.add(t2);

        assertTrue(SeparationDetector.hasConflict(collection, SEPARATION));
    }

    @Test
    public void hasConflict2() {
        Trajectory t1 = new FastestManeuverTrajectory(graph, 0.5, graph.getNearestWaypoint(new Point(0,0,0)), graph.getNearestWaypoint(new Point(1,1,0)), MAXTIME, SEPARATION/2);
        Trajectory t2 = new FastestManeuverTrajectory(graph, 0.5, graph.getNearestWaypoint(new Point(1,1,0)), graph.getNearestWaypoint(new Point(0,0,0)), MAXTIME, SEPARATION/2);

        List<Trajectory> collection = new LinkedList<Trajectory>();
        collection.add(t2);

        assertTrue(SeparationDetector.hasConflict(t1, collection, SEPARATION));
    }

    @Test
    public void hasNoConflict() {
        Trajectory t1 = new FastestManeuverTrajectory(graph, 0.5, graph.getNearestWaypoint(new Point(0,0,0)), graph.getNearestWaypoint(new Point(1,1,0)), MAXTIME, SEPARATION/2);
        Trajectory t2 = new FastestManeuverTrajectory(graph, 0.5, graph.getNearestWaypoint(new Point(1,0,0)), graph.getNearestWaypoint(new Point(0,0,0)), MAXTIME, SEPARATION/2);

        List<Trajectory> collection = new LinkedList<Trajectory>();
        collection.add(t1);
        collection.add(t2);

        assertFalse(SeparationDetector.hasConflict(collection, SEPARATION));
    }

    @Test
    public void hasConflictOneEdge() {

        Waypoint w1 = graph.getNearestWaypoint(new Point(0.25,0.25,0));
        Waypoint w2 = graph.getNearestWaypoint(new Point(0.5,0.5,0));
        Waypoint w3 = graph.getNearestWaypoint(new Point(0.5,0.25,0));
        Waypoint w4 = graph.getNearestWaypoint(new Point(0.25,0.5,0));


        Trajectory t1 = new FastestManeuverTrajectory(graph, 0.9, w1, w2, MAXTIME, SEPARATION/4.0);

        Trajectory t2 = new FastestManeuverTrajectory(graph, 0.9, w3, w4, MAXTIME, SEPARATION/4.0);

        List<Trajectory> collection1 = new LinkedList<Trajectory>();
        collection1.add(t1);
        collection1.add(t2);


        Trajectory t3 = new FastestManeuverTrajectory(graph, 0.9, w1, w2, MAXTIME, SEPARATION/4.0);

        Trajectory t4 = new FastestManeuverTrajectory(graph, 0.8, w3, w4, MAXTIME, SEPARATION/4.0);

        List<Trajectory> collection2 = new LinkedList<Trajectory>();
        collection1.add(t3);
        collection1.add(t4);

        assertTrue(SeparationDetector.hasConflict(collection1, SEPARATION));
        assertFalse(SeparationDetector.hasConflict(collection2, SEPARATION));
    }


}

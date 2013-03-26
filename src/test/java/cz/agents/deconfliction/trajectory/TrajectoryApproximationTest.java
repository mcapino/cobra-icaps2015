package cz.agents.deconfliction.trajectory;

import static org.junit.Assert.*;

import org.junit.Before;
import org.junit.Test;

import com.sun.org.apache.bcel.internal.generic.NEW;

import cz.agents.deconfliction.maneuvergraph.FourWayConstantSpeedGridGraph;
import org.jgrapht.Graph;
import cz.agents.deconfliction.trajectory.ShortestManeuverTrajectory;
import cz.agents.alite.trajectorytools.trajectory.Trajectory;
import cz.agents.deconfliction.trajectory.TrajectoryApproximation;
import cz.agents.deconfliction.util.Point;

public class TrajectoryApproximationTest {

    Graph<Waypoint, SpatialManeuver> graph = new FourWayConstantSpeedGridGraph(1, 1, 5, 5, 1.0);

    @Before
    public void setUp() throws Exception {

    }

    @Test
    public void testLong() {
        Trajectory t = new FastestManeuverTrajectory(graph, 0.5, graph.getNearestWaypoint(new Point(0,0,0)), graph.getNearestWaypoint(new Point(1,1,0)), 10.0, 1.0/3.0);
        TrajectoryApproximation ta = t.getApproximation();

        assertTrue(ta.getFirstSampleNo() == 2);
        assertTrue(ta.length() == 6);
    }

    @Test
    public void testShorterThanSamplingInterval() {
        Trajectory t = new FastestManeuverTrajectory(graph, 16.0, graph.getNearestWaypoint(new Point(0,0,0)), graph.getNearestWaypoint(new Point(1,1,0)), 100.0, 10.0);
        TrajectoryApproximation ta = t.getApproximation();

        assertTrue(ta.getFirstSampleNo() == 2);
        assertTrue(ta.length() == 0);
    }

    @Test
    public void testShorterThanSamplingIntervalOneSample() {
        Trajectory t = new FastestManeuverTrajectory(graph, 19.0, graph.getNearestWaypoint(new Point(0,0,0)), graph.getNearestWaypoint(new Point(1,1,0)), 100.0, 10.0);
        TrajectoryApproximation ta = t.getApproximation();

        assertTrue(ta.getFirstSampleNo() == 2);
        assertTrue(ta.length() == 1);
    }

}

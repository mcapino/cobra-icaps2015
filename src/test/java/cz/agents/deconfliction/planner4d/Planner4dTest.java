package cz.agents.deconfliction.planner4d;

import static org.junit.Assert.assertTrue;

import java.util.LinkedList;

import org.jgrapht.Graph;
import org.jgrapht.GraphPath;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

import cz.agents.alite.trajectorytools.graph.spatial.maneuvers.SpatialManeuver;
import cz.agents.alite.trajectorytools.trajectory.Trajectory;
import cz.agents.alite.trajectorytools.util.Waypoint;
import cz.agents.deconfliction.solver.central.Waypoint;
import cz.agents.deconfliction.solver.central.ODSolver;

public class Planner4dTest {


    @BeforeClass
    public static void setUpBeforeClass() throws Exception {
    }

    @Before
    public void setUp() throws Exception {
    }


    @Test
    public void testTrivialMovingObstacle() {
        Graph<Waypoint, SpatialManeuver> graph = new FourWayConstantSpeedGridGraph(10, 10, 1, 1, 1.0);
        double separation = 0.5;


        Waypoint a1s = graph.getNearestWaypoint(new Point(0,0,0));
        Waypoint a1g = graph.getNearestWaypoint(new Point(10,10,0));

        Waypoint a2s = graph.getNearestWaypoint(new Point(10,0,0));
        Waypoint a2g = graph.getNearestWaypoint(new Point(0,0,0));

        long start = System.nanoTime();
        Planner4d planner = new Planner4d(a1s, 10.0, a1g, graph, separation, 100, separation/4);
        Trajectory traj1 = planner.solveTrajectory();
        System.out.println("Planning time: " + (System.nanoTime() - start) / 1e6 + "ms, states expanded: " + planner.expandedStatesCounter);
        System.out.println(traj1);

        LinkedList<Trajectory> hardConstraints = new LinkedList<Trajectory>();
        hardConstraints.add(traj1);

        start = System.nanoTime();
        planner = new Planner4d(a2s, 0.0, a2g, graph, separation, separation/4, Double.POSITIVE_INFINITY, hardConstraints, new LinkedList<Trajectory>());
        GraphPath<Waypoint, SpatialManeuver> path2 = planner.solve();
        System.out.println("Planning time: " + (System.nanoTime() - start) / 1e6 + "ms, states expanded: " + planner.expandedStatesCounter);
        System.out.println(path2);

        assertTrue(path2.toString().equals("[(#2 : #2), (#2 : #0)]"));
        assertTrue(path2.getWeight() == 11.0);
    }

    @Test
    public void testMovingObstacle() {
        Graph<Waypoint, SpatialManeuver> graph = new EightWayConstantSpeedGridGraph(10, 10, 1, 1, 1.0);
        double separation = 0.5;

        Waypoint a1s = graph.getNearestWaypoint(new Point(0,0,0));
        Waypoint a1g = graph.getNearestWaypoint(new Point(10,10,0));

        Waypoint a2s = graph.getNearestWaypoint(new Point(10,0,0));
        Waypoint a2g = graph.getNearestWaypoint(new Point(0,10,0));

        long start = System.nanoTime();
        Planner4d planner = new Planner4d(a1s, 0.0, a1g, graph, separation, 100, separation / 4.0);
        Trajectory traj1 = planner.solveTrajectory();
        System.out.println("Planning time: " + (System.nanoTime() - start) / 1e6 + "ms, states expanded: " + planner.expandedStatesCounter);
        System.out.println(traj1);

        LinkedList<Trajectory> hardConstraints = new LinkedList<Trajectory>();
        hardConstraints.add(traj1);

        start = System.nanoTime();
        planner = new Planner4d(a2s, 0.0, a2g, graph, separation, separation / 4.0, Double.POSITIVE_INFINITY, hardConstraints, new LinkedList<Trajectory>());
        GraphPath<Waypoint, SpatialManeuver> path2 = planner.solve();
        System.out.println("Planning time: " + (System.nanoTime() - start) / 1e6 + "ms, states expanded: " + planner.expandedStatesCounter);
        System.out.println(path2);
    }

    @Test
    public void testSpeedAgainstOD() {
        Graph<Waypoint, SpatialManeuver> graph = new EightWayConstantSpeedGridGraph(10, 10, 40, 40, 1.0);
        double separation = 0.5;

        Waypoint a1s = graph.getNearestWaypoint(new Point(0,0,0));
        Waypoint a1g = graph.getNearestWaypoint(new Point(10,7,0));

        long start = System.nanoTime();
        Planner4d planner = new Planner4d(a1s, 0.0, a1g, graph, separation, 100, separation / 4.0);
        Trajectory traj1 = planner.solveTrajectory();
        System.out.println("4DPlanner Planning time: " + (System.nanoTime() - start) / 1e6 + "ms, states expanded: " + planner.getExpandedStatesCounter());
        System.out.println(traj1);

        Waypoint agent1s = new Waypoint(a1s, null, 0.0);
        Waypoint agent1g = new Waypoint(a1g, null, 0.0);

        start = System.nanoTime();
        ODSolver solver1 = new ODSolver(agent1s, agent1g, graph, separation, separation / 4.0, Double.POSITIVE_INFINITY, new LinkedList<Trajectory>(), new LinkedList<Trajectory>());
        Trajectory agent1Trajectory = solver1.solveTrajectories()[0];
        System.out.println("ODSolver Planning time: " + (System.nanoTime() - start) / 1e6 + "ms, states expanded: " + solver1.getExpandedStatesCounter());
        System.out.println(traj1);

    }



}

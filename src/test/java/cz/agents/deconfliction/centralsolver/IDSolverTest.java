package cz.agents.deconfliction.centralsolver;

import org.jgrapht.GraphPath;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

import cz.agents.deconfliction.maneuvergraph.EightWayConstantSpeedGridGraph;
import cz.agents.deconfliction.maneuvergraph.Maneuver;
import org.jgrapht.Graph;
import cz.agents.deconfliction.solver.central.Waypoint;
import cz.agents.deconfliction.solver.central.IDSolver;
import cz.agents.alite.trajectorytools.trajectory.Trajectory;
import cz.agents.deconfliction.util.Point;
import cz.agents.alite.trajectorytools.util.Waypoint;

public class IDSolverTest {

    double SEPARATION = 0.4;
    double APPROX_SAMPLING_INTERVAL = 0.2;

    @BeforeClass
    public static void setUpBeforeClass() throws Exception {
    }

    @Before
    public void setUp() throws Exception {
    }

    @Test
    public void test() {
        Graph<Waypoint, SpatialManeuver> graph = new EightWayConstantSpeedGridGraph(1, 1, 4, 4, 1.0);

        Waypoint a1s = graph.getNearestWaypoint(new Point(0,0,0));
        Waypoint a1g = graph.getNearestWaypoint(new Point(1,1,0));

        Waypoint a2s = graph.getNearestWaypoint(new Point(1,0,0));
        Waypoint a2g = graph.getNearestWaypoint(new Point(0,1,0));

        Waypoint agent1s = new Waypoint(a1s, null, 0.0);
        Waypoint agent2s = new Waypoint(a2s, null, 0.0);

        Waypoint agent1g = new Waypoint(a1g, null, 0.0);
        Waypoint agent2g = new Waypoint(a2g, null, 0.0);

        IDSolver solver = new IDSolver(new Waypoint[] { null, agent1s, null,
                agent2s, null }, new Waypoint[] { null, agent1g, null,
                agent2g, null }, graph, SEPARATION, APPROX_SAMPLING_INTERVAL);
        Trajectory[] result = solver.solve();

        for (Trajectory t: result) {
            System.out.println(t);
        }
    }

}

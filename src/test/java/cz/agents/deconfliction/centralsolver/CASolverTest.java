package cz.agents.deconfliction.centralsolver;

import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

import cz.agents.deconfliction.maneuvergraph.EightWayConstantSpeedGridGraph;
import org.jgrapht.Graph;
import cz.agents.deconfliction.solver.central.Waypoint;
import cz.agents.deconfliction.solver.central.CASolver;
import cz.agents.alite.trajectorytools.trajectory.Trajectory;
import cz.agents.deconfliction.util.Point;
import cz.agents.alite.trajectorytools.util.Waypoint;

public class CASolverTest {

    double SEPARATION = 0.5;
    double MAXTIME = 100.0;
    double APPROX_SAMPLING_INTERVAL = SEPARATION / 4.0;

    @BeforeClass
    public static void setUpBeforeClass() throws Exception {
    }

    @Before
    public void setUp() throws Exception {
    }
    
    @Test
    public void testDeterministicProperty() {
    	final int N = 100;

    	for (int i = 0; i < N; i++) {     
        	Graph<Waypoint, SpatialManeuver> graph = new EightWayConstantSpeedGridGraph(10, 10, 4, 4, 1.0);
	        Waypoint a1s = graph.getNearestWaypoint(new Point(10,5,0));
	        Waypoint a1g = graph.getNearestWaypoint(new Point(0, 5,0));
	
	        Waypoint a2s = graph.getNearestWaypoint(new Point(0, 5,0));
	        Waypoint a2g = graph.getNearestWaypoint(new Point(10,5,0));
	
	        Waypoint agent1s = new Waypoint(a1s, null, 0.0);
	        Waypoint agent2s = new Waypoint(a2s, null, 0.0);
	
	        Waypoint agent1g = new Waypoint(a1g, null, 0.0);
	        Waypoint agent2g = new Waypoint(a2g, null, 0.0);

			CASolver solver = new CASolver(
					new Waypoint[] { agent1s, agent2s }, 
					new Waypoint[] { agent1g, agent2g }, 
					graph, SEPARATION, MAXTIME, APPROX_SAMPLING_INTERVAL);
	        Trajectory[] result = solver.solve();
	        
	        //System.out.println();
	        //System.out.println(result[0]);
	        System.out.println(result[1]);	        
	        try {
				Thread.sleep(40);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
        }

    }
    
    @Test
    public void testSimpleGraph() {
        Graph<Waypoint, SpatialManeuver> graph = new EightWayConstantSpeedGridGraph(10, 10, 1, 1, 1.0);

        Waypoint a1s = graph.getNearestWaypoint(new Point(0,0,0));
        Waypoint a1g = graph.getNearestWaypoint(new Point(10,10,0));

        Waypoint a2s = graph.getNearestWaypoint(new Point(10,0,0));
        Waypoint a2g = graph.getNearestWaypoint(new Point(0,10,0));

        Waypoint agent1s = new Waypoint(a1s, null, 0.0);
        Waypoint agent2s = new Waypoint(a2s, null, 0.0);

        Waypoint agent1g = new Waypoint(a1g, null, 0.0);
        Waypoint agent2g = new Waypoint(a2g, null, 0.0);


		CASolver solver = new CASolver(
				new Waypoint[] { null, agent1s, null,	agent2s, null }, 
				new Waypoint[] { null, agent1g, null,	agent2g, null }, 
				graph, SEPARATION, MAXTIME, APPROX_SAMPLING_INTERVAL);
        Trajectory[] result = solver.solve();

        for (Trajectory t: result) {
            System.out.println(t);
        }
    }
}

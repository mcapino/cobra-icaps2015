package cz.agents.deconfliction.centralsolver;

import static org.junit.Assert.*;

import java.util.LinkedList;

import org.jgrapht.GraphPath;
import org.jgrapht.alg.AStarShortestPath;
import org.jgrapht.alg.DijkstraShortestPath;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

import cz.agents.deconfliction.maneuvergraph.EightWayConstantSpeedGridGraph;
import cz.agents.deconfliction.maneuvergraph.FourWayConstantSpeedGridGraph;
import cz.agents.deconfliction.maneuvergraph.Maneuver;
import org.jgrapht.Graph;
import cz.agents.deconfliction.maneuvergraph.RandomWaypointGraph;
import cz.agents.deconfliction.solver.central.Waypoint;
import cz.agents.deconfliction.solver.central.ODSolver;
import cz.agents.alite.trajectorytools.trajectory.Trajectory;
import cz.agents.deconfliction.util.Point;
import cz.agents.alite.trajectorytools.util.Waypoint;

public class ODSolverTest {

	double SEPARATION = 0.2;
    double APPROX_SAMPLING_INTERVAL = 0.1;

    @BeforeClass
    public static void setUpBeforeClass() throws Exception {
    }

    @Before
    public void setUp() throws Exception {
    }
    
    @Test
    public void testDeterministicProperty() {
    	final int N = 100;
    	Graph<Waypoint, SpatialManeuver> graph = new EightWayConstantSpeedGridGraph(1, 1, 4, 4, 1.0);
        
        for (int i = 0; i < N; i++) {        
	        Waypoint a1s = graph.getNearestWaypoint(new Point(1.0,0.5,0));
	        Waypoint a1g = graph.getNearestWaypoint(new Point(0.0,0.5,0));
	
	        Waypoint a2s = graph.getNearestWaypoint(new Point(0.0,0.5,0));
	        Waypoint a2g = graph.getNearestWaypoint(new Point(1.0,0.5,0));
	
	        Waypoint agent1s = new Waypoint(a1s, null, 0.0);
	        Waypoint agent2s = new Waypoint(a2s, null, 0.0);
	
	        Waypoint agent1g = new Waypoint(a1g, null, 0.0);
	        Waypoint agent2g = new Waypoint(a2g, null, 0.0);
	        
	        try {
				Thread.sleep(4);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
	        
	        ODSolver solver1 = new ODSolver(agent1s, agent1g, graph, SEPARATION, APPROX_SAMPLING_INTERVAL, Double.POSITIVE_INFINITY, new LinkedList<Trajectory>(), new LinkedList<Trajectory>());
	        Trajectory agent1Trajectory = solver1.solveTrajectories()[0];
	        LinkedList<Trajectory> hardConstraints = new LinkedList<Trajectory>();
	        hardConstraints.add(agent1Trajectory);
	        
	        ODSolver solver2 = new ODSolver(agent2s, agent2g, graph, SEPARATION, APPROX_SAMPLING_INTERVAL, Double.POSITIVE_INFINITY, hardConstraints, new LinkedList<Trajectory>());
	        GraphPath<Waypoint, SpatialManeuver>[] result = solver2.solve();
	        
	        //System.out.println();
	        //System.out.println(agent1Trajectory);
	        System.out.println(result[0].getStartVertex() + " " + result[0]);	        	        
        }

    }


    @Test
    public void test1() {
    	Graph<Waypoint, SpatialManeuver> graph = new EightWayConstantSpeedGridGraph(1, 1, 1, 1, 1.0);

        Waypoint a1s = graph.getNearestWaypoint(new Point(0,0,0));
        Waypoint a1g = graph.getNearestWaypoint(new Point(1,1,0));

        Waypoint a2s = graph.getNearestWaypoint(new Point(1,0,0));
        Waypoint a2g = graph.getNearestWaypoint(new Point(0,1,0));

        Waypoint agent1s = new Waypoint(a1s, null, 0.0);
        Waypoint agent2s = new Waypoint(a2s, null, 0.0);

        Waypoint agent1g = new Waypoint(a1g, null, 0.0);
        Waypoint agent2g = new Waypoint(a2g, null, 0.0);

        ODSolver solver = new ODSolver(new Waypoint[]{null, agent1s, null, agent2s, null}, new Waypoint[]{null, agent1g, null, agent2g, null}, graph, SEPARATION, APPROX_SAMPLING_INTERVAL);
        GraphPath<Waypoint, SpatialManeuver>[] result = solver.solve();

        assertTrue(Math.abs(result[1].getWeight() - 1.4) < 0.1);
        assertTrue(Math.abs(result[3].getWeight() - 2.0) < 0.1);
    }
    
    
    @Test
    public void testOptimalityOnRandomGraphsAgainstDijkstra() {
        final int N = 100;

        long odTime = 0;
        long dijkstraTime = 0;

        for (int seed=0; seed<N; seed++) {

            Graph<Waypoint, SpatialManeuver> graph = new RandomWaypointGraph(5, 5, 65, 6, seed);
            Waypoint start = graph.getNearestWaypoint(new Point(0, 0, 0));
            Waypoint end = graph.getNearestWaypoint(new Point(5.0, 5.0, 0));
            
            long startTime = System.nanoTime();

            ODSolver odsolver = new ODSolver(new Waypoint(start, null, 0.0), new Waypoint(end, null, 0.0), graph, 0.1, 0.1, Double.POSITIVE_INFINITY, new LinkedList<Trajectory>(), new LinkedList<Trajectory>());
            GraphPath<Waypoint, SpatialManeuver> odPath = odsolver.solve()[0];

            odTime += System.nanoTime()-startTime;



            startTime = System.nanoTime();
            DijkstraShortestPath<Waypoint, SpatialManeuver> dijkstra = new DijkstraShortestPath<Waypoint, SpatialManeuver>(
                    graph, start, end);

            GraphPath<Waypoint, SpatialManeuver> dijkstraPath = dijkstra
                    .getPath();

            dijkstraTime += System.nanoTime()-startTime;

            if (odPath != dijkstraPath && !odPath.getEdgeList().equals(dijkstraPath.getEdgeList())) {
                System.out.println("The two methods found different paths.");
            }

            if (dijkstraPath != null) {
                if (Math.abs(dijkstraPath.getWeight()-odPath.getWeight()) > 0.0001) {
                    System.out.println("The paths found by dijkstra and A* have different length!");
                    System.out.println(" --------- " + seed);
                    System.out.println(" g: " + graph);
                    System.out.println("Dijkstra");
                    printPath(dijkstraPath);
                    System.out.println("A*");
                    printPath(odPath);
                }
            }

            if (dijkstraPath != null)
                assertTrue(Math.abs(dijkstraPath.getWeight() - odPath.getWeight()) < 0.001 );
        }

        System.out.println("Examined " + N + " random graphs. Dijkstra avg. time: " + dijkstraTime/(1000000*N) + "ms" + ". OD avg. time: " + odTime/(1000000*N)+"ms.");
    }


    @Test
    public void testExperimental() {
        Graph<Waypoint, SpatialManeuver> graph = new EightWayConstantSpeedGridGraph(1, 1, 1, 1, 1.0);

        Waypoint a1s = graph.getNearestWaypoint(new Point(0,0,0));
        Waypoint a1g = graph.getNearestWaypoint(new Point(1,1,0));

        Waypoint a2s = graph.getNearestWaypoint(new Point(1,0,0));
        Waypoint a2g = graph.getNearestWaypoint(new Point(0,1,0));

        Waypoint agent1s = new Waypoint(a1s, null, 0.0);
        Waypoint agent2s = new Waypoint(a2s, null, 0.0);

        Waypoint agent1g = new Waypoint(a1g, null, 0.0);
        Waypoint agent2g = new Waypoint(a2g, null, 0.0);

        ODSolver solver = new ODSolver(new Waypoint[]{null, agent1s, null, agent2s, null}, new Waypoint[]{null, agent1g, null, agent2g, null}, graph, SEPARATION, APPROX_SAMPLING_INTERVAL);
        GraphPath<Waypoint, SpatialManeuver>[] result = solver.solve();


        for (GraphPath<Waypoint,Maneuver> path: result) {
            System.out.println(path);
        }
    }
    
    private void printPath(GraphPath<Waypoint, SpatialManeuver> path) {
        if (path == null) {
            System.out.println("no path found");
            return;
        }
        System.out.print(path.getStartVertex() + ", ");
        for (SpatialManeuver edge : path.getEdgeList()) {
            if (edge == null) {
                System.out.println("Edge is null");
            }
            System.out.print(edge.getSource() + "-"
                    + edge.getTarget() + ", ");
        }
        System.out.print(path.getEndVertex());
        System.out.print(" Path weight:" + path.getWeight());
        System.out.println();
   }

}

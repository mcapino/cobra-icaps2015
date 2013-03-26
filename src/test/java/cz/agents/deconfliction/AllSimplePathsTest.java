package cz.agents.deconfliction;

import static org.junit.Assert.*;

import java.util.List;

import org.jgrapht.GraphPath;
import org.jgrapht.alg.*;
import org.jgrapht.graph.DefaultWeightedEdge;
import org.junit.Test;

import cz.agents.deconfliction.maneuvergraph.EightWayConstantSpeedGridGraph;
import cz.agents.deconfliction.maneuvergraph.FourWayConstantSpeedGridGraph;
import cz.agents.deconfliction.maneuvergraph.Maneuver;
import org.jgrapht.Graph;
import cz.agents.deconfliction.maneuvergraph.NWayConstantSpeedGraph<Waypoint, SpatialManeuver>;
import cz.agents.deconfliction.util.AllSimpleGraphPathsGenerator;
import cz.agents.deconfliction.util.Point;
import cz.agents.alite.trajectorytools.util.Waypoint;
import cz.agents.deconfliction.waypointgraph.DefaultWaypointGraph;

public class AllSimplePathsTest {

    @Test
    public void test() {

       Graph<Waypoint, SpatialManeuver> graph = new FourWayConstantSpeedGridGraph(5, 5, 1, 1, 1.0);
       Waypoint start = graph.getNearestWaypoint(new Point(0,0,0));
       Waypoint end = graph.getNearestWaypoint(new Point(5.0,5.0,0));

       AllSimpleGraphPathsGenerator generator = new AllSimpleGraphPathsGenerator(graph, start, end);

       System.out.println("Graph: \n" + graph);


       int pathcounter = 0;
       GraphPath<Waypoint, SpatialManeuver> path;
       while ((path = generator.nextGraphPath()) != null) {
           pathcounter++;

           System.out.println("New path:");
           System.out.print(path.getStartVertex() + ", ");
           for(SpatialManeuver edge : path.getEdgeList()) {
               if (edge == null) {
                   System.out.println("Edge is null");
               }
               System.out.print(graph.getEdgeSource(edge) + "-" + graph.getEdgeTarget(edge)+", ");
           }
           System.out.print(path.getEndVertex());
           System.out.println();

           if (pathcounter % 1000 == 0)
               System.out.println(pathcounter);


       } ;

       System.out.println("Found " + pathcounter + " simple paths");

       //List<GraphPath<Waypoint, DefaultWeightedEdge>> paths = ksp.getPaths(graph.getNearestWaypoint(new Point(5,5,0)));
       //System.out.println(paths.size());

    }

}

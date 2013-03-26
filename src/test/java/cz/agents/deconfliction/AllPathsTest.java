package cz.agents.deconfliction;

import static org.junit.Assert.*;

import java.text.DecimalFormat;
import java.text.Format;
import java.util.List;

import org.jgrapht.GraphPath;
import org.jgrapht.alg.*;
import org.jgrapht.graph.DefaultWeightedEdge;
import org.junit.Test;

import cz.agents.deconfliction.maneuvergraph.CompleteGridWaypointGraph;
import cz.agents.deconfliction.maneuvergraph.EightWayConstantSpeedGridGraph;
import cz.agents.deconfliction.maneuvergraph.FourWayConstantSpeedGridGraph;
import cz.agents.deconfliction.maneuvergraph.NWayConstantSpeedGraph<Waypoint, SpatialManeuver>;
import cz.agents.deconfliction.util.AllGraphPathsGenerator;
import cz.agents.deconfliction.util.AllSimpleGraphPathsGenerator;
import cz.agents.deconfliction.util.Point;
import cz.agents.alite.trajectorytools.util.Waypoint;
import cz.agents.deconfliction.waypointgraph.DefaultWaypointGraph;

public class AllPathsTest {

    @Test
    public void test() {

       DefaultWaypointGraph graph = new CompleteGridWaypointGraph(5, 5, 6, 6);
       Waypoint start = graph.getNearestWaypoint(new Point(0,0,0));
       Waypoint end = graph.getNearestWaypoint(new Point(5.0,4.0,0));

       AllGraphPathsGenerator generator = new AllGraphPathsGenerator(graph, start, end, 6.52);

       System.out.println("Graph: \n" + graph);


       int pathcounter = 0;
       GraphPath<Waypoint, DefaultWeightedEdge> path;
       while ((path = generator.nextGraphPath()) != null) {
           pathcounter++;

           System.out.println("New path:");
           System.out.print(path.getStartVertex() + ", ");
           for(DefaultWeightedEdge edge : path.getEdgeList()) {
               if (edge == null) {
                   System.out.println("Edge is null");
               }
               System.out.print(graph.getEdgeSource(edge) + "-" + graph.getEdgeTarget(edge)+", ");
           }
           System.out.print(path.getEndVertex() + " (" + new DecimalFormat("#.##").format(path.getWeight()) + ")");
           System.out.println();

           if (pathcounter % 1000 == 0)
               System.out.println(pathcounter);


       } ;

       System.out.println("Found " + pathcounter + " paths");

       //List<GraphPath<Waypoint, DefaultWeightedEdge>> paths = ksp.getPaths(graph.getNearestWaypoint(new Point(5,5,0)));
       //System.out.println(paths.size());

    }

}

package cz.agents.deconfliction;

import static org.junit.Assert.*;

import java.util.List;

import org.jgrapht.GraphPath;
import org.jgrapht.alg.*;
import org.jgrapht.generate.CompleteBipartiteGraphGenerator;
import org.jgrapht.graph.DefaultWeightedEdge;
import org.junit.Test;

import cz.agents.deconfliction.maneuvergraph.CompleteGridWaypointGraph;
import cz.agents.deconfliction.maneuvergraph.NWayConstantSpeedGraph<Waypoint, SpatialManeuver>;
import cz.agents.deconfliction.util.Point;
import cz.agents.alite.trajectorytools.util.Waypoint;
import cz.agents.deconfliction.waypointgraph.DefaultWaypointGraph;

public class KShortestPathsTest {

    @Test
    public void test() {

       DefaultWaypointGraph graph = new CompleteGridWaypointGraph(5, 5, 10, 10);

       Waypoint start = graph.getNearestWaypoint(new Point(0,0,0));
       Waypoint end = graph.getNearestWaypoint(new Point(5.0,5.0,0));

       KShortestPaths<Waypoint, DefaultWeightedEdge> ksp =
               new KShortestPaths<Waypoint, DefaultWeightedEdge>(graph, start, 10, 2);

       List<GraphPath<Waypoint, DefaultWeightedEdge>> paths = ksp.getPaths(end);

       for (GraphPath<Waypoint, DefaultWeightedEdge> path : paths) {
           System.out.println(path);
       }


       //List<GraphPath<Waypoint, DefaultWeightedEdge>> paths = ksp.getPaths(graph.getNearestWaypoint(new Point(5,5,0)));
       //System.out.println(paths.size());

    }

}

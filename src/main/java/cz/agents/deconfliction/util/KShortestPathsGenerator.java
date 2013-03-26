package cz.agents.deconfliction.util;

import java.util.List;

import org.apache.log4j.Logger;
import org.jgrapht.GraphPath;
import org.jgrapht.alg.KShortestPaths;
import org.jgrapht.graph.DefaultWeightedEdge;

import cz.agents.deconfliction.maneuvergraph.Maneuver;
import org.jgrapht.Graph;
import cz.agents.alite.trajectorytools.util.Waypoint;
import cz.agents.deconfliction.waypointgraph.DefaultWaypointGraph;

public class KShortestPathsGenerator {
    static Logger LOGGER = Logger.getLogger(KShortestPathsGenerator.class);
    Graph<Waypoint, SpatialManeuver> graph;
    Waypoint start;
    Waypoint end;

    List<GraphPath<Waypoint, SpatialManeuver>> paths;
    int nextPathIndex;


    public KShortestPathsGenerator(Graph<Waypoint, SpatialManeuver> graph, Waypoint start, Waypoint end, int k) {
        super();
        this.graph = graph;
        this.start = start;
        this.end = end;

        KShortestPaths<Waypoint, SpatialManeuver> ksp = new KShortestPaths<Waypoint, SpatialManeuver>(graph, start, k);

        paths = ksp.getPaths(end);
        nextPathIndex = 0;
    }

    public KShortestPathsGenerator(KShortestPathsGenerator generator) {
        this.graph = generator.graph;
        this.start = generator.start;
        this.end = generator.end;

        paths = generator.paths;
        nextPathIndex = 0;
    }

    public GraphPath<Waypoint, SpatialManeuver> nextGraphPath() {
        if (nextPathIndex < paths.size()) {
            return paths.get(nextPathIndex++);
        }
        else {
            return null;
        }
    }

    public void reset() {
        nextPathIndex = 0;
    }
}

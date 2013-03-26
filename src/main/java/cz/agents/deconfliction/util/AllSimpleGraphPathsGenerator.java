package cz.agents.deconfliction.util;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;
import java.util.Set;
import java.util.Stack;

import org.apache.log4j.Logger;
import org.jgrapht.GraphPath;
import org.jgrapht.graph.DefaultWeightedEdge;
import org.jgrapht.graph.GraphPathImpl;

import cz.agents.deconfliction.maneuvergraph.Maneuver;
import org.jgrapht.Graph;
import cz.agents.deconfliction.trajectory.PiecewiseLinearTrajectory;
import cz.agents.alite.trajectorytools.trajectory.Trajectory;
import cz.agents.deconfliction.trajectory.TrajectoryGenerator;
import cz.agents.alite.trajectorytools.util.Waypoint;
import cz.agents.deconfliction.waypointgraph.DefaultWaypointGraph;

public class AllSimpleGraphPathsGenerator {
    static Logger LOGGER = Logger.getLogger(AllSimpleGraphPathsGenerator.class);
    Graph<Waypoint, SpatialManeuver> graph;
    Waypoint start;
    Waypoint end;

    Stack<Waypoint> stack = new Stack<Waypoint>();

    public AllSimpleGraphPathsGenerator(Graph<Waypoint, SpatialManeuver> graph, Waypoint start, Waypoint end) {
        super();
        this.graph = graph;
        this.start = start;
        this.end = end;
        stack.add(start);
    }

    public static GraphPath<Waypoint, SpatialManeuver> toGraphPath(Graph<Waypoint, SpatialManeuver> graph, List<Waypoint> waypoints) {
        List<SpatialManeuver> edges = new ArrayList<SpatialManeuver>();
        double weight = 0.0;

        if (waypoints.size() < 2)
            return null;

        for (int i = 0; i < waypoints.size()-1; i++) {
            SpatialManeuver edge = graph.getAllEdges(waypoints.get(i), waypoints.get(i+1)).toArray(new Maneuver[1])[0];

            edges.add(edge);
            weight += edge.getDuration();
        }

        return new GraphPathImpl<Waypoint, SpatialManeuver>(graph, waypoints.get(0), waypoints.get(waypoints.size()-1), edges, weight);
    }

    public List<Waypoint> next() {

        while (!stack.isEmpty()) {
            Waypoint current = stack.peek();

            if (current.equals(end)) {
                List<Waypoint> path = new LinkedList<Waypoint>(stack);
                //System.out.println("Path: " + stack);
                backtrack();
                return path;
            } else
            if (graph.degreeOf(current) > 0) {
                List<Waypoint> children = graph.getOrderedNeighbors(current);
                boolean childAdded = false;

                for (int i=0; i < children.size() && !childAdded; i++) {
                    if (!stack.contains(children.get(i))) {
                        stack.push(children.get(i));
                        childAdded = true;
                    }
                }

                if (!childAdded) {
                    // All children are already occurring in the path
                    backtrack();
                }
            } else {
                // Degree of the node is 0
                backtrack();
            }
        }

        return null;
    }

    public GraphPath<Waypoint, SpatialManeuver> nextGraphPath() {
        List<Waypoint> waypoints = next();
        if (waypoints != null) {
            return toGraphPath(graph, waypoints);
        }
        else {
            return null;
        }
    }

    private void backtrack() {
        if (stack.size() == 1) {
            // cannot backtrack further
            stack.pop();
        } else
        if (stack.size() >= 2) {
            // take next child
            Waypoint child = stack.pop();
            Waypoint parent = stack.peek();

            List<Waypoint> neighbors = graph.getOrderedNeighbors(parent);

            int currentChildIndex = neighbors.lastIndexOf(child);

            boolean newChildFound = false;
            for (int i=currentChildIndex+1; i < neighbors.size() && !newChildFound; i++) {
                if (!stack.contains(neighbors.get(i))) {
                    stack.push(neighbors.get(i));
                    newChildFound = true;
                }
            }

            if (!newChildFound) {
                // the child was the last one or all the following children have occurred in the path.
                // we must backtrack one level up
                backtrack();
            }
        }
    }


    public void reset() {
        stack.clear();
        stack.add(start);
    }
}

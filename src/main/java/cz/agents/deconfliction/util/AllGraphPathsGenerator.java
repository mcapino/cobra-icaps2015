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

import cz.agents.deconfliction.trajectory.PiecewiseLinearTrajectory;
import cz.agents.alite.trajectorytools.trajectory.Trajectory;
import cz.agents.deconfliction.trajectory.TrajectoryGenerator;
import cz.agents.alite.trajectorytools.util.Waypoint;
import cz.agents.deconfliction.waypointgraph.DefaultWaypointGraph;

public class AllGraphPathsGenerator {
    static Logger LOGGER = Logger.getLogger(AllGraphPathsGenerator.class);
    DefaultWaypointGraph graph;
    Waypoint start;
    Waypoint end;
    double maxWeight;

    Stack<Waypoint> stack = new Stack<Waypoint>();
    double weight = 0.0;

    public AllGraphPathsGenerator(DefaultWaypointGraph graph, Waypoint start, Waypoint end, double maxWeight) {
        super();
        this.graph = graph;
        this.start = start;
        this.end = end;
        this.maxWeight = maxWeight;
        stack.add(start);
    }

    public static GraphPath<Waypoint, DefaultWeightedEdge> toGraphPath(DefaultWaypointGraph graph, List<Waypoint> waypoints) {
        List<DefaultWeightedEdge> edges = new ArrayList<DefaultWeightedEdge>();
        double weight = 0.0;

        if (waypoints.size() < 2)
            return null;

        for (int i = 0; i < waypoints.size()-1; i++) {
            DefaultWeightedEdge edge = graph.getAllEdges(waypoints.get(i), waypoints.get(i+1)).toArray(new DefaultWeightedEdge[1])[0];

            edges.add(edge);
            weight += graph.getEdgeWeight(edge);
        }

        return new GraphPathImpl<Waypoint, DefaultWeightedEdge>(graph, waypoints.get(0), waypoints.get(waypoints.size()-1), edges, weight);
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
                    double childEdgeWeight = graph.getEdgeWeight(graph.getEdge(current, children.get(i)));
                    if (weight + childEdgeWeight <= maxWeight) {
                         stack.push(children.get(i));
                         weight += childEdgeWeight;
                         childAdded = true;
                    }
                }

                if (!childAdded) {
                    // All children exceed maximum weight
                    backtrack();
                }
            } else {
                // Degree of the node is 0
                backtrack();
            }
        }

        return null;
    }

    public GraphPath<Waypoint, DefaultWeightedEdge> nextGraphPath() {
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

            double currentChildEdgeWeight = graph.getEdgeWeight(graph.getEdge(parent, child));
            weight -= currentChildEdgeWeight;

            boolean newChildFound = false;
            for (int i=currentChildIndex+1; i < neighbors.size() && !newChildFound; i++) {
                double childEdgeWeight = graph.getEdgeWeight(graph.getEdge(parent, neighbors.get(i)));
                if (weight + childEdgeWeight <= maxWeight) {
                     stack.push(neighbors.get(i));
                     weight += childEdgeWeight;
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

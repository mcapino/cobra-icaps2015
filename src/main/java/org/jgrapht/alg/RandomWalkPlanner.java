package org.jgrapht.alg;

import java.util.LinkedList;
import java.util.List;
import java.util.Random;
import java.util.Set;

import org.jgrapht.Graph;
import org.jgrapht.GraphPath;
import org.jgrapht.alg.specifics.Specifics;
import org.jgrapht.alg.specifics.SpecificsFactory;
import org.jgrapht.graph.GraphPathImpl;
import org.jgrapht.util.Goal;
import org.jgrapht.util.HeuristicToGoal;


public class RandomWalkPlanner<V, E>{

    public static <V, E> GraphPath<V, E> findPathBetween(Graph<V, E> graph,
            HeuristicToGoal<V> heuristic, V startVertex, Goal<V> goal, Random random, double randomMoveRatio) {

        List<E> edgeList = new LinkedList<E>();
        double weight = 0;
        Specifics<V, E> specifics = SpecificsFactory.create(graph);

        V current = startVertex;
        while (!goal.isGoal(current)) {
            Set<E> outEdges = specifics.outgoingEdgesOf(current);

            // findMin
            E nextEdge = null;
            if (random.nextDouble() < randomMoveRatio) {
                // take random move
                @SuppressWarnings("unchecked")
                E[] outEdgesArray = (E[]) outEdges.toArray();
                nextEdge = outEdgesArray[random.nextInt(outEdgesArray.length)];
            } else {
                double bestCostToGo = Double.POSITIVE_INFINITY;
                for (E edge : outEdges) {
                    double costToGo = heuristic.getCostToGoalEstimate(graph.getEdgeTarget(edge));
                    if (costToGo <= bestCostToGo) {
                        nextEdge = edge;
                        bestCostToGo = costToGo;
                    }
                }
            }

            assert nextEdge != null : "The graph is assumed not to have dead-ends";
            edgeList.add(nextEdge);
            weight += graph.getEdgeWeight(nextEdge);
            current = graph.getEdgeTarget(nextEdge);
        }

        return new GraphPathImpl<V, E>(graph, startVertex, current, edgeList, weight);
    }
}

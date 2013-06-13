package cz.agents.admap.agent;

import java.util.Collection;

import org.jgrapht.DirectedGraph;
import org.jgrapht.GraphPath;
import org.jgrapht.alg.AStarShortestPath;
import org.jgrapht.util.Goal;
import org.jgrapht.util.Heuristic;

import tt.euclid2i.EvaluatedTrajectory;
import tt.euclid2i.Point;
import tt.euclid2i.Region;
import tt.euclid2i.discretization.LazyGrid;
import tt.euclid2i.discretization.ToGoalEdgeExtension;
import tt.euclid2i.region.Rectangle;
import tt.euclid2i.trajectory.StraightSegmentTrajectory;
import tt.euclidtime3i.discretization.ConstantSpeedTimeExtension;
import tt.euclidtime3i.discretization.Straight;

public class Util {

    private static final int GRID_STEP = 25;
    private static final int MAX_TIME = 100000;

    static public EvaluatedTrajectory computeBestResponse(final Point start, final Point goal,
            Collection<Region> obstacles, Rectangle bounds, Collection<tt.euclidtime3i.Region> avoid) {

        // create grid discretization
        final DirectedGraph<tt.euclid2i.Point, tt.euclid2i.Line> grid
            = new LazyGrid(start,
                    obstacles,
                    bounds,
                    LazyGrid.PATTERN_8_WAY,
                    GRID_STEP);

        final DirectedGraph<tt.euclid2i.Point, tt.euclid2i.Line> spatialGraph
            = new ToGoalEdgeExtension(grid, goal, GRID_STEP);

        // visualize the graph
//        VisManager.registerLayer(GraphLayer.create(new GraphProvider<tt.euclid2i.Point, tt.euclid2i.Line>() {
//            @Override
//            public Graph<tt.euclid2i.Point, tt.euclid2i.Line> getGraph() {
//                return ((ToGoalEdgeExtension) spatialGraph).generateFullGraph(start);
//            }
//        }, new tt.euclid2i.vis.ProjectionTo2d(), Color.GRAY, Color.GRAY, 1, 4));

        // time-extension
        DirectedGraph<tt.euclidtime3i.Point, Straight> graph
            = new ConstantSpeedTimeExtension(spatialGraph, MAX_TIME, new int[] {1}, avoid);


        // plan
        final GraphPath<tt.euclidtime3i.Point, Straight> path = AStarShortestPath
                .findPathBetween(graph,
                new Heuristic<tt.euclidtime3i.Point>() {
                    @Override
                    public double getCostToGoalEstimate(tt.euclidtime3i.Point current) {
                        return (current.getPosition()).distance(goal);
                    }
                },
                new tt.euclidtime3i.Point(start.x, start.y, 0),
                new Goal<tt.euclidtime3i.Point>() {
                    @Override
                    public boolean isGoal(tt.euclidtime3i.Point current) {
                        return current.getPosition().equals(goal);
                    }
                });

        EvaluatedTrajectory trajectory
            = new StraightSegmentTrajectory<tt.euclidtime3i.Point, Straight>(path, MAX_TIME);

        return trajectory;
    }

}

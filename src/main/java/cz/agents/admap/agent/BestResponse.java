package cz.agents.admap.agent;

import java.util.Collection;
import java.util.Random;

import org.jgrapht.DirectedGraph;
import org.jgrapht.GraphPath;
import org.jgrapht.alg.AStarShortestPath;
import org.jgrapht.alg.AStarShortestPathSimple;
import org.jgrapht.alg.RandomWalkPlanner;
import org.jgrapht.util.Goal;
import org.jgrapht.util.HeuristicToGoal;

import tt.euclid2i.EvaluatedTrajectory;
import tt.euclid2i.Point;
import tt.euclid2i.Region;
import tt.euclid2i.SegmentedTrajectory;
import tt.euclid2i.discretization.L1Heuristic;
import tt.euclid2i.discretization.L2Heuristic;
import tt.euclid2i.discretization.LazyGrid;
import tt.euclid2i.discretization.ObstacleWrapper;
import tt.euclid2i.discretization.ToGoalEdgeExtension;
import tt.euclid2i.region.Rectangle;
import tt.euclid2i.trajectory.StraightSegmentTrajectory;
import tt.euclidtime3i.discretization.ConstantSpeedTimeExtension;
import tt.euclidtime3i.discretization.FreeOnTargetWaitExtension;
import tt.euclidtime3i.discretization.Straight;
import tt.euclidtime3i.region.MovingCircle;
import tt.euclidtime3i.sipp.SippEdge;
import tt.euclidtime3i.sipp.SippGoal;
import tt.euclidtime3i.sipp.SippHeuristic;
import tt.euclidtime3i.sipp.SippNode;
import tt.euclidtime3i.sipp.SippUtils;
import tt.euclidtime3i.sipp.SippWrapper;
import tt.euclidtime3i.sipp.intervals.Interval;
import tt.euclidtime3i.sipprrts.DynamicObstaclesImpl;

public class BestResponse {

    private static final int GRID_STEP = 25;
    private static final int MAX_TIME = 2000;

    static public EvaluatedTrajectory computeBestResponse(final Point start, final Point goal,
            Collection<Region> obstacles, Rectangle bounds,
            Collection<tt.euclid2i.Region> staticObstacles,
            Collection<tt.euclidtime3i.Region> dynamicObstacles) {

        // create grid discretization
        final DirectedGraph<tt.euclid2i.Point, tt.euclid2i.Line> grid
            = new LazyGrid(start,
                    obstacles,
                    bounds,
                    LazyGrid.PATTERN_8_WAY,
                    GRID_STEP);

        return computeBestResponse(start, goal, grid, staticObstacles, dynamicObstacles);
    }

	public static EvaluatedTrajectory computeBestResponse(final Point start,
			final Point goal,
			final DirectedGraph<tt.euclid2i.Point, tt.euclid2i.Line> spatialGraph,
			Collection<tt.euclid2i.Region> staticObstacles,
			Collection<tt.euclidtime3i.Region> dynamicObstacles) {

		final DirectedGraph<tt.euclid2i.Point, tt.euclid2i.Line> adaptedSpatialGraph
			= new ObstacleWrapper<tt.euclid2i.Point, tt.euclid2i.Line>(spatialGraph, staticObstacles);

		// = new ToGoalEdgeExtension(graph, goal, GRID_STEP);

      //visualize the graph
//		VisManager.registerLayer(GraphLayer.create(
//				new GraphProvider<tt.euclid2i.Point, tt.euclid2i.Line>() {
//					@Override
//					public Graph<tt.euclid2i.Point, tt.euclid2i.Line> getGraph() {
//						return ((ToGoalEdgeExtension) spatialGraph)
//								.generateFullGraph(start);
//					}
//				}, new tt.euclid2i.vis.ProjectionTo2d(), Color.GRAY,
//				Color.GRAY, 1, 4));

        // time-extension
        DirectedGraph<tt.euclidtime3i.Point, Straight> graph
            = new ConstantSpeedTimeExtension(adaptedSpatialGraph, MAX_TIME, new int[] {1}, dynamicObstacles, GRID_STEP);

        DirectedGraph<tt.euclidtime3i.Point, Straight> graphFreeOnTarget
            = new FreeOnTargetWaitExtension(graph, goal);

        // plan
        final GraphPath<tt.euclidtime3i.Point, Straight> path = AStarShortestPath.findPathBetween(graphFreeOnTarget,
                new HeuristicToGoal<tt.euclidtime3i.Point>() {
                    @Override
                    public double getCostToGoalEstimate(tt.euclidtime3i.Point current) {
                        return (current.getPosition()).distance(goal);
                    }
                },
                new tt.euclidtime3i.Point(start.x, start.y, 0),
                new Goal<tt.euclidtime3i.Point>() {
                    @Override
                    public boolean isGoal(tt.euclidtime3i.Point current) {
                        return current.getPosition().equals(goal) &&
                                current.getTime() > (MAX_TIME - GRID_STEP - 1); // last space-time node might not be placed at MAX_TIME
                    }
                });
        if (path != null) {
            return new StraightSegmentTrajectory(path, MAX_TIME);
        } else {
            return null;
        }
	}

	public static EvaluatedTrajectory computeBestResponseSIPP(final Point start,
			final Point goal,
			final DirectedGraph<tt.euclid2i.Point, tt.euclid2i.Line> spatialGraph,
			Collection<tt.euclidtime3i.Region> avoid) {

        final SegmentedTrajectory[] trajArr = new SegmentedTrajectory[avoid.size()];
        int[] radiuses = new int[trajArr.length];

        int i = 0;
        for (tt.euclidtime3i.Region region: avoid) {
        	assert region instanceof MovingCircle;
        	MovingCircle mc = (MovingCircle) region;
        	assert mc.getTrajectory() instanceof MovingCircle;
        	trajArr[i] = (SegmentedTrajectory) mc.getTrajectory();
        	radiuses[i] = mc.getRadius();
        	i++;
        }

        DynamicObstaclesImpl dynamicEnv = new DynamicObstaclesImpl(trajArr, radiuses, MAX_TIME);

        System.out.println("Creating SIPP Wrapper...");

        SippWrapper wrapper = new SippWrapper(spatialGraph, dynamicEnv, 0, 1, 2, MAX_TIME);
        SippNode startSipp = new SippNode(start, Interval.toInfinity(0), 0);
        SippHeuristic heuristic = new SippHeuristic(new L2Heuristic(goal), 1);
        SippGoal goalSipp = new SippGoal(goal, MAX_TIME);

        System.out.println("..Done \nStarting A* search...");

        GraphPath<SippNode, SippEdge> path = AStarShortestPathSimple.findPathBetween(wrapper, heuristic, startSipp, goalSipp);

        if (path != null) {
        	final SegmentedTrajectory trajectory = SippUtils.parseTrajectory(path, MAX_TIME);
        	return trajectory;
        } else {
        	return null;
        }
	}


    static public EvaluatedTrajectory computeRandomRoute(final Point start, final Point goal,
            Collection<Region> obstacles, Rectangle bounds, Collection<tt.euclidtime3i.Region> avoid, Random random) {

        // create grid discretization
        final DirectedGraph<tt.euclid2i.Point, tt.euclid2i.Line> grid
            = new LazyGrid(start,
                    obstacles,
                    bounds,
                    LazyGrid.PATTERN_8_WAY,
                    GRID_STEP);

        final DirectedGraph<tt.euclid2i.Point, tt.euclid2i.Line> spatialGraph
            = new ToGoalEdgeExtension(grid, goal, GRID_STEP);

        // time-extension
        DirectedGraph<tt.euclidtime3i.Point, Straight> graph
            = new ConstantSpeedTimeExtension(spatialGraph, Integer.MAX_VALUE, new int[] {1}, avoid, ConstantSpeedTimeExtension.DISABLE_WAIT_MOVE);

        // plan
        final GraphPath<tt.euclidtime3i.Point, Straight> path = RandomWalkPlanner
                .findPathBetween(graph,
                new HeuristicToGoal<tt.euclidtime3i.Point>() {
                    @Override
                    public double getCostToGoalEstimate(tt.euclidtime3i.Point current) {
                        return (current.getPosition()).distance(goal);
                    }
                },
                new tt.euclidtime3i.Point(start.x, start.y, 0),
                new Goal<tt.euclidtime3i.Point>() {
                    @Override
                    public boolean isGoal(tt.euclidtime3i.Point current) {
                        return current.getPosition().equals(goal); // last space-time node might not be placed at MAX_TIME
                    }
                }, random, 0.2);

        if (path != null) {
            return new StraightSegmentTrajectory(path, MAX_TIME);
        } else {
            return null;
        }
    }

}

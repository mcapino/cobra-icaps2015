package cz.agents.map4rt.agent;

import java.awt.Color;
import java.util.Collection;

import org.jgrapht.DirectedGraph;
import org.jgrapht.Graph;
import org.jgrapht.GraphPath;
import org.jgrapht.alg.AStarShortestPathSimple;
import org.jgrapht.util.Goal;
import org.jgrapht.util.HeuristicToGoal;

import tt.euclid2i.EvaluatedTrajectory;
import tt.euclid2i.Line;
import tt.euclid2i.Point;
import tt.euclid2i.Region;
import tt.euclid2i.discretization.ObstacleWrapper;
import tt.euclid2i.trajectory.LineSegmentsConstantSpeedTrajectory;
import tt.euclid2i.trajectory.StraightSegmentTrajectory;
import tt.euclid2i.vis.RegionsLayer;
import tt.euclidtime3i.discretization.ConstantSpeedTimeExtension;
import tt.euclidtime3i.discretization.ControlEffortWrapper;
import tt.euclidtime3i.discretization.FreeOnTargetWaitExtension;
import tt.euclidtime3i.discretization.Straight;
import tt.euclidtime3i.vis.TimeParameterProjectionTo2d;
import tt.vis.GraphLayer;
import tt.vis.TimeParameterHolder;
import cz.agents.alite.vis.VisManager;
import cz.agents.alite.vis.layer.VisLayer;

public class BestResponse {

    private static final boolean DEBUG_VIS = false;

	public static EvaluatedTrajectory computeBestResponse(
			final tt.euclidtime3i.Point start,
			final Point goal,
			final float maxSpeed,
			final DirectedGraph<tt.euclid2i.Point, tt.euclid2i.Line> spatialGraph,
			final HeuristicToGoal<tt.euclidtime3i.Point> heuristic,
			final Collection<tt.euclid2i.Region> staticObstacles,
			final Collection<tt.euclidtime3i.Region> dynamicObstacles, final int maxTime, final int timeStep) {

		final ObstacleWrapper<tt.euclid2i.Point, tt.euclid2i.Line> adaptedSpatialGraph
			= new ObstacleWrapper<tt.euclid2i.Point, tt.euclid2i.Line>(spatialGraph, staticObstacles);
		
		VisLayer graphLayer;VisLayer sobstLayer;VisLayer dobstLayer;
		
		if (DEBUG_VIS) {
			 // --- debug visio --- begin
		     //visualize the graph
			 graphLayer = GraphLayer.create(
						new GraphLayer.GraphProvider<tt.euclid2i.Point, tt.euclid2i.Line>() {
							@Override
							public Graph<tt.euclid2i.Point, tt.euclid2i.Line> getGraph() {
								return ((ObstacleWrapper<Point, Line>) adaptedSpatialGraph).generateFullGraph(start.getPosition());
							}
						}, new tt.euclid2i.vis.ProjectionTo2d(), Color.BLUE,
						Color.BLUE, 1, 4);
			 
			 sobstLayer = RegionsLayer.create(new tt.euclid2i.vis.RegionsLayer.RegionsProvider() {
				@Override
				public Collection<? extends Region> getRegions() {
					return staticObstacles;
				}
			 }, Color.ORANGE);
			 
			 dobstLayer = tt.euclidtime3i.vis.RegionsLayer.create(new tt.euclidtime3i.vis.RegionsLayer.RegionsProvider() {
				
				@Override
				public Collection<tt.euclidtime3i.Region> getRegions() {
					return dynamicObstacles;
				}
			}, new TimeParameterProjectionTo2d(TimeParameterHolder.time), Color.BLACK, null);
			 
		      
		    VisManager.registerLayer(graphLayer);
		    VisManager.registerLayer(sobstLayer);
		    VisManager.registerLayer(dobstLayer);
		    
			// --- debug visio --- end
		}
	    
        // time-extension
        DirectedGraph<tt.euclidtime3i.Point, Straight> motions 
        	= createMotions(adaptedSpatialGraph, goal, maxSpeed, dynamicObstacles, maxTime, timeStep);

        // plan
        final GraphPath<tt.euclidtime3i.Point, Straight> path = AStarShortestPathSimple.findPathBetween(motions,
                heuristic,
                start,
                new Goal<tt.euclidtime3i.Point>() {
                    @Override
                    public boolean isGoal(tt.euclidtime3i.Point current) {
                        return current.getPosition().equals(goal)
                        		&& current.getTime() > (maxTime - timeStep - 1); // last space-time node might not be placed at MAX_TIME
                    }
                });

        if (path != null) {
        	if (DEBUG_VIS) {
	        	VisManager.unregisterLayer(graphLayer);
	        	VisManager.unregisterLayer(sobstLayer);
	        	VisManager.unregisterLayer(dobstLayer);
        	}
            return new StraightSegmentTrajectory(path, maxTime);
        } else {
            return null;
        }
	}


    public static EvaluatedTrajectory computeShortestPath(
			final Point start,
			final Point goal,
			final DirectedGraph<tt.euclid2i.Point, tt.euclid2i.Line> spatialGraph,
			HeuristicToGoal<Point> heuristic,
			Collection<tt.euclid2i.Region> staticObstacles) {

		final ObstacleWrapper<tt.euclid2i.Point, tt.euclid2i.Line> adaptedSpatialGraph
			= new ObstacleWrapper<tt.euclid2i.Point, tt.euclid2i.Line>(spatialGraph, staticObstacles);

        GraphPath<Point, Line> path = AStarShortestPathSimple.findPathBetween(adaptedSpatialGraph,
                heuristic,
                start,
                goal);
        
        return new LineSegmentsConstantSpeedTrajectory<Point, Line>(0, path, 1, (int) path.getWeight());
	}
	
	public static DirectedGraph<tt.euclidtime3i.Point,Straight> createMotions(DirectedGraph<Point, Line> spatialGraph, Point goal, float maxSpeed, Collection<? extends tt.euclidtime3i.Region> dynamicObstacles, int maxTime, int timeStep) {
        DirectedGraph<tt.euclidtime3i.Point, Straight> motions
            = new ConstantSpeedTimeExtension(spatialGraph, maxTime, new float[] {maxSpeed}, dynamicObstacles, timeStep, timeStep);
        motions = new FreeOnTargetWaitExtension(motions, goal);
        motions = new ControlEffortWrapper(motions, 0.01);
        return motions;
	}
}

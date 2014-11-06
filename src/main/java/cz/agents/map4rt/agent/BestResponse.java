package cz.agents.map4rt.agent;

import java.awt.Color;
import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedList;
import java.util.List;

import org.apache.log4j.Logger;
import org.jgrapht.DirectedGraph;
import org.jgrapht.Graph;
import org.jgrapht.GraphPath;
import org.jgrapht.alg.AStarShortestPathSimple;
import org.jgrapht.graph.GraphPathImpl;
import org.jgrapht.util.HeuristicToGoal;
import org.jgrapht.util.Goal;
import org.omg.PortableInterceptor.LOCATION_FORWARD;

import tt.euclid2i.EvaluatedTrajectory;
import tt.euclid2i.Line;
import tt.euclid2i.Point;
import tt.euclid2i.Region;
import tt.euclid2i.Trajectory;
import tt.euclid2i.discretization.ObstacleWrapper;
import tt.euclid2i.trajectory.DelayedStartTrajectory;
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
	
	
	private final static Logger LOGGER = Logger.getLogger(BestResponse.class);
    private static final boolean DEBUG_VIS = false;

	public static EvaluatedTrajectory computeBestResponse(
			final Point start,
			final int minTime,
			final int depTime,
			final Point goal,
			final float maxSpeed,
			final DirectedGraph<tt.euclid2i.Point, tt.euclid2i.Line> spatialGraph,
			final HeuristicToGoal<tt.euclidtime3i.Point> heuristic,
			final Collection<tt.euclidtime3i.Region> dynamicObstacles, 
			final int maxTime, 
			final int timeStep, 
			final int runtimeLimitMs) {

		VisLayer graphLayer;VisLayer sobstLayer;VisLayer dobstLayer;
		
		if (DEBUG_VIS) {
			 // --- debug visio --- begin
		     //visualize the graph
			 graphLayer = GraphLayer.create(
						new GraphLayer.GraphProvider<tt.euclid2i.Point, tt.euclid2i.Line>() {
							@Override
							public Graph<tt.euclid2i.Point, tt.euclid2i.Line> getGraph() {
								return ((ObstacleWrapper<Point, Line>) spatialGraph).generateFullGraph(start);
							}
						}, new tt.euclid2i.vis.ProjectionTo2d(), Color.BLUE,
						Color.BLUE, 1, 4);
			 
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
        	= createMotions(spatialGraph, goal, maxSpeed, dynamicObstacles, maxTime, timeStep);
        
        Goal<tt.euclidtime3i.Point> goalCond = new Goal<tt.euclidtime3i.Point>() {
            @Override
            public boolean isGoal(tt.euclidtime3i.Point current) {
                return current.getPosition().equals(goal) && current.getTime() > (maxTime - timeStep - 1); // last space-time node might not be placed at MAX_TIME
            }
        };
        AStarShortestPathSimple<tt.euclidtime3i.Point, Straight> astar 
        	= new AStarShortestPathSimple<>(motions, heuristic, new tt.euclidtime3i.Point(start, depTime), goalCond);
        	
        long startedAt = System.currentTimeMillis();	
        GraphPath<tt.euclidtime3i.Point, Straight> path = astar.findPathRuntimeLimit(Integer.MAX_VALUE, runtimeLimitMs);
        
        long runtime = System.currentTimeMillis()-startedAt;
		LOGGER.debug("Planning finshed in " + runtime + "ms; "
				+ astar.getIterationCount() + " iterations; "
				+ (int) (astar.getIterationCount() / (runtime / 1000.0)) + " it/sec "
				+ " path-length:" + ((path != null) ? path.getEdgeList().size() : "NONE"));
        
        if (path != null) {
        	if (DEBUG_VIS) {
	        	VisManager.unregisterLayer(graphLayer);
	        	VisManager.unregisterLayer(sobstLayer);
	        	VisManager.unregisterLayer(dobstLayer);
        	}
        	
        	// add initial 'wait at start' segments
        	List<Straight> edgeList = new ArrayList<Straight>();       	
        	assert (depTime-minTime) % timeStep == 0 : depTime + " - " + minTime + " is not divisible by " + timeStep;
        	for (int t=minTime; t < depTime; t += timeStep) {
        		edgeList.add(new Straight(new tt.euclidtime3i.Point(start, t), new tt.euclidtime3i.Point(start, t+timeStep)));
        	}
        	assert edgeList.get(edgeList.size()-1).getEnd().getTime() == depTime;
        	edgeList.addAll(path.getEdgeList());
        	
        	GraphPathImpl<tt.euclidtime3i.Point, Straight> path2 
        		= new GraphPathImpl<tt.euclidtime3i.Point, Straight>(path.getGraph(), path.getStartVertex(), path.getEndVertex(), edgeList, path.getWeight());
            // this should be constant step
        	return new StraightSegmentTrajectory(path2, maxTime-minTime);
        } else {
            return null;
        }
	}
	
	public static EvaluatedTrajectory computeBestResponseByDelayMethod(
			final tt.euclidtime3i.Point start,
			final Point goal,
			final float maxSpeed,
			final DirectedGraph<tt.euclid2i.Point, tt.euclid2i.Line> spatialGraph,
			final HeuristicToGoal<tt.euclid2i.Point> heuristic,
			final Collection<tt.euclidtime3i.Region> dynamicObstacles, final int maxTime, final int timeStep, final int runtimeLimitMs) {
		
		long startedAt = System.currentTimeMillis();
		EvaluatedTrajectory path = computeShortestPath(start.getPosition(), start.getTime(), goal, maxSpeed, maxTime, spatialGraph, heuristic);
		
		int delay = 0;
		while (true) {
			DelayedStartTrajectory delayedTraj = new DelayedStartTrajectory(path, delay);
			if (collisionFree(delayedTraj, dynamicObstacles, 200)) {
				return delayedTraj;
			}
			
			if (System.currentTimeMillis() > startedAt + runtimeLimitMs) {
				return null;
			}
			
			delay += timeStep;
		}
	}
	
	public static boolean collisionFree(Trajectory traj, Collection<tt.euclidtime3i.Region> dynamicObstacles, int samplingInterval) {
		for (int t=traj.getMinTime(); t<traj.getMaxTime(); t += samplingInterval) {
			tt.euclidtime3i.Point pos = new tt.euclidtime3i.Point(traj.get(t), t);
			for (tt.euclidtime3i.Region dobst : dynamicObstacles) {
				if (dobst.isInside(pos)) {
					return false;
				}
			}
		}
		return true;
	}

    public static EvaluatedTrajectory computeShortestPath(
			final Point start,
			final int departureTime,
			final Point goal,
			final float maxSpeed,
			final int maxTime,
			final DirectedGraph<tt.euclid2i.Point, tt.euclid2i.Line> spatialGraph,
			HeuristicToGoal<Point> heuristic) {

        GraphPath<Point, Line> path = AStarShortestPathSimple.findPathBetween(spatialGraph,
                heuristic,
                start,
                goal);
        
        return new LineSegmentsConstantSpeedTrajectory<Point, Line>(departureTime, path, maxSpeed, maxTime-departureTime);
	}
	
	public static DirectedGraph<tt.euclidtime3i.Point,Straight> createMotions(DirectedGraph<Point, Line> spatialGraph, Point goal, float maxSpeed, Collection<? extends tt.euclidtime3i.Region> dynamicObstacles, int maxTime, int timeStep) {
        DirectedGraph<tt.euclidtime3i.Point, Straight> motions
            = new ConstantSpeedTimeExtension(spatialGraph, maxTime, new float[] {maxSpeed}, dynamicObstacles, timeStep, timeStep);
        motions = new FreeOnTargetWaitExtension(motions, goal);
        motions = new ControlEffortWrapper(motions, 0.01);
        return motions;
	}
}

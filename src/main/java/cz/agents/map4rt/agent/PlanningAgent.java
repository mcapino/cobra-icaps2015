package cz.agents.map4rt.agent;

import java.util.Collection;
import java.util.LinkedList;
import java.util.List;

import org.apache.log4j.Logger;
import org.jgrapht.DirectedGraph;
import org.jgrapht.util.HeuristicToGoal;
import org.jgrapht.util.heuristics.PerfectHeuristic;

import tt.euclid2i.EvaluatedTrajectory;
import tt.euclid2i.Line;
import tt.euclid2i.Point;
import tt.euclid2i.SegmentedTrajectory;
import tt.euclid2i.probleminstance.Environment;
import tt.euclid2i.region.Circle;
import tt.euclid2i.trajectory.BasicSegmentedTrajectory;
import tt.euclidtime3i.Region;
import tt.euclidtime3i.ShortestPathHeuristic;
import tt.euclidtime3i.region.MovingCircle;
import tt.euclidtime3i.region.MovingCircleMinusPoint;
import tt.jointeuclid2ni.probleminstance.RelocationTask;

/**
 * An agent that resolves conflict by finding new conflict-free plans.
 */
public abstract class PlanningAgent extends Agent {
	
	final int T_PLANNING = 3000;
	
	Logger LOGGER = Logger.getLogger(PlanningAgent.class);
	
	EvaluatedTrajectory trajectory;
	
	protected final int timeStep;
	protected final int maxTime;
	protected EvaluatedTrajectory currentTrajectory;
	protected Point currentPos;
	
	public PlanningAgent(String name, Point start, List<RelocationTask> tasks,
			Environment environment, DirectedGraph<Point, Line> planningGraph,
			int agentBodyRadius, float maxSpeed, int maxTime, int timeStep) {
		super(name, start, tasks, environment, planningGraph, agentBodyRadius, maxSpeed);
		this.maxTime = maxTime;
		this.timeStep = timeStep;
		this.currentPos = start;
	}

	protected EvaluatedTrajectory getBestResponseTrajectory(
			Point start,
			int minTime, 
			int depTime,
			Point goal,
			Collection<Region> dynamicObst, 
			int maxTime) {
		
		LOGGER.debug(getName() + " started planning " + start + "@"  + minTime + "/" + depTime + " -> " + goal + " maxtime=" + maxTime);
		long startedAt = System.currentTimeMillis();

		EvaluatedTrajectory traj;
		LinkedList<Region> dObstInflated = inflateDynamicObstacles(dynamicObst, agentBodyRadius);


		final HeuristicToGoal<tt.euclid2i.Point> spatialHeuristic = new PerfectHeuristic<tt.euclid2i.Point, Line>(planningGraph, goal);
		final HeuristicToGoal<tt.euclidtime3i.Point> spaceTimeHeuristic = new HeuristicToGoal<tt.euclidtime3i.Point>() {
			@Override
			public double getCostToGoalEstimate(tt.euclidtime3i.Point current) {
				return spatialHeuristic.getCostToGoalEstimate(current.getPosition()) / (double)maxSpeed;
			}
		};
		
//		System.out.println("dobst:");
//		for (Region dobst : dObstInflated) {
//			BasicSegmentedTrajectory mctraj = (BasicSegmentedTrajectory)((MovingCircle) dobst).getTrajectory();
//			System.out.println(mctraj.getSegments().get(0) + " --> " + mctraj.getSegments().get(mctraj.getSegments().size()-1));
//		}
			
		traj = BestResponse.computeBestResponse(start, minTime, depTime, goal, maxSpeed, getPlanningGraph(), spaceTimeHeuristic, dObstInflated, maxTime, timeStep, T_PLANNING);
		
		if (traj == null) {
			LOGGER.error(" >>>>>>>>>>>>>>> !!!!! No trajectory found within the runtime limit of " + T_PLANNING + " ms !!!! <<<<<<<<<<<<<<<<<<<<<");
			System.out.println("dobst:");
			for (Region dobst : dObstInflated) {
				BasicSegmentedTrajectory mctraj = (BasicSegmentedTrajectory)((MovingCircle) dobst).getTrajectory();
				System.out.println(mctraj.getSegments().get(0) + " --> " + mctraj.getSegments().get(mctraj.getSegments().size()-1));
			}
			
			throw new RuntimeException("Failed to find a trajectory");
			//traj = BestResponse.computeBestResponseFallback(start, goal, maxSpeed, getPlanningGraph(), heuristic, dObstInflated, maxTime, timeStep);
		}
		
		LOGGER.debug(getName() + " finished planning in " + (System.currentTimeMillis() - startedAt) + "ms");
		return traj;
	}
	
	protected static LinkedList<tt.euclid2i.Region> inflateStaticObstacles(Collection<tt.euclid2i.Region> sObst, int radius) {
		// Inflate static obstacles
		LinkedList<tt.euclid2i.Region> sObstInflated = new LinkedList<tt.euclid2i.Region>();
		for (tt.euclid2i.Region region : sObst) {
			assert region instanceof Circle;
			Circle circle = (Circle) region;
			sObstInflated.add(new Circle(circle.getCenter(), circle.getRadius() + radius));
		}
		return sObstInflated;
	}

	protected static LinkedList<tt.euclidtime3i.Region> inflateDynamicObstacles(Collection<tt.euclidtime3i.Region> dObst, int radius) {
		// Inflate static obstacles
		LinkedList<tt.euclidtime3i.Region> dObstInflated = new LinkedList<tt.euclidtime3i.Region>();
		for (tt.euclidtime3i.Region region : dObst) {
			assert region instanceof MovingCircle;
			MovingCircle mc = (MovingCircle) region;
			dObstInflated.add(new MovingCircle(mc.getTrajectory(), mc.getRadius() + radius, mc.getSamplingInterval() * 2));
		}
		return dObstInflated;
	}
	
	@Override
	public EvaluatedTrajectory getCurrentTrajectory() {
		return currentTrajectory;
	}

	@Override
	public Point getCurrentPos() {
		if (currentTrajectory != null && currentTrajectory.get(time) != null) {
			currentPos = currentTrajectory.get(time);
		} 
		return currentPos;
	}

	
	
	
}

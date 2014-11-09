package cz.agents.map4rt.agent;

import java.util.Collection;
import java.util.LinkedList;
import java.util.List;

import org.apache.log4j.Logger;
import org.jgrapht.DirectedGraph;
import org.jgrapht.util.HeuristicToGoal;
import org.jgrapht.util.heuristics.PerfectHeuristic;

import cz.agents.map4rt.CommonTime;
import tt.euclid2i.EvaluatedTrajectory;
import tt.euclid2i.Line;
import tt.euclid2i.Point;
import tt.euclid2i.SegmentedTrajectory;
import tt.euclid2i.probleminstance.Environment;
import tt.euclid2i.region.Circle;
import tt.euclid2i.trajectory.BasicSegmentedTrajectory;
import tt.euclidtime3i.Region;
import tt.euclidtime3i.ShortestPathHeuristic;
import tt.euclidtime3i.discretization.Straight;
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
	protected int goalReachedTime;
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
		
		System.out.println("dobst:");
		for (Region dobst : dObstInflated) {
			BasicSegmentedTrajectory mctraj = (BasicSegmentedTrajectory)((MovingCircle) dobst).getTrajectory();
			System.out.println("r=" + ((MovingCircle) dobst).getRadius() + " " + mctraj.getSegments().get(0) + " --> " + mctraj.getSegments().get(mctraj.getSegments().size()-1));
		}
		
		// check that the start point is free
		
		int tmin = -1;
		double minStartDist = Double.MAX_VALUE;
		Region minDobst = null;
		for (Region dobst : dObstInflated) {
			BasicSegmentedTrajectory mctraj = (BasicSegmentedTrajectory)((MovingCircle) dobst).getTrajectory();
			for (int t = depTime; t <= maxTime-timeStep-1; t += timeStep/2) {
				double dist = mctraj.get(t).distance(start);
				if (dist < minStartDist) {
					minStartDist = dist;
					tmin = t;
					minDobst = dobst;
				}
			}
		}
		
		if (minStartDist < 50.0) {
			SegmentedTrajectory invadedTraj = (SegmentedTrajectory)((MovingCircle) minDobst).getTrajectory();
			System.out.println("The start position " + start + "is invaded at time " + tmin + "with minDist=" + minStartDist  + "by the trajectory " + invadedTraj.getSegments().get(0) + " --> " + invadedTraj.getSegments().get(invadedTraj.getSegments().size()-1));
			System.exit(0);
		}
			
		traj = BestResponse.computeBestResponse(start, minTime, depTime, goal,
				maxSpeed, getPlanningGraph(), spaceTimeHeuristic,
				dObstInflated, maxTime, timeStep, T_PLANNING);
		goalReachedTime = computeDestinationReachedTime((SegmentedTrajectory) traj, goal);
		
		if (traj == null) {
			LOGGER.error(" >>>>>>>>>>>>>>> !!!!! No trajectory found within the runtime limit of " + T_PLANNING + " ms !!!! <<<<<<<<<<<<<<<<<<<<<");
			System.out.println("dobst:");
			for (Region dobst : dObstInflated) {
				BasicSegmentedTrajectory mctraj = (BasicSegmentedTrajectory)((MovingCircle) dobst).getTrajectory();
				System.out.println(mctraj.getSegments().get(0) + " --> " + mctraj.getSegments().get(mctraj.getSegments().size()-1));
			}
			
			throw new RuntimeException("Failed to find a trajectory");
		}
		
		LOGGER.debug(getName() + " finished planning in " + (System.currentTimeMillis() - startedAt) + "ms");

		double minDist = BestResponse.minDistance(traj, dObstInflated, timeStep/2);
		LOGGER.debug("The resulting trajectory on interval [" + traj.getMinTime() + "," + traj.getMaxTime() + "] has min-distance " + minDist  + " to other trajectories. Goal will be reached at " + goalReachedTime);
		
		if (minDist < 50.0) {
			System.exit(0);
		}
		
		return traj;
	}
	
	private int computeDestinationReachedTime(SegmentedTrajectory traj,	Point goal) {
		List<Straight> segments = traj.getSegments();
		assert segments.get(segments.size()-1).getEnd().getPosition().equals(goal);

		int i = segments.size()-1;
		while (i > 0 && segments.get(i).getStart().getPosition().equals(segments.get(i).getEnd().getPosition())) 
			i--;
		
		return segments.get(i).getEnd().getTime();
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
			dObstInflated.add(new MovingCircle(mc.getTrajectory(), mc.getRadius() + radius, mc.getSamplingInterval()));
		}
		return dObstInflated;
	}
	
	@Override
	public EvaluatedTrajectory getCurrentTrajectory() {
		return currentTrajectory;
	}

	@Override
	public Point getCurrentPos() {
		if (currentTrajectory != null && currentTrajectory.get(CommonTime.currentTimeMs()) != null) {
			currentPos = currentTrajectory.get(CommonTime.currentTimeMs());
		} 
		return currentPos;
	}
	
	@Override
	protected boolean currentTaskDestinationReached() {
		return CommonTime.currentTimeMs() >= this.goalReachedTime;
	}
	
}

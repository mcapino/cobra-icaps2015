package cz.agents.map4rt.agent;

import java.util.List;

import org.jgrapht.DirectedGraph;
import org.jgrapht.util.HeuristicToGoal;

import cz.agents.map4rt.CommonTime;
import tt.euclid2i.EvaluatedTrajectory;
import tt.euclid2i.Line;
import tt.euclid2i.Point;
import tt.euclid2i.probleminstance.Environment;
import tt.jointeuclid2ni.probleminstance.RelocationTask;

public class BaselineAgent extends PlanningAgent {

	public BaselineAgent(String name, Point start, List<RelocationTask> tasks,
			Environment env, DirectedGraph<Point, Line> planningGraph,
			int agentBodyRadius, float maxSpeed, int maxTime, int timeStep) {
		super(name, start, tasks, env, planningGraph, agentBodyRadius, maxSpeed, maxTime, timeStep);
	}

	@Override
	protected void handleNewTask(RelocationTask task) {
		
		// start at a multiple of timestep
		int depTime = CommonTime.currentTimeMs();
		final Point startPoint = getCurrentPos();
		final Point goal = task.getDestination();
		
		LOGGER.debug(getName() + " started planning " + start + "@"  + depTime + " -> " + goal + " maxtime=" + maxTime);
		
		EvaluatedTrajectory traj = BestResponse.computeShortestPath(startPoint, depTime, goal, maxSpeed, maxTime, getPlanningGraph(), new HeuristicToGoal<Point>() {
			@Override
			public double getCostToGoalEstimate(Point current) {
				return current.distance(goal);
			}
		});
		
		currentTrajectory = traj;
	}

	@Override
	public void start() {}

	@Override
	protected boolean currentTaskDestinationReached() {
		return currentTask.getDestination().equals(getCurrentPos());
	}
	
	

}

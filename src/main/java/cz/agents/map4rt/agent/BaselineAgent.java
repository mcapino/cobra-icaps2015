package cz.agents.map4rt.agent;

import java.util.Random;

import org.jgrapht.DirectedGraph;
import org.jgrapht.util.HeuristicToGoal;

import cz.agents.map4rt.CommonTime;
import tt.euclid2i.EvaluatedTrajectory;
import tt.euclid2i.Line;
import tt.euclid2i.Point;
import tt.euclid2i.probleminstance.Environment;

public class BaselineAgent extends PlanningAgent {

	public BaselineAgent(String name, Point start, int nTasks,
			Environment env, DirectedGraph<Point, Line> planningGraph,
			int agentBodyRadius, float maxSpeed, int maxTime, int timeStep, Random random) {
		super(name, start, nTasks, env, planningGraph, agentBodyRadius, maxSpeed, maxTime, timeStep, random);
	}

	@Override
	protected void handleNewTask(final Point task) {
		
		// start at a multiple of timestep
		int depTime = CommonTime.currentTimeMs();
		final Point startPoint = getCurrentPos();
		
		LOGGER.debug(getName() + " started planning " + start + "@"  + depTime + " -> " + task + " maxtime=" + maxTime);
		
		EvaluatedTrajectory traj = BestResponse.computeShortestPath(startPoint, depTime, task, maxSpeed, maxTime, getPlanningGraph(), new HeuristicToGoal<Point>() {
			@Override
			public double getCostToGoalEstimate(Point current) {
				return current.distance(task);
			}
		});
		
		currentTrajectory = traj;
	}

	@Override
	public void start() {}

	@Override
	protected boolean currentTaskDestinationReached() {
		return currentTask.equals(getCurrentPos());
	}
	
	

}

package cz.agents.map4rt.agent;

import java.util.LinkedList;
import java.util.List;
import java.util.Random;

import org.jgrapht.DirectedGraph;

import cz.agents.map4rt.CommonTime;
import tt.euclid2i.EvaluatedTrajectory;
import tt.euclid2i.Line;
import tt.euclid2i.Point;
import tt.euclid2i.probleminstance.Environment;
import tt.euclidtime3i.Region;
import tt.jointeuclid2ni.probleminstance.RelocationTask;

public class BaselineSTAgent extends PlanningAgent {

	public BaselineSTAgent(String name, Point start, int nTasks,
			Environment env, DirectedGraph<Point, Line> planningGraph,
			int agentBodyRadius, float maxSpeed, int maxTime, int timeStep, int seed, Random random) {
		super(name, start, nTasks, env, planningGraph, agentBodyRadius, maxSpeed, maxTime, timeStep, random);
	}

	@Override
	protected void handleNewTask(Point task) {
		
		// nearest timestep in past
		int minTime = ((int) Math.floor( (double) CommonTime.currentTimeMs() / (double) timeStep) ) * timeStep;
		// start at a multiple of timestep
		int depTime = ((int) Math.ceil( (double) (CommonTime.currentTimeMs() + T_PLANNING) / (double) timeStep) ) * timeStep;
		
		EvaluatedTrajectory traj = getBestResponseTrajectory(
				getCurrentPos(), minTime, depTime, task,
				new LinkedList<Region>(), maxTime);
		
		currentTrajectory = traj;
		lastTaskTravelStartedAt = depTime;
		currentTaskTouchedGoal = false;
	}

	@Override
	public void start() {}

}

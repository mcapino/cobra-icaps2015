package cz.agents.map4rt.agent;

import java.util.LinkedList;
import java.util.List;

import org.jgrapht.DirectedGraph;

import cz.agents.map4rt.CommonTime;
import tt.euclid2i.EvaluatedTrajectory;
import tt.euclid2i.Line;
import tt.euclid2i.Point;
import tt.euclid2i.probleminstance.Environment;
import tt.euclidtime3i.Region;
import tt.jointeuclid2ni.probleminstance.RelocationTask;

public class BaselineAgent extends PlanningAgent {

	public BaselineAgent(String name, Point start, List<RelocationTask> tasks,
			Environment env, DirectedGraph<Point, Line> planningGraph,
			int agentBodyRadius, float maxSpeed, int maxTime, int timeStep) {
		super(name, start, tasks, env, planningGraph, agentBodyRadius, maxSpeed, maxTime, timeStep);
	}

	@Override
	protected void handleNewTask(RelocationTask task) {
		
		// nearest timestep in past
		int minTime = ((int) Math.floor( (double) CommonTime.currentTimeMs() / (double) timeStep) ) * timeStep;
		// start at a multiple of timestep
		int depTime = ((int) Math.ceil( (double) (CommonTime.currentTimeMs() + T_PLANNING) / (double) timeStep) ) * timeStep;
		
		EvaluatedTrajectory traj = getBestResponseTrajectory(
				getCurrentPos(), minTime, depTime, task.getDestination(),
				new LinkedList<Region>(), maxTime);
		
		currentTrajectory = traj;
	}

	@Override
	public void start() {}

}

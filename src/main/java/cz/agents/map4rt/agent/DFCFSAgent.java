package cz.agents.map4rt.agent;

import java.util.Random;

import org.jgrapht.DirectedGraph;

import tt.euclid2i.EvaluatedTrajectory;
import tt.euclid2i.Line;
import tt.euclid2i.Point;
import tt.euclid2i.probleminstance.Environment;
import tt.euclidtime3i.region.MovingCircle;
import cz.agents.map4rt.CommonTime;

public class DFCFSAgent extends PlanningAgent {

	public DFCFSAgent(String name, Point start, int nTasks,
			Environment env, DirectedGraph<Point, Line> planningGraph, 
			int agentBodyRadius, float maxSpeed, int maxTime, int timeStep, Random random) {
		super(name, start, nTasks, env, planningGraph, agentBodyRadius, maxSpeed, maxTime, timeStep, random);
		
		handleNewTask(start);
	}

	@Override
	protected void handleNewTask(Point task) {
		// nearest timestep in past
		int minTime = ((int) Math.floor( (double) CommonTime.currentTimeMs() / (double) timeStep) ) * timeStep;
		// start at a multiple of timestep
		int depTime = ((int) Math.ceil( (double) (CommonTime.currentTimeMs() + T_PLANNING) / (double) timeStep) ) * timeStep;
		
		EvaluatedTrajectory traj = getBestResponseTrajectory(
				getCurrentPos(), minTime, depTime, task,
				Token.getReservedRegions(getName()), maxTime);
		
		assert timeStep % 2 == 0;
		int samplingInterval = timeStep/2;
		Token.register(getName(), new MovingCircle(traj, agentBodyRadius, samplingInterval));
		currentTrajectory = traj;
		lastTaskTravelStartedAt = depTime;
		currentTaskTouchedGoal = false;
	}

	@Override
	public void start() {}

}

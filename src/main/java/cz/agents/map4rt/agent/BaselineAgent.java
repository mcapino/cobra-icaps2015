package cz.agents.map4rt.agent;

import java.util.LinkedList;
import java.util.List;

import org.jgrapht.DirectedGraph;

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
		
		EvaluatedTrajectory traj = getBestResponseTrajectory(
				new tt.euclidtime3i.Point(getCurrentPos(), (int) (time + T_PLANNING)), task.getDestination(),
				new LinkedList<tt.euclid2i.Region>(), new LinkedList<Region>(), maxTime);
		
		currentTrajectory = traj;
	}

	@Override
	public void start() {}

}

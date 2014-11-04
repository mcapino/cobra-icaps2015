package cz.agents.map4rt.agent;

import java.util.LinkedList;
import java.util.List;

import org.jgrapht.DirectedGraph;

import tt.euclid2i.EvaluatedTrajectory;
import tt.euclid2i.Line;
import tt.euclid2i.Point;
import tt.euclid2i.probleminstance.Environment;
import tt.euclidtime3i.Region;
import tt.euclidtime3i.region.MovingCircle;
import tt.jointeuclid2ni.probleminstance.RelocationTask;
import tt.jointeuclid2ni.probleminstance.RelocationTaskImpl;

public class DFCFSAgent extends PlanningAgent {

	public DFCFSAgent(String name, Point start, List<RelocationTask> tasks,
			Environment env, DirectedGraph<Point, Line> planningGraph, 
			int agentBodyRadius, float maxSpeed, int maxTime, int timeStep) {
		super(name, start, tasks, env, planningGraph, agentBodyRadius, maxSpeed, maxTime, timeStep);
		
		handleNewTask(new RelocationTaskImpl(0, 0, start));
	}

	@Override
	protected void handleNewTask(RelocationTask task) {
		
		synchronized (Token.class) {
			
			int startTime = ((int) Math.ceil( (double) (time + T_PLANNING) / (double) timeStep) ) * timeStep;
			
			EvaluatedTrajectory traj = getBestResponseTrajectory(
					new tt.euclidtime3i.Point(getCurrentPos(), startTime), task.getDestination(),
					new LinkedList<tt.euclid2i.Region>(), Token.getReservedRegions(getName()), maxTime);
			
			Token.register(getName(), new MovingCircle(traj, agentBodyRadius, (int) (agentBodyRadius/maxSpeed)/4 ));
			currentTrajectory = traj;
		}
	}

	@Override
	public void start() {}

}

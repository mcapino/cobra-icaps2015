package cz.agents.map4rt.agent;

import java.util.List;

import org.jgrapht.DirectedGraph;

import tt.euclid2i.EvaluatedTrajectory;
import tt.euclid2i.Line;
import tt.euclid2i.Point;
import tt.euclid2i.probleminstance.Environment;
import tt.euclidtime3i.region.MovingCircle;
import tt.jointeuclid2ni.probleminstance.RelocationTask;
import tt.jointeuclid2ni.probleminstance.RelocationTaskImpl;

public class DFCFSAgent extends PlanningAgent {

	private RelocationTask taskToBePlannedFor;

	public DFCFSAgent(String name, Point start, List<RelocationTask> tasks,
			Environment env, DirectedGraph<Point, Line> planningGraph, 
			int agentBodyRadius, float maxSpeed, int maxTime, int timeStep) {
		super(name, start, tasks, env, planningGraph, agentBodyRadius, maxSpeed, maxTime, timeStep);
		
		handleNewTask(new RelocationTaskImpl(0, 0, start));
	}

	@Override
	protected void handleNewTask(RelocationTask task) {
		taskToBePlannedFor = task; // in the subsequent ticks, try to acquire the token and plan the trajectory
	}

	@Override
	public void start() {}

	@Override
	public void tick(int timeMs) {
		super.tick(timeMs);
		
		if (taskToBePlannedFor != null) {
			if (Token.tryLock()) {
				
				// nearest timestep in past
				int minTime = ((int) Math.floor( (double) time / (double) timeStep) ) * timeStep;
				// start at a multiple of timestep
				int depTime = ((int) Math.ceil( (double) (time + T_PLANNING) / (double) timeStep) ) * timeStep;
				
				EvaluatedTrajectory traj = getBestResponseTrajectory(
						getCurrentPos(), minTime, depTime, taskToBePlannedFor.getDestination(),
						Token.getReservedRegions(getName()), maxTime);
				
				Token.register(getName(), new MovingCircle(traj, agentBodyRadius, (int) (agentBodyRadius/maxSpeed)/4 ));
				currentTrajectory = traj;
				taskToBePlannedFor = null;
				Token.unlock();
			}
		}
	}
	
	
	

}

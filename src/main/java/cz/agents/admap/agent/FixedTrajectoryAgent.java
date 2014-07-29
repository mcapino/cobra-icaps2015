package cz.agents.admap.agent;

import tt.euclid2i.EvaluatedTrajectory;
import tt.euclid2i.Point;
import tt.euclid2i.probleminstance.Environment;

public class FixedTrajectoryAgent extends Agent {

	private EvaluatedTrajectory traj;
	
	public FixedTrajectoryAgent(String name, Point start, Point goal,
			Environment environment, int agentBodyRadius,
			EvaluatedTrajectory traj) {
		super(name, start, goal, environment, agentBodyRadius);
		this.traj = traj;
	}

	@Override
	public EvaluatedTrajectory getCurrentTrajectory() {
		return traj;
	}

	@Override
	public void start() {
	}

	@Override
	public boolean isGlobalTerminationDetected() {
		return false;
	}

	@Override
	public int getMessageSentCounter() {
		return 1;
	}
	
	@Override
	public boolean hasSucceeded() {
		return true;
	}

}

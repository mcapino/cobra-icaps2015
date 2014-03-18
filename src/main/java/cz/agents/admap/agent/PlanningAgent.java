package cz.agents.admap.agent;

import tt.euclid2i.EvaluatedTrajectory;
import tt.euclid2i.Point;
import tt.euclid2i.probleminstance.Environment;

/**
 * An agent that resolves conflict by finding new conflict-free plans.
 */
public abstract class PlanningAgent extends Agent {

	EvaluatedTrajectory trajectory;

	public PlanningAgent(String name, Point start, Point goal,
			Environment environment, int agentBodyRadius) {
		super(name, start, goal, environment, agentBodyRadius);
	}


}

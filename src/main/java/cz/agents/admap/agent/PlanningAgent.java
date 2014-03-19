package cz.agents.admap.agent;

import java.util.Collection;

import tt.euclid2i.EvaluatedTrajectory;
import tt.euclid2i.Point;
import tt.euclid2i.probleminstance.Environment;
import tt.euclidtime3i.Region;

/**
 * An agent that resolves conflict by finding new conflict-free plans.
 */
public abstract class PlanningAgent extends Agent {

	EvaluatedTrajectory trajectory;

	public PlanningAgent(String name, Point start, Point goal,
			Environment environment, int agentBodyRadius) {
		super(name, start, goal, environment, agentBodyRadius);
	}

	protected EvaluatedTrajectory getBestResponseTrajectory(Collection<Region> avoidRegions) {
		if (getPlanningGraph() != null) {
    		return Util.computeBestResponse(start, goal, getPlanningGraph(), avoidRegions);
    	} else {
    		return Util.computeBestResponse(start, goal, inflatedObstacles, environment.getBoundary().getBoundingBox(), avoidRegions);
    	}
	}


}

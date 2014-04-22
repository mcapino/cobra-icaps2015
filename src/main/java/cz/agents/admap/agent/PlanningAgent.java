package cz.agents.admap.agent;

import java.util.Collection;
import java.util.LinkedList;

import org.apache.log4j.Logger;

import tt.euclid2i.EvaluatedTrajectory;
import tt.euclid2i.Point;
import tt.euclid2i.probleminstance.Environment;
import tt.euclidtime3i.Region;
import tt.euclidtime3i.region.StaticObstacle;

/**
 * An agent that resolves conflict by finding new conflict-free plans.
 */
public abstract class PlanningAgent extends Agent {

	Logger LOGGER = Logger.getLogger(PlanningAgent.class);

	EvaluatedTrajectory trajectory;

	public PlanningAgent(String name, Point start, Point goal,
			Environment environment, int agentBodyRadius) {
		super(name, start, goal, environment, agentBodyRadius);
	}

	protected EvaluatedTrajectory getBestResponseTrajectory(Collection<tt.euclid2i.Region> sRegions, Collection<Region> dRegions) {

		Collection<Region> regions = new LinkedList<>(dRegions);
		for (tt.euclid2i.Region region : sRegions) {
			regions.add(new StaticObstacle(region));
		}
		return getBestResponseTrajectory(regions);
	}

	protected EvaluatedTrajectory getBestResponseTrajectory(Collection<Region> avoidRegions) {

		LOGGER.debug(getName() + " started planning ...");
		long startedAt = System.currentTimeMillis();

		EvaluatedTrajectory traj;
		if (getPlanningGraph() != null) {
    		traj = BestResponse.computeBestResponse(start, goal, getPlanningGraph(), avoidRegions);
    	} else {
    		traj = BestResponse.computeBestResponse(start, goal, inflatedObstacles, environment.getBoundary().getBoundingBox(), avoidRegions);
    	}

		LOGGER.debug(getName() + " finished planning in " + (System.currentTimeMillis() - startedAt) + "ms");
		return traj;
	}


}

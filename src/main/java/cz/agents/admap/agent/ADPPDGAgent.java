package cz.agents.admap.agent;

import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;

import org.apache.log4j.Logger;

import tt.euclid2i.EvaluatedTrajectory;
import tt.euclid2i.Point;
import tt.euclid2i.Trajectory;
import tt.euclid2i.probleminstance.Environment;
import tt.euclidtime3i.L2Heuristic;
import tt.euclidtime3i.Region;
import tt.euclidtime3i.region.MovingCircle;
import tt.euclidtime3i.util.IntersectionChecker;
import cz.agents.admap.msg.InformNewTrajectory;
import cz.agents.alite.communication.Message;

public class ADPPDGAgent extends PlanningAgent {

	static final Logger LOGGER = Logger.getLogger(ADPPDGAgent.class);

    Map<String, Objectives> group = new HashMap<String, Objectives>();
    Map<String, Trajectory> trajectories =  new HashMap<String, Trajectory>();
    Map<String, MovingCircle> avoids =  new HashMap<String, MovingCircle>();

    public ADPPDGAgent(String name, Point start, Point goal, Environment environment, int agentBodyRadius, int maxTime, int waitMoveDuration) {
        super(name, start, goal, environment, agentBodyRadius, maxTime, waitMoveDuration);
        this.group.put(name, new Objectives(start, goal));
    }

    @Override
    public EvaluatedTrajectory getCurrentTrajectory() {
        return trajectory;
    }

    @Override
    public void start() {
    	if (getPlanningGraph() != null) {
    		trajectory = BestResponse.computeBestResponse(start, goal, getPlanningGraph(), new L2Heuristic(goal), Collections.EMPTY_SET, new LinkedList<Region>(), maxTime, waitMoveDuration);
    	} else {
    		trajectory = BestResponse.computeBestResponse(start, goal, inflatedObstacles, environment.getBoundary().getBoundingBox(), Collections.EMPTY_SET, new LinkedList<Region>(), maxTime, waitMoveDuration);
    	}
    	broadcast(new InformNewTrajectory(getName(), new MovingCircle(getCurrentTrajectory(), agentBodyRadius)));
    }

    private void replan() {

        // inflate the space-time regions

        Collection<tt.euclidtime3i.Region> avoidRegions = new LinkedList<tt.euclidtime3i.Region>();
        for (MovingCircle movingCircle : avoids.values()) {
            avoidRegions.add(new MovingCircle(movingCircle.getTrajectory(), movingCircle.getRadius() + agentBodyRadius));
        }

        if (getCurrentTrajectory() != null &&
        	IntersectionChecker.intersect(new MovingCircle(getCurrentTrajectory(), agentBodyRadius), new LinkedList<tt.euclidtime3i.Region>(avoids.values()))) {

        	LOGGER.trace(getName() + " started planning...");

        	if (getPlanningGraph() != null) {
        		trajectory = BestResponse.computeBestResponse(start, goal, getPlanningGraph(), new L2Heuristic(goal), Collections.EMPTY_SET, avoidRegions, maxTime, waitMoveDuration);
        	} else {
        		trajectory = BestResponse.computeBestResponse(start, goal, inflatedObstacles, environment.getBoundary().getBoundingBox(), Collections.EMPTY_SET, avoidRegions, maxTime, waitMoveDuration);
        	}

	        LOGGER.trace(getName() + " has a new trajectory. Cost: " + trajectory.getCost());

	        // broadcast to the others
	        broadcast(new InformNewTrajectory(getName(), new MovingCircle(getCurrentTrajectory(), agentBodyRadius)));
        }
    }

    @Override
    protected void notify(Message message) {
        super.notify(message);
        if (message.getContent() instanceof InformNewTrajectory) {
            InformNewTrajectory newTrajectoryMessage = (InformNewTrajectory) (message.getContent());
            String agentName = newTrajectoryMessage.getAgentName();
            MovingCircle occupiedRegion = (MovingCircle) newTrajectoryMessage.getRegion();

            if (agentName.compareTo(getName()) < 0) {
            	//LOGGER.trace(getName() + " adding new trajectory " + occupiedRegion.getTrajectory() + " of " + agentName + " to avoids set");
                avoids.put(agentName, occupiedRegion);
                replan();
            }
        }
    }

	@Override
	public boolean isGlobalTerminationDetected() {
		return false;
	}
	
	@Override
	public boolean hasSucceeded() {
		return true;
	}
}

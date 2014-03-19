package cz.agents.admap.agent;

import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import org.apache.log4j.Logger;

import tt.euclid2i.EvaluatedTrajectory;
import tt.euclid2i.Point;
import tt.euclid2i.Trajectory;
import tt.euclid2i.probleminstance.Environment;
import tt.euclidtime3i.Region;
import tt.euclidtime3i.region.MovingCircle;
import tt.euclidtime3i.util.IntersectionChecker;
import cz.agents.admap.msg.InformFinished;
import cz.agents.admap.msg.InformNewTrajectory;
import cz.agents.alite.communication.Communicator;
import cz.agents.alite.communication.Message;

public class ADPPAgent extends PlanningAgent {

	static final Logger LOGGER = Logger.getLogger(ADPPAgent.class);

    Map<String, MovingCircle> avoids =  new HashMap<String, MovingCircle>();

    boolean higherPriorityRobotsFinished = false;
    boolean finished = false;

    List<String> sortedAgents = new LinkedList<String>();

    public ADPPAgent(String name, Point start, Point goal, Environment environment, int agentBodyRadius) {
        super(name, start, goal, environment, agentBodyRadius);
    }

    @Override
	public void setCommunicator(Communicator communicator, List<String> agents) {
		super.setCommunicator(communicator, agents);
		sortedAgents = new LinkedList<String>(agents);
		Collections.sort(sortedAgents);
	}

	@Override
    public EvaluatedTrajectory getCurrentTrajectory() {
        return trajectory;
    }

    @Override
    public void start() {
    	trajectory = getBestResponseTrajectory(Collections.<Region>emptySet());

    	broadcastNewTrajectory(getCurrentTrajectory());

    	if (isHighestPriority()) {
    		finished = true;
    		broadcastFinished();
    		LOGGER.info(getName() +  " has finished...");
    	}
    }

    private void broadcastNewTrajectory(EvaluatedTrajectory currentTrajectory) {
    	broadcast(new InformNewTrajectory(getName(), new MovingCircle(getCurrentTrajectory(), agentBodyRadius)));
	}

    private void broadcastFinished() {
    	broadcast(new InformFinished(getName()));
	}

	private boolean isHighestPriority() {
		return sortedAgents.get(0).equals(getName());
	}

	private void replan() {

        // inflate the space-time regions

        Collection<tt.euclidtime3i.Region> avoidRegions = new LinkedList<tt.euclidtime3i.Region>();
        for (MovingCircle movingCircle : avoids.values()) {
            avoidRegions.add(new MovingCircle(movingCircle.getTrajectory(), movingCircle.getRadius() + agentBodyRadius));
        }

        if (getCurrentTrajectory() != null) {

        	if (IntersectionChecker.intersect(new MovingCircle(getCurrentTrajectory(), agentBodyRadius),
        			new LinkedList<tt.euclidtime3i.Region>(avoids.values()))) {
        		// The current trajectory is inconsistent
	        	LOGGER.trace(getName() + " started planning...");

	        	trajectory = getBestResponseTrajectory(avoidRegions);

		        LOGGER.trace(getName() + " has a new trajectory. Cost: " + trajectory.getCost());

		        // broadcast to the others
		        broadcastNewTrajectory(getCurrentTrajectory());
        	}

        	if (higherPriorityRobotsFinished) {
        		// we have consistent trajectory and the higher-priority agents are fixed
        		finished = true;
        		broadcastFinished();
        		LOGGER.info(getName() +  " has finished...");
        	}
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

        if (message.getContent() instanceof InformFinished) {
        	String agentName = ((InformFinished) message.getContent()).getAgentName();
            if (isMyPredecessor(agentName)) {
            	higherPriorityRobotsFinished = true;
            	replan();
            }
        }
    }

	private boolean isMyPredecessor(String agentName) {
		for (int i=0; i < sortedAgents.size()-1; i++) {
			if (sortedAgents.get(i).equals(agentName) && sortedAgents.get(i+1).equals(getName())) {
				return true;
			}
		}
		return false;
	}

	@Override
	public boolean isFinished() {
		return finished;
	}

}

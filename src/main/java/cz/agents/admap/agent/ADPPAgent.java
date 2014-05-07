package cz.agents.admap.agent;

import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import org.apache.log4j.Logger;

import tt.euclid2i.EvaluatedTrajectory;
import tt.euclid2i.Point;
import tt.euclid2i.SegmentedTrajectory;
import tt.euclid2i.probleminstance.Environment;
import tt.euclid2i.region.Circle;
import tt.euclid2i.trajectory.SegmentedTrajectories;
import tt.euclidtime3i.Region;
import tt.euclidtime3i.region.MovingCircle;
import tt.euclidtime3i.util.IntersectionChecker;
import cz.agents.admap.msg.InformFinished;
import cz.agents.admap.msg.InformNewTrajectory;
import cz.agents.alite.communication.Communicator;
import cz.agents.alite.communication.Message;

public class ADPPAgent extends PlanningAgent {

	static final Logger LOGGER = Logger.getLogger(ADPPAgent.class);

    Map<String, MovingCircle> agentView =  new HashMap<String, MovingCircle>();

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
    	if (isHighestPriority()) {
    		higherPriorityRobotsFinished = true;
    	}

    	assertConsistency();
    }

    private void broadcastNewTrajectory(EvaluatedTrajectory newTrajectory) {
    	broadcast(new InformNewTrajectory(getName(), new MovingCircle(newTrajectory, agentBodyRadius)));
	}

    private void broadcastFinished() {
    	broadcast(new InformFinished(getName()));
	}

	private boolean isHighestPriority() {
		return sortedAgents.get(0).equals(getName());
	}

	private void assertConsistency() {
		if (getCurrentTrajectory() == null) {
	    	trajectory = assertConsistency(getCurrentTrajectory(), Collections.<tt.euclid2i.Region> emptySet(), Collections.<Region> emptySet());
		} else {

			Collection<tt.euclid2i.Region> sObst = new LinkedList<tt.euclid2i.Region>();
			Collection<tt.euclidtime3i.Region> dObst = new LinkedList<tt.euclidtime3i.Region>();

	        for (Entry<String, MovingCircle> entry : agentView.entrySet()) {
	        	String name = entry.getKey();
	        	MovingCircle movingCircle = entry.getValue();

	        	if (getName().compareTo(name) > 0) {
	        		// Dynamic obstacles
	        		dObst.add(new MovingCircle(movingCircle.getTrajectory(), movingCircle.getRadius()));
	        	} else {
	        		// Static obstacles
	        		sObst.add(new Circle(movingCircle.getTrajectory().get(0), movingCircle.getRadius()));
	        	}
	        }

	        trajectory = assertConsistency(getCurrentTrajectory(), sObst, dObst);
		}

    	if (!finished && higherPriorityRobotsFinished && lowerPriorityAgentViewFull()) {
    		// we have consistent trajectory and the higher-priority agents are fixed
    		finished = true;
    		broadcastFinished();
    		LOGGER.info(getName() +  " has finished!");
    	}

	}

	private EvaluatedTrajectory assertConsistency(EvaluatedTrajectory currentTraj, Collection<tt.euclid2i.Region> sObst, Collection<Region> dObst) {

		if (currentTraj == null || !consistent(new MovingCircle(currentTraj, agentBodyRadius), sObst, dObst)) {
    		// The current trajectory is inconsistent
			LOGGER.trace(getName() + " detected inconsistency");

        	EvaluatedTrajectory newTrajectory = getBestResponseTrajectory(sObst, dObst);

        	if (newTrajectory == null) {
        		// Failure
        		throw new RuntimeException(getName() + ": FAIL: Cannot find a consistent trajectory.");
        	}

	        LOGGER.trace(getName() + " has a new trajectory. Cost: " + newTrajectory.getCost());

	        // broadcast to the others
	        broadcastNewTrajectory(newTrajectory);
        	return newTrajectory;
		} else {
			return currentTraj;
		}
    }

    private boolean lowerPriorityAgentViewFull() {

    	for (String otherAgentName : sortedAgents) {
    		if (otherAgentName.compareTo(getName()) > 0) {
    			if (!agentView.containsKey(otherAgentName)) {
    				return false;
    			}
    		}
    	}

    	return true;
   	}

	private boolean consistent(MovingCircle movingCircle, Collection<tt.euclid2i.Region> sObst, Collection<Region> dObst) {

    	assert movingCircle.getTrajectory() instanceof SegmentedTrajectory;
    	LinkedList<tt.euclid2i.Region> sObstInflated = inflateStaticObstacles(sObst, agentBodyRadius);

    	boolean consistentWithStaticObstacles = SegmentedTrajectories.isInFreeSpace((SegmentedTrajectory) movingCircle.getTrajectory(), sObstInflated);
    	boolean consistentWithDynamicObstacles = !IntersectionChecker.intersect(movingCircle, dObst);
    	return  consistentWithStaticObstacles && consistentWithDynamicObstacles;
	}

	@Override
    protected void notify(Message message) {
        super.notify(message);
        if (message.getContent() instanceof InformNewTrajectory) {
            InformNewTrajectory newTrajectoryMessage = (InformNewTrajectory) (message.getContent());
            String agentName = newTrajectoryMessage.getAgentName();
            MovingCircle occupiedRegion = (MovingCircle) newTrajectoryMessage.getRegion();

            LOGGER.trace(occupiedRegion.getTrajectory());

            if (agentName.compareTo(getName()) != 0) {
                agentView.put(agentName, occupiedRegion);
                assertConsistency();
            }
        }

        if (message.getContent() instanceof InformFinished) {
        	String agentName = ((InformFinished) message.getContent()).getAgentName();
        	if (isMyPredecessor(agentName)) {
        		higherPriorityRobotsFinished = true;
        		assertConsistency();
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

	@Override
	public void tick(long time) {
		super.tick(time);
	}

}

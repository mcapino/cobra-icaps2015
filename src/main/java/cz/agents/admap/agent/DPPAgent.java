package cz.agents.admap.agent;

import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import org.apache.log4j.Logger;

import cz.agents.admap.msg.InformAgentFinished;
import cz.agents.admap.msg.InformNewTrajectory;
import cz.agents.alite.communication.Communicator;
import cz.agents.alite.communication.Message;
import tt.euclid2i.EvaluatedTrajectory;
import tt.euclid2i.Point;
import tt.euclid2i.SegmentedTrajectory;
import tt.euclid2i.probleminstance.Environment;
import tt.euclid2i.region.Circle;
import tt.euclid2i.trajectory.SegmentedTrajectories;
import tt.euclidtime3i.Region;
import tt.euclidtime3i.region.MovingCircle;
import tt.euclidtime3i.util.IntersectionCheckerWithProtectedPoint;

public abstract class DPPAgent extends PlanningAgent {
	
	static final Logger LOGGER = Logger.getLogger(DPPAgent.class);

	public DPPAgent(String name, Point start, Point goal,
			Environment environment, int agentBodyRadius) {
		super(name, start, goal, environment, agentBodyRadius);
	}
	
    Map<String, MovingCircle> agentView =  new HashMap<String, MovingCircle>();

    boolean higherPriorityAgentsFinished = false;
    boolean agentFinished = false;

    List<String> sortedAgents = new LinkedList<String>();

	protected boolean agentViewDirty;

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

	protected void broadcastNewTrajectory(EvaluatedTrajectory newTrajectory) {
    	broadcast(new InformNewTrajectory(getName(), new MovingCircle(newTrajectory, agentBodyRadius)));
	}

    protected void broadcastAgentFinished() {
    	broadcast(new InformAgentFinished(getName()));
	}

	protected boolean isHighestPriority() {
		return sortedAgents.get(0).equals(getName());
	}
	
	protected boolean isLowestPriority() {
		return sortedAgents.get(sortedAgents.size()-1).equals(getName());
	}
	
	protected Collection<tt.euclid2i.Region> sObst() {
		Collection<tt.euclid2i.Region> sObst = new LinkedList<tt.euclid2i.Region>();

        for (Entry<String, MovingCircle> entry : agentView.entrySet()) {
        	String name = entry.getKey();
        	MovingCircle movingCircle = entry.getValue();

        	if (getName().compareTo(name) < 0) {
        		// Static obstacles
        		sObst.add(new Circle(movingCircle.getTrajectory().get(0), movingCircle.getRadius()));
        	}
        }
        
        return sObst;
	}
	
	protected  Collection<Region> dObst() {
		Collection<tt.euclidtime3i.Region> dObst = new LinkedList<tt.euclidtime3i.Region>();

        for (Entry<String, MovingCircle> entry : agentView.entrySet()) {
        	String name = entry.getKey();
        	MovingCircle movingCircle = entry.getValue();

        	if (getName().compareTo(name) > 0) {
        		// Dynamic obstacles
        		dObst.add(new MovingCircle(movingCircle.getTrajectory(), movingCircle.getRadius()));
        	} 
        }
        
        return dObst;
	}

	protected EvaluatedTrajectory assertConsistentTrajectory(EvaluatedTrajectory currentTraj, Collection<tt.euclid2i.Region> sObst, Collection<Region> dObst) {

		if (currentTraj == null || !consistent(new MovingCircle(currentTraj, agentBodyRadius), sObst, dObst)) {
    		// The current trajectory is inconsistent
			LOGGER.trace(getName() + " detected inconsistency. My traj:" + currentTraj + "");
			
        	EvaluatedTrajectory newTrajectory = getBestResponseTrajectory(sObst, dObst, getStart());

        	if (newTrajectory == null) {
        		// Failure
        		throw new RuntimeException(getName() + ": FAILURE: Cannot find a consistent trajectory.");
        	}

	        LOGGER.trace(getName() + " has a new trajectory. Cost: " + newTrajectory.getCost());

	        // broadcast to the others
	        broadcastNewTrajectory(newTrajectory);
        	return newTrajectory;
		} else {
			return currentTraj;
		}
    }

    protected boolean lowerPriorityAgentViewFull() {

    	for (String otherAgentName : sortedAgents) {
    		if (otherAgentName.compareTo(getName()) > 0) {
    			if (!agentView.containsKey(otherAgentName)) {
    				return false;
    			}
    		}
    	}

    	return true;
   	}

	protected boolean consistent(MovingCircle movingCircle, Collection<tt.euclid2i.Region> sObst, Collection<Region> dObst) {

    	assert movingCircle.getTrajectory() instanceof SegmentedTrajectory;
    	LinkedList<tt.euclid2i.Region> sObstInflated = inflateStaticObstacles(sObst, agentBodyRadius);

    	boolean consistentWithStaticObstacles = SegmentedTrajectories.isInFreeSpace((SegmentedTrajectory) movingCircle.getTrajectory(), sObstInflated);
    	boolean consistentWithDynamicObstacles = !IntersectionCheckerWithProtectedPoint.intersect(movingCircle, dObst, getStart());
    	LOGGER.trace("Consistent with static: " + consistentWithStaticObstacles + " Consistent with dynamic: " + consistentWithDynamicObstacles);
    	return  consistentWithStaticObstacles && consistentWithDynamicObstacles;
	}

	@Override
    protected void notify(Message message) {
        super.notify(message);
        if (message.getContent() instanceof InformNewTrajectory) {
            InformNewTrajectory newTrajectoryMessage = (InformNewTrajectory) (message.getContent());
            String agentName = newTrajectoryMessage.getAgentName();
            MovingCircle occupiedRegion = (MovingCircle) newTrajectoryMessage.getRegion();

            if (agentName.compareTo(getName()) != 0) {
                agentView.put(agentName, occupiedRegion);
                agentViewDirty = true;
            }
        }
    }

	protected boolean isMyPredecessor(String agentName) {
		for (int i=0; i < sortedAgents.size()-1; i++) {
			if (sortedAgents.get(i).equals(agentName) && sortedAgents.get(i+1).equals(getName())) {
				return true;
			}
		}
		return false;
	}

	@Override
	public boolean isFinished() {
		return agentFinished;
	}

	@Override
	public void tick(long time) {
		super.tick(time);
	}
	
}

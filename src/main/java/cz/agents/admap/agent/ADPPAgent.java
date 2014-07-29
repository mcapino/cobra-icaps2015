package cz.agents.admap.agent;

import java.util.Collection;
import java.util.Collections;

import org.apache.log4j.Logger;

import cz.agents.admap.msg.InformAgentFailed;
import cz.agents.admap.msg.InformAgentFinished;
import cz.agents.alite.communication.Message;
import tt.euclid2i.Point;
import tt.euclid2i.probleminstance.Environment;
import tt.euclidtime3i.Region;

public class ADPPAgent extends DPPAgent {

	static final Logger LOGGER = Logger.getLogger(ADPPAgent.class);
	private boolean agentFinished = false;

    public ADPPAgent(String name, Point start, Point goal, Environment environment, int agentBodyRadius, int maxTime, int waitMoveDuration, Collection<tt.euclid2i.Region> sObst) {
        super(name, start, goal, environment, agentBodyRadius, maxTime, waitMoveDuration, sObst);
    }
    
    @Override
    public void start() {
    	if (isHighestPriority()) {
    		higherPriorityAgentsFinished = true;
    		agentViewDirty = false;
    	}

    	assertConsistentTrajectory();
    	
    	if (isLowestPriority() && higherPriorityAgentsFinished && trajectory != null) {
    		setGlobalTerminationDetected(true);
    	}
    }

	@Override
	protected void notify(Message message) {
		if (!globalTerminationDetected) {		
			super.notify(message);
			
	        if (message.getContent() instanceof InformAgentFinished) {
	        	String agentName = ((InformAgentFinished) message.getContent()).getAgentName();
	        	if (isMyPredecessor(agentName)) {
	        		higherPriorityAgentsFinished = true;
	        		agentViewDirty = true;
	        	}
	        }
	        
	        if (message.getContent() instanceof InformAgentFailed) {
	        	agentFinished = true;
	        }
			
	        if (agentViewDirty && getInboxSize() == 0 && !agentFinished) {
	        	assertConsistentTrajectory();
	        	agentViewDirty = false;
	        	
	        	if (isLowestPriority() && higherPriorityAgentsFinished && getCurrentTrajectory() != null) {
	        		broadcastSuccessfulConvergence();
	        		LOGGER.info(getName() +  " globally terminated!");
	        		setGlobalTerminationDetected(true);
	        	}
	        }	
		}
	}
	
	protected void assertConsistentTrajectory() {
		if (getCurrentTrajectory() == null) {
	    	trajectory = assertConsistentTrajectory(getCurrentTrajectory(), sObst(), Collections.<Region> emptySet());
		} else {
	        trajectory = assertConsistentTrajectory(getCurrentTrajectory(), sObst(), dObst());
		}
		
		if (trajectory != null) {
			// trajectory found
	    	if (!agentFinished && higherPriorityAgentsFinished && allStartRegionsOfLowerPriorityRobotsKnown()) {
	    		// we have consistent trajectory and the higher-priority agents are fixed
	    		agentFinished = true;
	    		broadcastAgentFinished();
	    		LOGGER.info(getName() +  " has finished!");
	    	}
		} else {
			// trajectory not found
    		agentFinished = true;
    		broadcastFailure();  
    		setGlobalTerminationDetected(false);
    		LOGGER.info(getName() +  " has FAILED !!!");
		}
	}

    
}

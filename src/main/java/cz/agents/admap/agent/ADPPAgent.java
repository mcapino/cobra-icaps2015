package cz.agents.admap.agent;

import java.util.Collections;

import org.apache.log4j.Logger;

import cz.agents.admap.msg.InformAgentFinished;
import cz.agents.alite.communication.Message;
import tt.euclid2i.Point;
import tt.euclid2i.probleminstance.Environment;
import tt.euclidtime3i.Region;

public class ADPPAgent extends DPPAgent {

	static final Logger LOGGER = Logger.getLogger(ADPPAgent.class);

    public ADPPAgent(String name, Point start, Point goal, Environment environment, int agentBodyRadius, int maxTime) {
        super(name, start, goal, environment, agentBodyRadius, maxTime);
    }
    
    @Override
    public void start() {
    	if (isHighestPriority()) {
    		higherPriorityAgentsFinished = true;
    		agentViewDirty = false;
    	}

    	assertConsistentTrajectory();
    	
    	if (isLowestPriority() && higherPriorityAgentsFinished) {
    		agentTerminated();
    	}
    }

	@Override
	protected void notify(Message message) {
		super.notify(message);
		
        if (message.getContent() instanceof InformAgentFinished) {
        	String agentName = ((InformAgentFinished) message.getContent()).getAgentName();
        	if (isMyPredecessor(agentName)) {
        		higherPriorityAgentsFinished = true;
        		agentViewDirty = true;
        	}
        }
		
        if (agentViewDirty && getInboxSize() == 0) {
        	assertConsistentTrajectory();
        	agentViewDirty = false;
        	
        	if (isLowestPriority() && higherPriorityAgentsFinished) {
        		broadcastGloballyConverged();
        		LOGGER.info(getName() +  " globally terminated!");
        		agentTerminated();
        	}
        }		
	}
	
	protected void assertConsistentTrajectory() {
		if (getCurrentTrajectory() == null) {
	    	trajectory = assertConsistentTrajectory(getCurrentTrajectory(), Collections.<tt.euclid2i.Region> emptySet(), Collections.<Region> emptySet());
		} else {
	        trajectory = assertConsistentTrajectory(getCurrentTrajectory(), sObst(), dObst());
		}

    	if (!isTerminated() && higherPriorityAgentsFinished && lowerPriorityAgentViewFull()) {
    		// we have consistent trajectory and the higher-priority agents are fixed
    		broadcastAgentFinished();
    		LOGGER.info(getName() +  " has finished!");
    	}
	}

    
}

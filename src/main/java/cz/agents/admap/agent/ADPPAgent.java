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

    public ADPPAgent(String name, Point start, Point goal, Environment environment, int agentBodyRadius) {
        super(name, start, goal, environment, agentBodyRadius);
    }
    
    @Override
    public void start() {
    	if (isHighestPriority()) {
    		higherPriorityAgentsFinished = true;
    		agentViewDirty = false;
    	}

    	assertConsistentTrajectory();
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
        }		
	}
	
	protected void assertConsistentTrajectory() {
		if (getCurrentTrajectory() == null) {
	    	trajectory = assertConsistentTrajectory(getCurrentTrajectory(), Collections.<tt.euclid2i.Region> emptySet(), Collections.<Region> emptySet());
		} else {
	        trajectory = assertConsistentTrajectory(getCurrentTrajectory(), sObst(), dObst());
		}

    	if (!agentFinished && higherPriorityAgentsFinished && lowerPriorityAgentViewFull()) {
    		// we have consistent trajectory and the higher-priority agents are fixed
    		agentFinished = true;
    		broadcastAgentFinished();
    		LOGGER.info(getName() +  " has finished!");
    	}
	}

    
}

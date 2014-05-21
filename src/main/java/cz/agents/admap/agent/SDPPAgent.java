package cz.agents.admap.agent;

import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

import org.apache.log4j.Logger;

import cz.agents.admap.msg.InformAgentFinished;
import cz.agents.admap.msg.InformAgentFinishedRound;
import cz.agents.admap.msg.InformNewRound;
import cz.agents.admap.msg.InformNewTrajectory;
import cz.agents.alite.communication.Message;
import tt.euclid2i.Point;
import tt.euclid2i.probleminstance.Environment;
import tt.euclidtime3i.Region;
import tt.euclidtime3i.region.MovingCircle;

public class SDPPAgent extends DPPAgent {

	static final Logger LOGGER = Logger.getLogger(SDPPAgent.class);
	int round;
	boolean agentViewChangedInLastRound;
	
    public SDPPAgent(String name, Point start, Point goal, Environment environment, int agentBodyRadius, int maxTime) {
        super(name, start, goal, environment, agentBodyRadius, maxTime);
    }
    
    @Override
    public void start() {
    	
    	round = 0;
    	assertConsistentTrajectory();
    	
    	if (isHighestPriority()) {
    		broadcastAgentFinishedRound();
    		LOGGER.info(getName() +  " has finished planning!");
    		LOGGER.debug(getName() +  ": round " + round + " finished!");
    	}
    }
        
	@Override
	protected void notify(Message message) {
		super.notify(message);
		if (message.getContent() instanceof InformNewTrajectory) {
			agentViewChangedInLastRound = true;
		}
		
		if (message.getContent() instanceof InformAgentFinishedRound) {
			String agentName = ((InformAgentFinishedRound) message.getContent()).getAgentName();
			int roundInMsg = ((InformAgentFinishedRound) message.getContent()).getRound();
			if (isMyPredecessor(agentName) && roundInMsg == round) {
				if (isLowestPriority() ) {
					LOGGER.debug(getName() +  ": round " + round + " finished!");
					// everyone finished in the round
					round++; 
					if (agentViewChangedInLastRound) {
						agentViewChangedInLastRound = false;
						broadcastNewRound();
						LOGGER.debug("--------------------------------- ROUND: " + round + "----------------------------");
						LOGGER.info(getName() +  " new round: " + round);
			        	assertConsistentTrajectory();
					} else {
						// The process converged...
						LOGGER.info("The process converged!");
					}
				} else {
					broadcastAgentFinishedRound();
					LOGGER.debug(getName() +  ": round " + round + " finished!");
				}
			}
		}
		
        if (message.getContent() instanceof InformNewRound) {
        	InformNewRound inrContent = (InformNewRound) message.getContent();
        	round = inrContent.getRoundNo();
        	LOGGER.info(getName() +  " new round: " + round);
        	assertConsistentTrajectory();
        	if (isHighestPriority()) {
        		broadcastAgentFinishedRound();
        		LOGGER.debug(getName() +  ": round " + round + " finished!");
        	}
        	
        }		
        
	}
	
	protected void assertConsistentTrajectory() {
		if (getCurrentTrajectory() == null) {
	    	trajectory = assertConsistentTrajectory(getCurrentTrajectory(), Collections.<tt.euclid2i.Region> emptySet(), Collections.<Region> emptySet());
		} else {
	        trajectory = assertConsistentTrajectory(getCurrentTrajectory(), sObst(), dObst());
		}
	}
	
    protected void broadcastNewRound() {
    	broadcast(new InformNewRound(round));
	}
    
    protected void broadcastAgentFinishedRound() {
    	broadcast(new InformAgentFinishedRound(getName(), round));
	}
	
	

}

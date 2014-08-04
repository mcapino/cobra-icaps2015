package cz.agents.admap.agent;

import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import org.apache.log4j.Logger;

import cz.agents.admap.msg.InformAgentFailed;
import cz.agents.admap.msg.InformAgentFinished;
import cz.agents.admap.msg.InformSuccessfulConvergence;
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
import tt.euclidtime3i.discretization.Straight;
import tt.euclidtime3i.region.CircleMovingToTarget;
import tt.euclidtime3i.region.MovingCircle;
import tt.euclidtime3i.util.IntersectionCheckerWithProtectedPoint;

public abstract class DPPAgent extends PlanningAgent {
	
	static final Logger LOGGER = Logger.getLogger(DPPAgent.class);


	public DPPAgent(String name, Point start, Point goal,
			Environment environment, int agentBodyRadius, int maxTime, int waitMoveDuration, 
			Collection<tt.euclid2i.Region> sObst) {
		super(name, start, goal, environment, agentBodyRadius, maxTime, waitMoveDuration);
		this.sObst = sObst;
	}
	
    Map<String, CircleMovingToTarget> agentView =  new HashMap<String, CircleMovingToTarget>();
    
    
    final static boolean SOBST_KNOWN_AT_START = true;
    private Collection<tt.euclid2i.Region> sObst;

    boolean higherPriorityAgentsFinished = false;
    protected boolean globalTerminationDetected = false;

    List<String> sortedAgents = new LinkedList<String>();

	protected boolean agentViewDirty;
	
	public int infromNewTrajectorySentCounter = 0;
	
	static final int UNKNOWN = (-1);
	private int optimalSingleAgentPathDuration = UNKNOWN;
	private boolean succeeded;
		

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

	protected void broadcastNewTrajectory(EvaluatedTrajectory newTrajectory, int targetReachedTime) {
		this.infromNewTrajectorySentCounter++;
    	broadcast(new InformNewTrajectory(getName(), new CircleMovingToTarget(newTrajectory, agentBodyRadius, targetReachedTime)));
	}

    protected void broadcastAgentFinished() {
    	broadcast(new InformAgentFinished(getName()));
	}
    
    protected void broadcastFailure() {
    	broadcast(new InformAgentFailed(getName()));
	}
    
    
    protected void broadcastSuccessfulConvergence() {
    	broadcast(new InformSuccessfulConvergence());
	}

	protected boolean isHighestPriority() {
		return sortedAgents.get(0).equals(getName());
	}
	
	protected boolean isLowestPriority() {
		return sortedAgents.get(sortedAgents.size()-1).equals(getName());
	}
	
	protected Collection<tt.euclid2i.Region> sObst() {
		if (SOBST_KNOWN_AT_START) {
			return this.sObst;
		} else {		
			Collection<tt.euclid2i.Region> sObst = new LinkedList<tt.euclid2i.Region>();
	
	        for (Entry<String, CircleMovingToTarget> entry : agentView.entrySet()) {
	        	String name = entry.getKey();
	        	MovingCircle movingCircle = entry.getValue();
	
	        	if (getName().compareTo(name) < 0) {
	        		// Static obstacles
	        		sObst.add(new Circle(movingCircle.getTrajectory().get(0), movingCircle.getRadius()));
	        	}
	        }
	        
	        return sObst;
		}
	}
	
	protected  Collection<Region> dObst() {
		Collection<tt.euclidtime3i.Region> dObst = new LinkedList<tt.euclidtime3i.Region>();

        for (Entry<String, CircleMovingToTarget> entry : agentView.entrySet()) {
        	String name = entry.getKey();
        	CircleMovingToTarget movingCircle = entry.getValue();

        	if (getName().compareTo(name) > 0) {
        		// Dynamic obstacles
        		dObst.add(new CircleMovingToTarget(movingCircle.getTrajectory(), movingCircle.getRadius(), movingCircle.getTargetReachedTime() ));
        	} 
        }
        
        return dObst;
	}

	protected EvaluatedTrajectory assertConsistentTrajectory(EvaluatedTrajectory currentTraj, Collection<tt.euclid2i.Region> sObst, Collection<Region> dObst) {

		if (currentTraj == null || !consistent(new MovingCircle(currentTraj, agentBodyRadius), sObst, dObst)) {
    		// The current trajectory is inconsistent
			int currentMaxTime = computeMaxTime(dObst);
			LOGGER.trace(getName() + " detected inconsistency. Replanning with maxtime=" + currentMaxTime);
					
        	EvaluatedTrajectory newTrajectory = getBestResponseTrajectory(sObst, dObst, SOBST_KNOWN_AT_START ? null : getStart(), currentMaxTime);

        	if (newTrajectory != null) {
    	        // broadcast to the others
        		LOGGER.trace(getName() + " has a new trajectory. Cost: " + newTrajectory.getCost());
    	        broadcastNewTrajectory(newTrajectory, computeTargetReachedTime(newTrajectory, goal));
            	return newTrajectory;
        	} else {
        		LOGGER.debug(getName() + " Cannot find a consistent trajectory. Maxtime=" + currentMaxTime + ". dObst=" + dObst() );
        		
        		if (SOBST_KNOWN_AT_START) {
        			return null;
        		} else {
        			if (higherPriorityAgentsFinished) {
        				return null;
        			} else {
        				return currentTraj;
        			}
        		}
        	}

		} else {
			return currentTraj;
		}
    }

	private int computeMaxTime(Collection<Region> dObst) {
		
		return maxTime;
		
		// Temporarily disabled, as this is inconsistent with the regular timestep planning method...
		
//		if (optimalSingleAgentPathDuration == UNKNOWN && lowerPriorityAgentViewFull()) {
//			LOGGER.trace(getName() + " Computing the length of single-agent shortest path...");
//			optimalSingleAgentPathDuration = getSingleAgentShortestPath(sObst()).getMaxTime();
//			LOGGER.trace(getName() + " Duration of the shortest path is " + optimalSingleAgentPathDuration);
//		} 
//		
//		if (optimalSingleAgentPathDuration != UNKNOWN) {
//		
//			// in the worst-case, we will have to wait until the last reaches the goal
//			int latestReachesGoal = 0;
//			for (Region region : dObst) {
//				assert region instanceof CircleMovingToTarget;
//				int targetReached = ((CircleMovingToTarget) region).getTargetReachedTime(); 
//				if (latestReachesGoal < targetReached) {
//					latestReachesGoal = targetReached;
//				}
//			}
//			return latestReachesGoal + optimalSingleAgentPathDuration + waitMoveDuration;
//		} else {
//			return maxTime;
//		}
	}

	static private int computeTargetReachedTime(EvaluatedTrajectory traj, Point goal) {
    	
    	assert traj instanceof SegmentedTrajectory;
    	List<Straight> segmentsList = ((SegmentedTrajectory) traj).getSegments();
    	
    	Straight[] segments = segmentsList.toArray(new Straight[0]);
    	for (int i=segments.length-1; i >= 0; i--) {
    		if (segments[i].getEnd().getPosition().equals(goal) && !segments[i].getStart().getPosition().equals(goal)) {
    			return segments[i].getEnd().getTime();
    		}
    	}
    	
    	return 0;
	}

	protected boolean allStartRegionsOfLowerPriorityRobotsKnown() {
		if (SOBST_KNOWN_AT_START) {
			return true;
		} else {
	    	for (String otherAgentName : sortedAgents) {
	    		if (otherAgentName.compareTo(getName()) > 0) {
	    			if (!agentView.containsKey(otherAgentName)) {
	    				return false;
	    			}
	    		}
	    	}
	    	return true;
		}

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
            CircleMovingToTarget occupiedRegion = (CircleMovingToTarget) newTrajectoryMessage.getRegion();

            if (agentName.compareTo(getName()) < 0) {
            	// the messages is from higher-priority agent
            	agentView.put(agentName, occupiedRegion);
            	agentViewDirty = true;
            }

            if (agentName.compareTo(getName()) > 0 && !SOBST_KNOWN_AT_START) {
            	// the messages is from a lower-priority agent
            	if (!agentView.containsKey(agentName)) {
            		agentView.put(agentName, occupiedRegion);
            		agentViewDirty = true;
            	}
            }
        }
        
        if (message.getContent() instanceof InformSuccessfulConvergence) {
        	setGlobalTerminationDetected(true);
        }
        
        if (message.getContent() instanceof InformAgentFailed) {        	
        	setGlobalTerminationDetected(false);
        }
    }

	protected void setGlobalTerminationDetected(boolean succeeded) {
    	globalTerminationDetected = true;
    	this.succeeded = succeeded;
    	logFinalStats();
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
	public boolean isGlobalTerminationDetected() {
		return globalTerminationDetected;
	}

	@Override
	public boolean hasSucceeded() {
		return succeeded;
	}

	@Override
	public void tick(long time) {
		super.tick(time);
	}
	
	void logFinalStats() {
		LOGGER.info(getName() + ": New trajectory messages broadcasted: " + infromNewTrajectorySentCounter + "; Replanned: " + replanningCounter);
	}

	@Override
	public void start() {
		// TODO Auto-generated method stub
		
	}
	
	@Override
	public int getMessageSentCounter() {
		return this.infromNewTrajectorySentCounter;
	}
	
}

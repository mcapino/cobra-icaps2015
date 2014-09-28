package cz.agents.admap.agent;

import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import org.apache.log4j.Logger;
import org.jgrapht.DirectedGraph;
import org.jgrapht.util.HeuristicToGoal;

import cz.agents.admap.msg.InformAgentFailed;
import cz.agents.admap.msg.InformAgentFinished;
import cz.agents.admap.msg.InformSuccessfulConvergence;
import cz.agents.admap.msg.InformNewTrajectory;
import cz.agents.alite.communication.Communicator;
import cz.agents.alite.communication.Message;
import tt.euclid2i.EvaluatedTrajectory;
import tt.euclid2i.Point;
import tt.euclid2i.SegmentedTrajectory;
import tt.euclid2i.Trajectory;
import tt.euclid2i.probleminstance.Environment;
import tt.euclid2i.region.Circle;
import tt.euclid2i.trajectory.SegmentedTrajectories;
import tt.euclidtime3i.Region;
import tt.euclidtime3i.ShortestPathHeuristic;
import tt.euclidtime3i.discretization.Straight;
import tt.euclidtime3i.discretization.softconstraints.BumpSeparationPenaltyFunction;
import tt.euclidtime3i.discretization.softconstraints.PenaltyFunction;
import tt.euclidtime3i.region.CircleMovingToTarget;
import tt.euclidtime3i.region.MovingCircle;
import tt.euclidtime3i.util.IntersectionCheckerWithProtectedPoint;
import tt.jointtraj.separableflow.AStarTrajectoryOptimizer;
import tt.jointtraj.separableflow.PenalizedEvaluatedTrajectory;
import tt.jointtraj.separableflow.TrajectoryOptimizer;

public class ADPMAgent extends PlanningAgent {
	
	static final Logger LOGGER = Logger.getLogger(ADPMAgent.class);
	
	TrajectoryOptimizer trajectoryOptimizer;

	private int timeStep;
	private int constraintSamplingInterval;

	public ADPMAgent(String name, Point start, Point goal,
			Environment environment, int agentBodyRadius, int maxTime, int waitMoveDuration, 
			Collection<tt.euclid2i.Region> sObst) {
		super(name, start, goal, environment, agentBodyRadius, maxTime, waitMoveDuration);
		this.sObst = sObst;
		this.timeStep = waitMoveDuration;
		this.constraintSamplingInterval = waitMoveDuration / 10;
	}
	
	class AgentViewRecord {
				
		public AgentViewRecord(CircleMovingToTarget occupiedRegion,
				double weight) {
			super();
			this.occupiedRegion = occupiedRegion;
			this.weight = weight;
		}
		
		public CircleMovingToTarget occupiedRegion;
		public double weight;
	}
	
    Map<String, AgentViewRecord> agentView =  new HashMap<String, AgentViewRecord>();
    
    private Collection<tt.euclid2i.Region> sObst;

    boolean higherPriorityAgentsFinished = false;
    protected boolean globalTerminationDetected = false;
    
	PenalizedEvaluatedTrajectory trajectory;
    
    List<String> sortedAgents = new LinkedList<String>();

	protected boolean agentViewDirty;
	
	public int infromNewTrajectorySentCounter = 0;
	
	static final int UNKNOWN = (-1);

	private static final double EPS = 1;

	private static final double RADIUS_BUFFER_COEF = 1.2;
	
	private boolean agentFinished = false;
	private boolean succeeded;

    @Override
	public void setCommunicator(Communicator communicator, List<String> agents) {
		super.setCommunicator(communicator, agents);
		sortedAgents = new LinkedList<String>(agents);
		Collections.sort(sortedAgents);
	}

	@Override
    public PenalizedEvaluatedTrajectory getCurrentTrajectory() {
        return trajectory;
    }

	protected void broadcastNewTrajectory(EvaluatedTrajectory newTrajectory, int targetReachedTime, double penalty) {		
		this.infromNewTrajectorySentCounter++;
    	broadcast(new InformNewTrajectory(getName(), new CircleMovingToTarget(newTrajectory, agentBodyRadius, targetReachedTime), penalty));
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
		return this.sObst;
	}
	
	protected  Collection<MovingCircle> dObst() {
		Collection<MovingCircle> dObst = new LinkedList<MovingCircle>();

        for (Entry<String, AgentViewRecord> entry : agentView.entrySet()) {
        	if (!entry.getKey().equals(getName())) {
	        	CircleMovingToTarget movingCircle = entry.getValue().occupiedRegion;
	        	dObst.add(new CircleMovingToTarget(movingCircle.getTrajectory(), movingCircle.getRadius(), movingCircle.getTargetReachedTime() ));
        	}
        }
        
        return dObst;
	}

	protected PenalizedEvaluatedTrajectory assertOptimalTrajectory(PenalizedEvaluatedTrajectory currentTraj, Collection<tt.euclid2i.Region> sObst, Collection<MovingCircle> dObst, double penaltyWeight) {
	
		double currentPenalty = getPenalty(1);
		if (currentTraj == null || currentPenalty > 0) {
    		// The current trajectory is inconsistent
			int currentMaxTime = computeMaxTime(dObst);
			LOGGER.trace(getName() + ": Current penalty p=" + d4(currentPenalty) +  ", w*p=" + d4(penaltyWeight * currentPenalty) +"  Replanning with w=" + d4(penaltyWeight) + " maxtime=" + currentMaxTime);
			
			if (trajectoryOptimizer == null) {
				// Initialize trajectory optimizer if it has not been used yet
				HeuristicToGoal<tt.euclidtime3i.Point> heuristic = new ShortestPathHeuristic(planningGraph, goal);
	            DirectedGraph<tt.euclidtime3i.Point, Straight> motions = BestResponse.createMotions(planningGraph, goal, dObst, maxTime, timeStep);
            		
				
	            trajectoryOptimizer = new AStarTrajectoryOptimizer(motions, 
						new tt.euclidtime3i.Point(start, 0), new tt.euclidtime3i.Point(goal, maxTime), 
						heuristic, timeStep, constraintSamplingInterval);
			}
					
        	PenaltyFunction[] penaltyFunctions = new PenaltyFunction[dObst.size()];
        	Trajectory[] otherTrajectories = new Trajectory[dObst.size()];
        	
        	int i = 0;
        	for (MovingCircle movingCircle : dObst) {
        		penaltyFunctions[i] = new BumpSeparationPenaltyFunction(penaltyWeight, (movingCircle.getRadius() + agentBodyRadius) * RADIUS_BUFFER_COEF, 1);
        		otherTrajectories[i] = movingCircle.getTrajectory();
        		i++;
        	}
        				
			PenalizedEvaluatedTrajectory newTrajectory = trajectoryOptimizer.getOptimalTrajectoryConstrained(penaltyFunctions, otherTrajectories, currentTraj, Double.POSITIVE_INFINITY, Long.MAX_VALUE);
        	
        	if (newTrajectory != null) {
    	        // broadcast to the others
        		LOGGER.trace(getName() + " has a new trajectory. Cost: " + d4(newTrajectory.getCost()) + " from which is penalty w*p=" + d4(newTrajectory.getPenalty()));
    	        broadcastNewTrajectory(newTrajectory.getTrajectory(), computeTargetReachedTime(newTrajectory, goal), penaltyWeight);
            	return newTrajectory;
        	} else {
        		LOGGER.debug(getName() + " Cannot find a consistent trajectory. Maxtime=" + currentMaxTime + ". dObst=" + dObst() );
      			return null;
        	}

		} else {
			return currentTraj;
		}
    }

	private int computeMaxTime(Collection<MovingCircle> dObst) {
		return maxTime;
	}

	static private int computeTargetReachedTime(PenalizedEvaluatedTrajectory traj, Point goal) {
    	assert traj.getTrajectory() instanceof SegmentedTrajectory;
    	List<Straight> segmentsList = ((SegmentedTrajectory) traj.getTrajectory()).getSegments();
    	
    	Straight[] segments = segmentsList.toArray(new Straight[0]);
    	for (int i=segments.length-1; i >= 0; i--) {
    		if (segments[i].getEnd().getPosition().equals(goal) && !segments[i].getStart().getPosition().equals(goal)) {
    			return segments[i].getEnd().getTime();
    		}
    	}
    	
    	return 0;
	}

	protected boolean consistent(MovingCircle movingCircle, Collection<tt.euclid2i.Region> sObst, Collection<MovingCircle> dObst) {

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
            double weight = newTrajectoryMessage.getWeight();

           	agentView.put(agentName, new AgentViewRecord(occupiedRegion, weight));
           	agentViewDirty = true;
        }
        
        if (message.getContent() instanceof InformSuccessfulConvergence) {
        	setGlobalTerminationDetected(true);
        }
        
        if (message.getContent() instanceof InformAgentFailed) {        	
        	setGlobalTerminationDetected(false);
        }
        
        if (message.getContent() instanceof InformAgentFinished) {
        	String agentName = ((InformAgentFinished) message.getContent()).getAgentName();
        	if (isMyPredecessor(agentName)) {
        		higherPriorityAgentsFinished = true;
        		agentViewDirty = true;
        	}
        }
		
        if (agentViewDirty && getInboxSize() == 0 && !agentFinished) {
        	assertOptimalTrajectory();
        	agentViewDirty = false;
        	
        	if (isLowestPriority() && higherPriorityAgentsFinished && getCurrentTrajectory().getPenalty() == 0) {
        		broadcastSuccessfulConvergence();
        		LOGGER.info(getName() +  " globally terminated!");
        		setGlobalTerminationDetected(true);
        	}
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
    	if (isHighestPriority()) {
    		higherPriorityAgentsFinished = true;
    		agentViewDirty = false;
    	}

    	assertOptimalTrajectory();
    	
    	if (isLowestPriority() && higherPriorityAgentsFinished && trajectory != null) {
    		setGlobalTerminationDetected(true);
    	}
    }
	
	@Override
	public int getMessageSentCounter() {
		return this.infromNewTrajectorySentCounter;
	}
	
	protected void assertOptimalTrajectory() {
		if (getCurrentTrajectory() == null) {
	    	trajectory = assertOptimalTrajectory(getCurrentTrajectory(), sObst(), Collections.<MovingCircle> emptySet(), 0);
		} else {
	        trajectory = assertOptimalTrajectory(getCurrentTrajectory(), sObst(), dObst(), getAgentViewMaxWeight() + EPS);
		}
		
//		if (trajectory.getPenalty() < 0.01) {
//			// trajectory found
//	    	if (!agentFinished && higherPriorityAgentsFinished) {
//	    		// we have consistent trajectory and the higher-priority agents are fixed
//	    		agentFinished = true;
//	    		broadcastAgentFinished();
//	    		LOGGER.info(getName() +  " has finished!");
//	    	}
//		}
	}

	private double getAgentViewMaxWeight() {
		double maxWeight = 0;
		for (AgentViewRecord record : agentView.values()) {
			if (record.weight > maxWeight) {
				maxWeight = record.weight; 
			}
		}
		return maxWeight;
	}	
	
	public double getPenalty(double penaltyWeight) {
		if (trajectory == null) {
			return Double.POSITIVE_INFINITY;
		} else {
			Collection<MovingCircle> dObst = dObst();
	    	PenaltyFunction[] penaltyFunctions = new PenaltyFunction[dObst.size()];
	    	Trajectory[] otherTrajectories = new Trajectory[dObst.size()];
	    	
	    	int i = 0;
	    	for (MovingCircle movingCircle : dObst) {
	    		penaltyFunctions[i] = new BumpSeparationPenaltyFunction(penaltyWeight, (movingCircle.getRadius() + agentBodyRadius) * RADIUS_BUFFER_COEF, 1);
	    		otherTrajectories[i] = movingCircle.getTrajectory();
	    		i++;
	    	}
	    	
	    	assert trajectory.getTrajectory() instanceof SegmentedTrajectory;
	    	return AStarTrajectoryOptimizer.computePenaltyForTrajectory((SegmentedTrajectory)trajectory.getTrajectory(), penaltyFunctions, otherTrajectories, constraintSamplingInterval);
		}
	}
	
	public static String d4(double num) {
		return String.format("%.4f", num);
	}
}

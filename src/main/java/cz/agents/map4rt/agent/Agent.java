package cz.agents.map4rt.agent;

import java.util.Collection;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;
import java.util.Random;

import org.apache.log4j.Logger;
import org.jgrapht.DirectedGraph;
import org.jgrapht.GraphPath;
import org.jgrapht.alg.AStarShortestPathSimple;
import org.jgrapht.util.HeuristicToGoal;

import tt.euclid2i.EvaluatedTrajectory;
import tt.euclid2i.Line;
import tt.euclid2i.Point;
import tt.euclid2i.Region;
import tt.euclid2i.discretization.AdditionalPointsExtension;
import tt.euclid2i.probleminstance.Environment;
import cz.agents.alite.communication.Communicator;
import cz.agents.alite.communication.InboxBasedCommunicator;
import cz.agents.alite.communication.Message;
import cz.agents.alite.communication.MessageHandler;
import cz.agents.alite.communication.content.Content;
import cz.agents.map4rt.CommonTime;


public abstract class Agent {

    static final Logger LOGGER = Logger.getLogger(Agent.class);
    
    String name;
    Point start;
    int nTasks;

    Environment environment;

    int agentBodyRadius;

    Communicator communicator;
    List<String> agents;
    float maxSpeed;

    Collection<Region> inflatedObstacles;
    DirectedGraph<Point, Line> planningGraph;
    
    Point currentTask = null;
    Random random;
    
    long lastTaskIssuedAt;
    long lastTaskReachedAtMs;    
    long issueFirstTaskAt = 0;

	protected long currentTaskBaseDuration;

	public long baseSum;
	public long baseSumSq;
	
	public long waitSum;
	public long waitSumSq;	
	public long planSum;
	public long planSumSq;
	public long pWindowSum;
	public long pWindowSumSq;	
	
	public long prolongTSum;
	public long prolongTSumSq;	
	public long prolongRSum;
	public long prolongRSumSq;

	private double maxEdgeLength;
	
	public Agent(String name, Point start, int nTasks, Environment environment, DirectedGraph<Point, Line> planningGraph, int agentBodyRadius, float maxSpeed, Random random) {
        super();
        this.name = name;
        this.start = start;
        this.nTasks = nTasks;
        this.random = random;
        this.environment = environment;
        this.agentBodyRadius = agentBodyRadius;
        this.inflatedObstacles = tt.euclid2i.util.Util.inflateRegions(environment.getObstacles(), agentBodyRadius);
        this.inflatedObstacles.addAll(tt.euclid2i.util.Util.inflateRegions(Collections.singleton(environment.getBoundary()), agentBodyRadius));
        this.maxSpeed = maxSpeed;
        this.planningGraph = planningGraph;
        
        maxEdgeLength = 0;
        for (Line edge : getPlanningGraph().edgeSet()) {
        	if (edge.getDistance() > maxEdgeLength) {
        		maxEdgeLength = edge.getDistance();
        	}
        }
        
        
        assert CurrentTasks.tryToRegisterTask(name, start);
    }

    public synchronized Point getStart() {
        return start;
    }

    public String getName() {
        return name;
    }
    
    public int getAgentBodyRadius() {
		return agentBodyRadius;
	}

    public abstract EvaluatedTrajectory getCurrentTrajectory();

    public tt.euclidtime3i.Region getOccupiedRegion() {
        if (getCurrentTrajectory() != null) {
            return new tt.euclidtime3i.region.MovingCircle(getCurrentTrajectory(), agentBodyRadius);
        } else {
            return null;
        }
    }

    public void setCommunicator(Communicator communicator, List<String> agents) {
        this.communicator = communicator;
        this.agents = agents;
        this.communicator.addMessageHandler(new MessageHandler() {
            @Override
            public void notify(Message message) {
                Agent.this.notify(message);
            }
        });
    }

    protected Communicator getCommunicator() {
        return communicator;
    }

    public abstract void start();

    // messaging

    protected void broadcast(Content content) {
        Message msg = getCommunicator().createMessage(content);
        LinkedList<String> receivers = new LinkedList<String>(agents);
        receivers.remove(getName());
        msg.addReceivers(receivers);
        getCommunicator().sendMessage(msg);
    }

    protected void send(String receiver, Content content) {
        Message msg = getCommunicator().createMessage(content);
        LinkedList<String> receivers = new LinkedList<String>();
        receivers.add(receiver);
        msg.addReceivers(receivers);
        getCommunicator().sendMessage(msg);
    }

    protected void notify(Message message) {
    	//LOGGER.trace(getName() + " >>> received message " + message.getContent());
    }

    public void tick(int timeMs) {
    	//LOGGER.info(getName() + " Tick @ " + time/1000.0 + "s");
    	
    	if (currentTask == null && CommonTime.currentTimeMs() > issueFirstTaskAt && nTasks > 0) {
    		lastTaskIssuedAt = CommonTime.currentTimeMs();
    		synchronized (Agent.class) {
    			long waitDuration = CommonTime.currentTimeMs() - lastTaskIssuedAt;
    			waitSum += waitDuration;
    			waitSumSq += waitDuration * waitDuration;
    			
    			currentTask = CurrentTasks.assignRandomDestination(getName(), random);
    			currentTaskBaseDuration = getTaskDuration(getCurrentPos(), currentTask);
    			baseSum += currentTaskBaseDuration;
    			baseSumSq += currentTaskBaseDuration * currentTaskBaseDuration;
    			
    			LOGGER.info(getName() + " Carrying out new task " + currentTask + ", baseline duration is " + currentTaskBaseDuration + ". There is " + nTasks + " tasks in the stack to be carried out.");
    			handleNewTask(currentTask);
    			nTasks--;
    		}
    	} 
    	
    	if (currentTask != null && currentTaskDestinationReached()) {
			if (nTasks == 0) {
    			LOGGER.info(getName() + " finished all tasks");
    			lastTaskReachedAtMs = CommonTime.currentTimeMs();
    		}
			currentTask = null;
    	}
    }
    
    protected abstract boolean currentTaskDestinationReached();

	/**
     * @return true if the task has been handled, false if the task could not been handled at this point
     */
    protected abstract void handleNewTask(Point task);
    
	public String getStatus() { return getName(); }

    public DirectedGraph<Point, Line> getPlanningGraph() {
    	return planningGraph;
    }

    public abstract Point getCurrentPos();
    
    public boolean hasCompletedAllTasks() {
    	return currentTask==null && nTasks == 0;
    }

	public int getMessageSentCounter() {
		return ((InboxBasedCommunicator) communicator).getMessagesSent();
	}
	
	public int getInboxSize() {
		return ((InboxBasedCommunicator) communicator).getInboxSize();
	}

	public long getLastTaskReachedTime() {
		return lastTaskReachedAtMs;
	}
	
	public void setIssueFirstTaskAt(long issueFirstTaskAt) {
		this.issueFirstTaskAt = issueFirstTaskAt;
	}
	
	protected long getTaskDuration(final Point startPoint, final Point goal) {
		
		HeuristicToGoal<Point> heuristic = new HeuristicToGoal<Point>() {
			@Override
			public double getCostToGoalEstimate(Point current) {
				return current.distance(goal);
			}
		};
		
		AdditionalPointsExtension graph = new AdditionalPointsExtension(getPlanningGraph(), Collections.singleton(startPoint), (int) Math.ceil(maxEdgeLength));
		
        GraphPath<Point, Line> path = AStarShortestPathSimple.findPathBetween(graph,
                heuristic,
                startPoint,
                goal);
        
        return (long) (path.getWeight()/maxSpeed);
	}
	

}

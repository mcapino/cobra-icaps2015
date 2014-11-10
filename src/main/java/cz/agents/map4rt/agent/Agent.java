package cz.agents.map4rt.agent;

import java.util.Collection;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;
import java.util.Random;

import org.apache.log4j.Logger;
import org.jgrapht.DirectedGraph;

import tt.euclid2i.EvaluatedTrajectory;
import tt.euclid2i.Line;
import tt.euclid2i.Point;
import tt.euclid2i.Region;
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
    
    long lastTickAtMs;

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
    	lastTickAtMs = CommonTime.currentTimeMs();
    	
    	if (currentTask == null) {
    		synchronized (Agent.class) {
	    		if (nTasks > 0) {
	    			currentTask = CurrentTasks.assignRandomDestination(getName(), random);
	    			LOGGER.info(getName() + " Carrying out new task " + currentTask + ". There is " + nTasks + " tasks in the stack to be carried out.");
	    			handleNewTask(currentTask);
	    			nTasks--;
	    		} 
    		}
    	} else if (currentTaskDestinationReached()) {
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

	public long getLastTickAtMs() {
		return lastTickAtMs;
	}
}

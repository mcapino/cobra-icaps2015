package cz.agents.map4rt.agent;

import java.util.Collection;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

import org.apache.log4j.Logger;
import org.jgrapht.DirectedGraph;

import tt.euclid2i.EvaluatedTrajectory;
import tt.euclid2i.Line;
import tt.euclid2i.Point;
import tt.euclid2i.Region;
import tt.euclid2i.probleminstance.Environment;
import tt.jointeuclid2ni.probleminstance.RelocationTask;
import cz.agents.alite.communication.Communicator;
import cz.agents.alite.communication.InboxBasedCommunicator;
import cz.agents.alite.communication.Message;
import cz.agents.alite.communication.MessageHandler;
import cz.agents.alite.communication.content.Content;


public abstract class Agent {

    static final Logger LOGGER = Logger.getLogger(Agent.class);

    String name;
    Point start;
    List<RelocationTask> tasks;

    Environment environment;

    int agentBodyRadius;

    Communicator communicator;
    List<String> agents;
    float maxSpeed;

    Collection<Region> inflatedObstacles;
    DirectedGraph<Point, Line> planningGraph;
    
    RelocationTask currentTask = null;
    int time = 0;


	public Agent(String name, Point start, List<RelocationTask> tasks, Environment environment, DirectedGraph<Point, Line> planningGraph, int agentBodyRadius, float maxSpeed) {
        super();
        this.name = name;
        this.start = start;
        this.tasks = tasks;
        this.environment = environment;
        this.agentBodyRadius = agentBodyRadius;
        this.inflatedObstacles = tt.euclid2i.util.Util.inflateRegions(environment.getObstacles(), agentBodyRadius);
        this.inflatedObstacles.addAll(tt.euclid2i.util.Util.inflateRegions(Collections.singleton(environment.getBoundary()), agentBodyRadius));
        this.maxSpeed = maxSpeed;
        this.planningGraph = planningGraph;
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
        LOGGER.trace(getName() + " >>> received message " + message.getContent());
    }

    public void tick(int timeMs) {
    	LOGGER.info(getName() + " Tick @ " + time/1000.0 + "s");
    	
    	time = timeMs;
    	
    	if (currentTask == null) {
    		if (!tasks.isEmpty() && tasks.get(0).getIssueTime() < timeMs) {
    			currentTask = tasks.get(0);
    			LOGGER.info(getName() + " Carrying out new task " + currentTask);
    			handleNewTask(currentTask);
    			tasks.remove(0);
    		} 
    	} else if (currentTask.getDestination().equals(getCurrentPos())) {
    		currentTask = null;
    	}
    }
    
    /**
     * @return true if the task has been handled, false if the task could not been handled at this point
     */
    protected abstract void handleNewTask(RelocationTask task);
    
	public String getStatus() { return getName(); }

    public DirectedGraph<Point, Line> getPlanningGraph() {
    	return planningGraph;
    }

    public abstract Point getCurrentPos();
    
    public boolean hasCompletedAllTasks() {
    	return currentTask == null & tasks.isEmpty();
    }

	public int getMessageSentCounter() {
		return ((InboxBasedCommunicator) communicator).getMessagesSent();
	}
	
	public int getInboxSize() {
		return ((InboxBasedCommunicator) communicator).getInboxSize();
	}
}

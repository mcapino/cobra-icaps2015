package cz.agents.admap.agent;

import java.util.Collection;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

import org.apache.log4j.Logger;
import org.jgrapht.DirectedGraph;

import tt.euclid2i.EvaluatedTrajectory;
import tt.euclid2i.Point;
import tt.euclid2i.Line;
import tt.euclid2i.Region;
import tt.euclid2i.probleminstance.Environment;
import cz.agents.alite.communication.Communicator;
import cz.agents.alite.communication.Message;
import cz.agents.alite.communication.MessageHandler;
import cz.agents.alite.communication.content.Content;


public abstract class Agent {

    static final Logger LOGGER = Logger.getLogger(Agent.class);

    String name;
    Point start;
    Point goal;

    Point currentPosition;

    Environment environment;

    int agentBodyRadius;

    EvaluatedTrajectory trajectory;

    Communicator communicator;
    List<String> agents;

    Collection<Region> inflatedObstacles;
    DirectedGraph<Point, Line> planningGraph;



	public Agent(String name, Point start, Point goal, Environment environment, int agentBodyRadius) {
        super();
        this.name = name;
        this.start = start;
        this.goal = goal;
        this.environment = environment;
        this.agentBodyRadius = agentBodyRadius;
        this.inflatedObstacles = tt.euclid2i.util.Util.inflateRegions(environment.getObstacles(), agentBodyRadius);
        this.inflatedObstacles.addAll(tt.euclid2i.util.Util.inflateRegions(Collections.singleton(environment.getBoundary()), agentBodyRadius));
    }

    public synchronized Point getStart() {
        return start;
    }

    public synchronized Point getGoal() {
        return goal;
    }

    public synchronized Point getCurrentPosition() {
        return currentPosition;
    }

    public synchronized void setCurrentPosition(Point currentPosition) {
		this.currentPosition = currentPosition;
	}

    public String getName() {
        return name;
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
        LOGGER.trace(getName() + " received " + message.getContent());
    }

    public void tick(long time) {

    }

    public String getStatus() { return getName(); }

    public DirectedGraph<Point, Line> getPlanningGraph() {
    	return planningGraph;
    }

    public void setPlanningGraph(DirectedGraph<Point, Line> planningGraph) {
    	this.planningGraph = planningGraph;
    }
}

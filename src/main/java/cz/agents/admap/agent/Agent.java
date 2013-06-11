package cz.agents.admap.agent;

import java.util.LinkedList;
import java.util.List;

import org.apache.log4j.Logger;

import tt.euclid2i.EvaluatedTrajectory;
import tt.euclid2i.Point;
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

    Environment environment;
    int agentSizeRadius;

    EvaluatedTrajectory trajectory;

    Communicator communicator;
    List<String> agents;

    public Agent(String name, Point start, Point goal, Environment environment, int agentSizeRadius) {
        super();
        this.name = name;
        this.start = start;
        this.goal = goal;
        this.environment = environment;
        this.agentSizeRadius = agentSizeRadius;
    }

    public synchronized Point getStart() {
        return start;
    }

    public synchronized Point getGoal() {
        return goal;
    }

    public String getName() {
        return name;
    }

    public abstract EvaluatedTrajectory getCurrentTrajectory();

    public tt.euclidtime3i.Region getOccupiedRegion() {
        return new tt.euclidtime3i.region.MovingCircle(getCurrentTrajectory(), agentSizeRadius);
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
        LOGGER.debug(getName() + " received " + message.getContent());
    }

    public void tick(long time) {
        LOGGER.debug(getName() + " tick: " + time);
//        try {
//            Thread.sleep(1000);
//        } catch (InterruptedException e) {}
    }

}

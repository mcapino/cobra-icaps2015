package cz.agents.admap.agent;

import java.util.LinkedList;
import java.util.List;

import tt.euclid2i.EvaluatedTrajectory;
import tt.euclid2i.Point;
import tt.euclid2i.probleminstance.Environment;
import cz.agents.alite.communication.Communicator;
import cz.agents.alite.communication.Message;
import cz.agents.alite.communication.MessageHandler;
import cz.agents.alite.communication.content.Content;


public abstract class Agent {

    String name;
    Point start;
    Point goal;

    Environment environment;
    int agentSizeRadius;

    EvaluatedTrajectory trajectory;

    // messaging
    Communicator communicator;
    List<String> agents;
    MessageHandler messageHandler;

    public Agent(String name, Point start, Point goal, Environment environment, int agentSizeRadius) {
        super();
        this.name = name;
        this.start = start;
        this.goal = goal;
        this.environment = environment;
        this.agentSizeRadius = agentSizeRadius;
        this.messageHandler = new MessageHandler() {
            @Override
            public void notify(Message message) {
                Agent.this.notify(message);
            }
        };
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

    public void setCommunicator(Communicator communicator, List<String> agents) {
        this.communicator = communicator;
        this.agents = agents;
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

    protected void notify(Message message) {};

}

package cz.agents.admap.agent;

import java.util.List;

import tt.euclid2i.EvaluatedTrajectory;
import tt.euclid2i.Point;
import tt.euclid2i.probleminstance.Environment;
import cz.agents.alite.communication.Communicator;

public abstract class Agent {

    String name;
    Point start;
    Point goal;
    Communicator communicator;
    List<String> agents;
    Environment environment;
    int agentSizeRadius;
    EvaluatedTrajectory trajectory;

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

    public void setCommunicator(Communicator communicator, List<String> agents) {
        this.communicator = communicator;
        this.agents = agents;
    }

    protected Communicator getCommunicator() {
        return communicator;
    }

    public void start() {
    }
}

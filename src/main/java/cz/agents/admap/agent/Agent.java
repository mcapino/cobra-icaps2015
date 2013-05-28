package cz.agents.admap.agent;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import tt.euclid2i.Point;
import tt.euclid2i.Region;
import tt.euclid2i.Trajectory;
import tt.euclid2i.probleminstance.Environment;
import cz.agents.alite.communication.Communicator;

public class Agent {

    String name;
    Map<String, Objectives> group = new HashMap<String, Objectives>();
    Map<String, Trajectory> trajectories =  new HashMap<String, Trajectory>();
    Map<String, Trajectory> avoids =  new HashMap<String, Trajectory>();
    Environment environment;

    protected Communicator communicator;
    protected List<String> agents;

    public Agent(String name, Point start, Region goal, Environment environment) {
        this.name = name;
        this.group.put(name, new Objectives(start, goal));
        this.environment = environment ;
    }

    public synchronized Point getStart() {
        return group.get(name).start;
    }

    public synchronized Region getGoal() {
        return group.get(name).goal;
    }

    public String getName() {
        return name;
    }

    public Trajectory getCurrentTrajectory() {
        return trajectories.get(name);
    }

    public void setCommunicator(Communicator communicator, List<String> agents) {
        this.communicator = communicator;
        this.agents = agents;
    }

    protected Communicator getCommunicator() {
        return communicator;
    }

    public void start() {
        replan();
    }

    private void replan() {


    }



}

package cz.agents.admap.agent;

import java.util.HashMap;
import java.util.Map;

import tt.euclid2i.Point;
import tt.euclid2i.Trajectory;
import tt.euclid2i.probleminstance.Environment;
import tt.euclid2i.region.Region;

public class Agent {

    String name;
    Map<String, Objectives> group = new HashMap<String, Objectives>();
    Map<String, Trajectory> trajectories =  new HashMap<String, Trajectory>();
    Map<String, Trajectory> avoids =  new HashMap<String, Trajectory>();
    Environment environment;

    public Agent(String name, Point start, Region goal, Environment environment) {
        this.name = name;
        this.group.put(name, new Objectives(start, goal));
        this.environment= environment ;
    }

    public synchronized Point getStart() {
        return group.get(name).start;
    }

    public synchronized Region getGoal() {
        return group.get(name).goal;
    }

}

package cz.agents.admap.agent;

import cz.agents.alite.common.entity.Entity;
import cz.agents.alite.trajectorytools.trajectory.EvaluatedTrajectory;
import cz.agents.alite.trajectorytools.util.OrientedPoint;

public abstract class Agent extends Entity implements Comparable<Agent>{

    protected final OrientedPoint start;
    protected final double startTime;
    protected final OrientedPoint destination;


    public Agent(String name, OrientedPoint start, double startTime,  OrientedPoint destination) {
        super(name);
        this.start = start;
        this.startTime = startTime;
        this.destination = destination;
    }

    public abstract EvaluatedTrajectory getCurrentTrajectory();

    public synchronized OrientedPoint getStart() {
        return start;
    }

    public double getStartTime() {
        return startTime;
    }

    public synchronized OrientedPoint getDestination() {
        return destination;
    }

    @Override
    public int compareTo(Agent o) {
        return this.getName().compareTo(o.getName());
    }

    public abstract void start() ;
}

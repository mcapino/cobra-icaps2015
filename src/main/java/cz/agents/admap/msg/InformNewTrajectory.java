package cz.agents.admap.msg;

import tt.euclid2i.Trajectory;
import cz.agents.alite.communication.content.Content;

public class InformNewTrajectory extends Content {
    final String agentName;
    final double priority;
    final Trajectory trajectory;


    public InformNewTrajectory(String agentName, double priority, Trajectory t) {
        super(t);

        this.agentName = agentName;
        this.priority = priority;
        this.trajectory = t;
    }

    public synchronized String getAgentName() {
        return agentName;
    }

    public synchronized Trajectory getTrajectory() {
        return trajectory;
    }

    public double getPriority() {
        return priority;
    }

    @Override
    public String toString() {
        return "InformNewTrajectory [agentName=" + agentName + ", priority="
                + priority + ", trajectory=" + trajectory + "]";
    }



}

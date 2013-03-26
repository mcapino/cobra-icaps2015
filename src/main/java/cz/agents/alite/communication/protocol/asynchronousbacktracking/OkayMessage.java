package cz.agents.alite.communication.protocol.asynchronousbacktracking;

import cz.agents.alite.trajectorytools.trajectory.Trajectory;

public class OkayMessage {
    String senderAgent;
    Trajectory trajectory;

    public OkayMessage(String agent, Trajectory trajectory) {
        this.senderAgent = agent;
        this.trajectory = trajectory;
    }

    public String getSenderAgent() {
        return senderAgent;
    }

    public Trajectory getTrajectory() {
        return trajectory;
    }
}

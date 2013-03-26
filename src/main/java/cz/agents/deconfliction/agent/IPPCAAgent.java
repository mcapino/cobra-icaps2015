package cz.agents.deconfliction.agent;

import cz.agents.alite.trajectorytools.trajectory.Trajectory;
import cz.agents.alite.trajectorytools.util.OrientedPoint;

public class IPPCAAgent extends CommunicatingAgent {

    public IPPCAAgent(String name, OrientedPoint start,
            OrientedPoint destination, double speed) {
        super(name, start, 0.0, destination);
    }

    @Override
    public Trajectory getCurrentTrajectory() {
        return null;
    }

    @Override
    public void start() {

    }

}

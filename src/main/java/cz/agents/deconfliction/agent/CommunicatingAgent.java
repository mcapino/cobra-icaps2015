package cz.agents.deconfliction.agent;

import java.util.List;

import cz.agents.alite.communication.Communicator;
import cz.agents.alite.trajectorytools.trajectory.Trajectory;
import cz.agents.alite.trajectorytools.util.OrientedPoint;

public abstract class CommunicatingAgent extends Agent {

    protected Communicator communicator;
    protected List<String> agents;
    
    

    public CommunicatingAgent(String name, OrientedPoint startPoint, double startTime, OrientedPoint destination) {
        super(name, startPoint, startTime,  destination);
    }

    public abstract Trajectory getCurrentTrajectory();

    public void setCommunicator(Communicator communicator, List<String> agents) {
        this.communicator = communicator;
        this.agents = agents;
    }

    protected Communicator getCommunicator() {
        return communicator;
    }
}

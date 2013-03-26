package cz.agents.deconfliction.agent;

import cz.agents.alite.trajectorytools.trajectory.EvaluatedTrajectory;
import cz.agents.alite.trajectorytools.util.OrientedPoint;

public class FixedTrajectoryAgent extends Agent {

    EvaluatedTrajectory trajectory;

    public FixedTrajectoryAgent(String name, OrientedPoint start,
            double startTime, OrientedPoint destination, EvaluatedTrajectory trajectory) {
        super(name, start, startTime, destination);
        this.trajectory = trajectory;
    }

    @Override
    public EvaluatedTrajectory getCurrentTrajectory() {
        return trajectory;
    }

    @Override
    public void start() {}

}

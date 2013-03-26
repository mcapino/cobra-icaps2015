package cz.agents.deconfliction.trajectory;

public interface TrajectoryGenerator {
    Trajectory nextTrajectory();
    void reset();
}

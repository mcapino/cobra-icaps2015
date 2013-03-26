package cz.agents.deconfliction.agent;

import java.util.LinkedList;
import java.util.List;

import org.jgrapht.Graph;

import cz.agents.alite.trajectorytools.graph.spatial.maneuvers.SpatialManeuver;
import cz.agents.alite.trajectorytools.trajectory.Trajectory;
import cz.agents.alite.trajectorytools.util.OrientedPoint;
import cz.agents.alite.trajectorytools.util.SeparationDetector;
import cz.agents.alite.trajectorytools.util.Waypoint;
import cz.agents.deconfliction.solver.dpp.InformNewTrajectory;
import cz.agents.deconfliction.trajectory.CandidateSolution;

public class AsynchronousDecentralizedPrioritizedPlanningAgent extends
        DecentralizedPrioritizedPlanningAgent {

    protected CandidateSolution deltaAgentView;

    List<Trajectory> trajectoryInbox = new LinkedList<Trajectory>();

    public AsynchronousDecentralizedPrioritizedPlanningAgent(String name,
            Graph<Waypoint, SpatialManeuver> maneuvers, double startTime,
            OrientedPoint startPoint, OrientedPoint destination,
            double separation, double maxTime, double approxSamplingInterval, double vmax,
            ReplanningStrategy replanningStrategy) {
        super(name, maneuvers, startTime, startPoint, destination, separation,
                maxTime, vmax, approxSamplingInterval, replanningStrategy);

        this.deltaAgentView = new CandidateSolution();
    }

    protected void recordAgentViewDelta(InformNewTrajectory message) {
        if (message.getPriority() < priorities.get(getName())) {
            deltaAgentView.set(message.getAgentName(), message.getTrajectory());
        }
    }

    protected void recordAgentViewDelta(String myName, Trajectory trajectory) {
        deltaAgentView.set(myName, trajectory);
    }

    protected void commitDeltaAgentView() {
        for (String agentName : deltaAgentView.getAgentNames()) {
            if (!agentName.equals(getName())) {
                agentView.set(agentName, deltaAgentView.get(agentName));
            }
        }
        deltaAgentView = new CandidateSolution();
        deltaAgentView.set(getName(), getCurrentTrajectory());
    }

    @Override
    public void start() {

    }

    protected boolean areConflicting(Trajectory t1, Trajectory t2) {
        long startNanos = System.nanoTime();

        LinkedList<Trajectory> otherTrajectories = new LinkedList<Trajectory>();
        otherTrajectories.add(t2);

        boolean result = SeparationDetector.hasConflict(t1, otherTrajectories, separation);
        this.conflictCheckingCumulativeRuntime += System.nanoTime() - startNanos;
        return result;
    }

}

package cz.agents.deconfliction.agent;

import org.apache.log4j.Logger;

import cz.agents.alite.communication.Message;
import org.jgrapht.Graph;
import cz.agents.deconfliction.solver.NoSolutionFoundException;
import cz.agents.deconfliction.solver.dpp.InformNewTrajectory;
import cz.agents.alite.trajectorytools.trajectory.Trajectory;
import cz.agents.alite.trajectorytools.util.OrientedPoint;

public class BlockingAsynchronousPrioritizedPlanningAgent extends
        AsynchronousDecentralizedPrioritizedPlanningAgent {

    Logger LOGGER = Logger
            .getLogger(BlockingAsynchronousPrioritizedPlanningAgent.class);

    public BlockingAsynchronousPrioritizedPlanningAgent(String name,
            Graph<Waypoint, SpatialManeuver> maneuvers, double startTime,
            OrientedPoint startPoint, OrientedPoint destination,
            double separation, double maxTime, double approxSamplingInterval,
            ReplanningStrategy replanningStrategy) {
        super(name, maneuvers, startTime, startPoint, destination, separation,
                maxTime, approxSamplingInterval, replanningStrategy);
    }

    @Override
    public void start() {
        Trajectory newTrajectory = getBestResponse(getName(),
                startWaypoint, destWaypoint, startTime, maneuvers,
                agentView, priorities, separation, maxTime,
                approxSamplingInterval);


        broadcast(new InformNewTrajectory(getName(),
                priorities.get(getName()), newTrajectory));
        agentView.set(getName(), newTrajectory);
        recordAgentViewDelta(getName(), newTrajectory);

    }


    @Override
    public void notify(Message message) {
        super.notify(message);
    }




    @Override
    protected void afterInfoMessageCheck(InformNewTrajectory inform) {

        recordAgentViewDelta(inform);

        if (inform.getPriority() < priorities.get(getName())) {
            trajectoryInbox.add(inform.getTrajectory());
        }

        if (getInboxSize() > 0 || trajectoryInbox.isEmpty() ) {
            return;
        }


        checkDeltaConsistency();
        trajectoryInbox.clear();
     }

    protected void checkDeltaConsistency() {
        if (!isConsistent(getName(), deltaAgentView, priorities, separation)) {
            // current delta is inconsistent
            commitDeltaAgentView();
            Trajectory newTrajectory = getBestResponse(getName(),
                    startWaypoint, destWaypoint, startTime, maneuvers,
                    agentView, priorities, separation, maxTime,
                    approxSamplingInterval);

            if (newTrajectory != null) {
                // We've got a new assignment
                agentView.set(getName(), newTrajectory);
                broadcast(new InformNewTrajectory(getName(),
                        priorities.get(getName()), newTrajectory));
                recordAgentViewDelta(getName(), newTrajectory);
            } else {
                throw new NoSolutionFoundException();
            }
        }

    }

    protected void checkConsistency() {

        if (getCurrentTrajectory() == null) {
            // No trajectory generated yet
            Trajectory newTrajectory = getBestResponse(getName(),
                    startWaypoint, destWaypoint, startTime, maneuvers,
                    agentView, priorities, separation, maxTime,
                    approxSamplingInterval);

            if (newTrajectory != null) {
                // We've got a new assignment
                agentView.set(getName(), newTrajectory);
                broadcast(new InformNewTrajectory(getName(),
                        priorities.get(getName()), newTrajectory));
            } else {
                throw new NoSolutionFoundException();
            }
        } else {
            // We have a trajectory, but we may need to replan the current
            // trajectory
            switch (replanningStrategy) {
            case IF_INCONSISTENT:
                if (!isConsistent(getName(), agentView, priorities,
                        separation)) {
                    Trajectory newTrajectory = getBestResponse(getName(),
                            startWaypoint, destWaypoint, startTime, maneuvers,
                            agentView, priorities, separation, maxTime,
                            approxSamplingInterval);

                    if (newTrajectory != null) {
                        // We've got a new assignment
                        // LOGGER.trace(getName() + ": Current traj. is inconsistent. Adopted new: " + newTrajectory);
                        agentView.set(getName(), newTrajectory);
                        broadcast(new InformNewTrajectory(getName(),
                                priorities.get(getName()), newTrajectory));
                    } else {
                        throw new NoSolutionFoundException();
                    }
                } else {
                    //LOGGER.trace(getName() + ": Current trajectory is good: "  + getCurrentTrajectory());
                }

                break;

            case IF_BETTER_AVAILABLE:
                Trajectory newCandidateTrajectory = getBestResponse(getName(),
                        startWaypoint, destWaypoint, startTime, maneuvers,
                        agentView, priorities, separation, maxTime,
                        approxSamplingInterval);

                if (newCandidateTrajectory != null) {
                    if (!isConsistent(getName(), agentView, priorities,
                            separation)
                            || newCandidateTrajectory.getDuration() < getCurrentTrajectory()
                                    .getDuration()) {
                        LOGGER.trace(getName()
                                + ": Current traj. is too long or inconsistent. Adopted new: "
                                + newCandidateTrajectory);
                        agentView.set(getName(), newCandidateTrajectory);
                        broadcast(new InformNewTrajectory(getName(),
                                priorities.get(getName()),
                                newCandidateTrajectory));
                    } else {
                        LOGGER.trace(getName()
                                + ": Current trajectory is good: "
                                + getCurrentTrajectory());
                    }
                } else {
                    throw new NoSolutionFoundException();
                }

                break;

            case IF_DIFFERENT_AVAILABLE:
                Trajectory newTrajectory = getBestResponse(getName(),
                        startWaypoint, destWaypoint, startTime, maneuvers,
                        agentView, priorities, separation, maxTime,
                        approxSamplingInterval);

                if (newTrajectory != null) {
                    if (!newTrajectory.equals(getCurrentTrajectory())) {
                        LOGGER.trace(getName() + ": Adopted new traj.: "
                                + newTrajectory);
                        agentView.set(getName(), newTrajectory);
                        broadcast(new InformNewTrajectory(getName(), priorities.get(getName()), newTrajectory));
                    }
                } else {
                    throw new NoSolutionFoundException();
                }

                break;

            default:
                throw new RuntimeException("Unknown replanning strategy.");
            }
        }

    }
}

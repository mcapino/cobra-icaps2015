package cz.agents.deconfliction.agent;

import java.util.HashMap;
import java.util.Map;

import org.apache.log4j.Logger;
import org.jgrapht.Graph;

import cz.agents.alite.communication.Message;
import cz.agents.alite.trajectorytools.graph.spatial.maneuvers.SpatialManeuver;
import cz.agents.alite.trajectorytools.trajectory.Trajectory;
import cz.agents.alite.trajectorytools.util.OrientedPoint;
import cz.agents.alite.trajectorytools.util.Waypoint;
import cz.agents.deconfliction.solver.NoSolutionFoundException;
import cz.agents.deconfliction.solver.dpp.InformDone;
import cz.agents.deconfliction.solver.dpp.InformNewTrajectory;
import cz.agents.deconfliction.trajectory.CandidateSolution;

public class SynchronousPrioritizedPlanningAgent extends DecentralizedPrioritizedPlanningAgent {

    Logger LOGGER = Logger.getLogger(SynchronousPrioritizedPlanningAgent.class);


    int round = 1;
    CandidateSolution lastRoundSolution = new CandidateSolution();

    Map<String, Integer> done = new HashMap<String, Integer>();

    public int doneBroadcastedCounter = 0;
    public int doneReceivedCounter = 0;

    public SynchronousPrioritizedPlanningAgent(
            String name, Graph<Waypoint, SpatialManeuver> maneuvers,  double startTime,
            OrientedPoint startPoint, OrientedPoint destination,
            double separation, double maxTime, double vmax, double approxSamplingInterval, ReplanningStrategy replanningStrategy) {
        super(name, maneuvers, startTime, startPoint,  destination,  separation, maxTime, vmax, approxSamplingInterval, replanningStrategy);
    }


    @Override
    public void start() {
        iterate();
    }


    @Override
    public void notify(Message message) {

        if (message.getContent() instanceof InformDone) {
            doneReceivedCounter++;
            InformDone inform = (InformDone) message.getContent();
            //LOGGER.trace(getName() + " received: " + inform + " ");
            done.put(inform.getAgentName(), inform.getRound());
            afterDoneMessageCheck();
        }

        super.notify(message);
    }

    protected void afterDoneMessageCheck() {
        if (allAgentsDoneWithRound(round)) {
            // Making transition to the next round
            LOGGER.debug(getName() + ": All agents done with round " + round + ".");
            // Has any of the assignments changed since the last round?
            if (!lastRoundSolution.equals(agentView)) {
                lastRoundSolution = new CandidateSolution(agentView);
                round++;
                LOGGER.debug(getName() + ": Not converged yet. starting round " + round+".");
                iterate();
            }
        }
    }

    protected void iterate() {

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
                if (!isConsistent(getName(), agentView, priorities, separation)) {
                    Trajectory newTrajectory = getBestResponse(getName(),
                            startWaypoint, destWaypoint, startTime, maneuvers,
                            agentView, priorities, separation, maxTime,
                            approxSamplingInterval);

                    if (newTrajectory != null) {
                        // We've got a new assignment
                        //LOGGER.trace(getName() + ": Current traj. is inconsistent. Adopted new: " + newTrajectory);
                        agentView.set(getName(), newTrajectory);
                        broadcast(new InformNewTrajectory(getName(),
                                priorities.get(getName()), newTrajectory));
                    } else {
                        throw new NoSolutionFoundException();
                    }
                } else {
                    //LOGGER.trace(getName() + ": Current trajectory is good: " + getCurrentTrajectory());
                }

                break;

            case IF_BETTER_AVAILABLE:
                Trajectory newCandidateTrajectory = getBestResponse(getName(),
                        startWaypoint, destWaypoint, startTime, maneuvers,
                        agentView, priorities, separation, maxTime,
                        approxSamplingInterval);

                if (newCandidateTrajectory != null) {
                    if (!isConsistent(getName(), agentView, priorities, separation) ||
                        newCandidateTrajectory.getDuration() < getCurrentTrajectory().getDuration()) {
                        LOGGER.trace(getName() + ": Current traj. is too long or inconsistent. Adopted new: " + newCandidateTrajectory);
                        agentView.set(getName(), newCandidateTrajectory);
                        broadcast(new InformNewTrajectory(getName(),
                                priorities.get(getName()), newCandidateTrajectory));
                    } else {
                        LOGGER.trace(getName() + ": Current trajectory is good: " + getCurrentTrajectory());
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
                        LOGGER.trace(getName() + ": Adopted new traj.: " + newTrajectory);
                        agentView.set(getName(), newTrajectory);
                        broadcast(new InformNewTrajectory(getName(),
                                priorities.get(getName()), newTrajectory));
                    }
                } else {
                    throw new NoSolutionFoundException();
                }

                break;

            default:
                throw new RuntimeException("Unknown replanning strategy.");
            }

        }

        done.put(getName(), round);
        doneBroadcastedCounter++;
        broadcast(new InformDone(getName(), round));
    }

    private boolean allAgentsDoneWithRound(int round) {
        for (String agentName : agents) {
            if (done.get(agentName) != null) {
                if (done.get(agentName) < round) {
                    return false;
                }
            } else {
                return false;
            }
        }
        return true;
    }


    @Override
    public String toString() {
        return getName() + "[round=" + round
                + ", infoBroadcast=" + getBroadcastMessageCounter() + " infoRecv.=" + getInfoReceivedCounter()
                + ", doneBroadcast=" + doneBroadcastedCounter + " doneRecv.=" + doneReceivedCounter +"]";
    }

    public int getRound() {
        return round;
    }

}

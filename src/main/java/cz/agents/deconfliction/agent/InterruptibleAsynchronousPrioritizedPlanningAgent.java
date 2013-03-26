package cz.agents.deconfliction.agent;

import java.util.LinkedList;
import java.util.List;

import org.apache.log4j.Logger;
import org.jgrapht.Graph;

import cz.agents.alite.common.event.DurativeEvent;
import cz.agents.alite.common.event.DurativeEventHandler;
import cz.agents.alite.common.event.DurativeEventProcessor;
import cz.agents.alite.communication.Message;
import cz.agents.alite.trajectorytools.graph.spatial.maneuvers.SpatialManeuver;
import cz.agents.alite.trajectorytools.trajectory.Trajectory;
import cz.agents.alite.trajectorytools.util.OrientedPoint;
import cz.agents.alite.trajectorytools.util.Waypoint;
import cz.agents.deconfliction.planner4d.Planner4d;
import cz.agents.deconfliction.planner4d.Planner4d.SearchStepResult;
import cz.agents.deconfliction.solver.NoSolutionFoundException;
import cz.agents.deconfliction.solver.dpp.InformNewTrajectory;

public class InterruptibleAsynchronousPrioritizedPlanningAgent extends
        AsynchronousDecentralizedPrioritizedPlanningAgent {

    public static enum InterruptingStrategy {
        IF_INCONSISTENT,
        ALWAYS
    };

    Logger LOGGER = Logger
            .getLogger(InterruptibleAsynchronousPrioritizedPlanningAgent.class);

    DurativeEventProcessor eventProcessor;
    Planner4d planner = null;

    InterruptingStrategy interruptingStrategy = InterruptingStrategy.IF_INCONSISTENT;

    public InterruptibleAsynchronousPrioritizedPlanningAgent(String name,
            Graph<Waypoint, SpatialManeuver> maneuvers, double startTime,
            OrientedPoint startPoint, OrientedPoint destination,
            double separation, double maxTime, double approxSamplingInterval,
            ReplanningStrategy replanningStrategy, DurativeEventProcessor eventProcessor) {
        super(name, maneuvers, startTime, startPoint, destination, separation, maxTime,
                approxSamplingInterval, replanningStrategy);

        this.eventProcessor = eventProcessor;
    }

    @Override
    public void start() {
        resetPlanner();
    }


    @Override
    public void notify(Message message) {
        super.notify(message);
    }

    protected void plannerStep() {
        // LOGGER.trace(getName() + "@" + getTimeString() + ": Invoked search step...");
        if (planner != null) {
            long startNanos = System.nanoTime();
            SearchStepResult result = planner.searchStep(1 /*ms*/ * (1000000));
            plannerCumulativeRuntime += System.nanoTime() - startNanos;
            plannerCumulativeExpandedStates += planner.getExpandedStatesCounter();

            if (result.isFinished()) {
                if (result.foundSolution()) {
                    // We've got new solution
                    Trajectory newTrajectory = result.getAsTrajectory(startTime, maxTime, approxSamplingInterval);

                    if (getCurrentTrajectory() == null) {
                        //LOGGER.trace(getName() + "@" + getTimeString() + ": Planned first trajectory: " + newTrajectory);
                        agentView.set(getName(), newTrajectory);
                        broadcast(new InformNewTrajectory(getName(),
                                priorities.get(getName()), newTrajectory));

                        planner = null;
                        recordAgentViewDelta(getName(), newTrajectory);
                        if (interruptingStrategy.equals(InterruptingStrategy.IF_INCONSISTENT)) {
                            checkDeltaConsistency();
                        }
                        return;

                    } else {
                        switch (replanningStrategy) {
                        case IF_INCONSISTENT:
                            // We've got a new assignment
                            //LOGGER.trace(getName() + "@" + getTimeString() + ": Current traj. is inconsistent. Adopted new: " + newTrajectory);
                            agentView.set(getName(), newTrajectory);
                            broadcast(new InformNewTrajectory(getName(),
                                    priorities.get(getName()), newTrajectory));

                            planner = null;
                            recordAgentViewDelta(getName(), newTrajectory);
                            if (interruptingStrategy.equals(InterruptingStrategy.IF_INCONSISTENT)) {
                                checkDeltaConsistency();
                            }
                            return;
                            //break;

                        default:
                            break;
                        }
                    }

                    planner = null;
                    return;

                } else {
                    throw new NoSolutionFoundException();
                }
            }

            eventProcessor.addEvent(eventProcessor.getCurrentEventTime()
                    + eventProcessor.getCurrentEventHandlingDurationNanos(),
                    getName(), new DurativeEventHandler() {

                @Override
                public long handleEvent(DurativeEvent event) {
                    plannerStep();
                    return COUNT_SYSTEM_NANOS;
                }

                @Override
                public DurativeEventProcessor getEventProcessor() {
                    return eventProcessor;
                }
            });
        }
    }



    @Override
    protected void afterInfoMessageCheck(InformNewTrajectory inform) {
        recordAgentViewDelta(inform);

        if (inform.getPriority() < priorities.get(getName())) {
            trajectoryInbox.add(inform.getTrajectory());
        } // Ignore lower priority messages

        if (getInboxSize() > 0 || trajectoryInbox.isEmpty()) {
            return;
        }

        if (planner == null) {
            // The planner is not running
            checkDeltaConsistency();
        } else {
            // The planner is running, should we interrupt it?

            if (interruptingStrategy.equals(InterruptingStrategy.IF_INCONSISTENT)) {
                if (getCurrentTrajectory() != null && !isConsistent(getCurrentTrajectory(), trajectoryInbox, separation)) {
                    commitDeltaAgentView();
                    resetPlanner();
                }

            } else if (interruptingStrategy.equals(InterruptingStrategy.ALWAYS)) {
                commitDeltaAgentView();
                resetPlanner();
            }
        }

        trajectoryInbox.clear();

    }


    protected void checkDeltaConsistency() {
        if (!isConsistent(getName(), deltaAgentView, priorities, separation)) {
            // current delta is inconsistent
            commitDeltaAgentView();
            resetPlanner();
        }
    }

    protected void checkConsistency() {
        switch (replanningStrategy) {
        case IF_INCONSISTENT:
            if (getCurrentTrajectory() != null) {
                if (!isConsistent(getName(), agentView, priorities, separation)) {
                    // current delta is inconsistent
                    resetPlanner();
                }
            }
            break;

        default:
            break;
        }

    }

    private void resetPlanner() {
        plannerInvocationCounter++;
        LOGGER.trace(getName() + "@" + getTimeString() + ": Reseting the solver ");
        List<Trajectory> hardConstraints = new LinkedList<Trajectory>();

        for (String agentName : agentView.getTrajectories().keySet()) {
            if (priorities.get(getName()) > priorities.get(agentName)) {
                hardConstraints.add(agentView.getTrajectories().get(agentName));
            }
        }

        List<Trajectory> softConstraints = new LinkedList<Trajectory>();

        if (planner == null) {
            // start the solver execution loop
            eventProcessor.addEvent(eventProcessor.getCurrentEventTime()
                    + eventProcessor.getCurrentEventHandlingDurationNanos(),
                    getName(), new DurativeEventHandler() {

                @Override
                public long handleEvent(DurativeEvent event) {
                    plannerStep();
                    return COUNT_SYSTEM_NANOS;
                }

                @Override
                public DurativeEventProcessor getEventProcessor() {
                    return eventProcessor;
                }
            });
        }
        planner = new Planner4d(startWaypoint, startTime, destWaypoint, maneuvers,
                separation, maxTime, approxSamplingInterval,
                hardConstraints,
                softConstraints);
    }

    private long getTimeNanos() {
        return eventProcessor.getCurrentEventTime() + eventProcessor.getCurrentEventHandlingDurationNanos();
    }

    private double getTimeSec() {
        return getTimeNanos() / 1e9d;
    }

    private String getTimeString() {
        return String.format("%.4f", getTimeSec());
    }

}

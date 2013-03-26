package cz.agents.deconfliction.agent;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import org.apache.log4j.Logger;
import org.jgrapht.Graph;

import cz.agents.alite.communication.Communicator;
import cz.agents.alite.communication.InboxBasedCommunicator;
import cz.agents.alite.communication.Message;
import cz.agents.alite.communication.MessageHandler;
import cz.agents.alite.communication.content.Content;
import cz.agents.alite.trajectorytools.graph.spatial.SpatialGraphs;
import cz.agents.alite.trajectorytools.graph.spatial.maneuvers.SpatialManeuver;
import cz.agents.alite.trajectorytools.trajectory.Trajectory;
import cz.agents.alite.trajectorytools.trajectory.EvaluatedTrajectoryWrapper;
import cz.agents.alite.trajectorytools.util.OrientedPoint;
import cz.agents.alite.trajectorytools.util.SeparationDetector;
import cz.agents.alite.trajectorytools.util.Waypoint;
import cz.agents.deconfliction.planner4d.Planner4d;
import cz.agents.deconfliction.solver.dpp.InformNewTrajectory;
import cz.agents.deconfliction.trajectory.CandidateSolution;

public abstract class DecentralizedPrioritizedPlanningAgent extends CommunicatingAgent implements MessageHandler {

    public static enum ReplanningStrategy {
        IF_INCONSISTENT,
        IF_BETTER_AVAILABLE,
        IF_DIFFERENT_AVAILABLE
    };

    private static Logger LOGGER = Logger.getLogger(DecentralizedPrioritizedPlanningAgent.class);

    protected Waypoint startWaypoint;
    protected Waypoint destWaypoint;

    protected CandidateSolution agentView = new CandidateSolution();
    protected Map<String, Double> priorities = new HashMap<String, Double>();

    protected Graph<Waypoint, SpatialManeuver> maneuvers;
    protected double separation;
    protected double maxTime;
    protected double approxSamplingInterval;
    protected double vmax;

    protected final ReplanningStrategy replanningStrategy;

    private int infoBroadcastMessageCounter = 0;
    private int infoReceivedCounter = 0;
    public int plannerInvocationCounter = 0;
    public int plannerConstraintCounter = 0;
    public long plannerCumulativeRuntime = 0;
    public long plannerCumulativeExpandedStates = 0;
    public long communicationCumulativeRuntime = 0;
    public long conflictCheckingCumulativeRuntime = 0;
    public long conflictCheckCounter = 0;


    public DecentralizedPrioritizedPlanningAgent(String name, Graph<Waypoint, SpatialManeuver> maneuvers, double startTime,
            OrientedPoint startPoint, OrientedPoint destination, double separation, double maxTime, double vmax, double approxSamplingInterval, ReplanningStrategy replanningStrategy) {
        super(name, startPoint, startTime, destination);

        this.separation = separation;
        this.maxTime = maxTime;
        this.approxSamplingInterval = approxSamplingInterval;
        this.maneuvers = maneuvers;
        this.vmax = vmax;

        this.startWaypoint = SpatialGraphs.getNearestVertex(maneuvers,startPoint);
        this.destWaypoint = SpatialGraphs.getNearestVertex(maneuvers,destination);

        this.replanningStrategy = replanningStrategy;
    }



    @Override
    public void setCommunicator(Communicator communicator, List<String> agents) {
        super.setCommunicator(communicator, agents);

        List<String> agentNames = new ArrayList<String>(agents);
        Collections.sort(agentNames);

        double priority = 0.0;
        for (String agentName : agentNames) {
            priorities.put(agentName, priority);
            priority += 1.0;
        }

        getCommunicator().addMessageHandler(this);
    }

    @Override
    public abstract void start();

    @Override
    public Trajectory getCurrentTrajectory() {
        return agentView.get(getName());
    }

    @Override
    public void notify(Message message) {
        if (message.getContent() instanceof InformNewTrajectory) {
            InformNewTrajectory inform = (InformNewTrajectory) message.getContent();
            infoReceivedCounter++;

            //LOGGER.trace(getName() + " received: " + inform + " " + toString());
            agentView.set(inform.getAgentName(), inform.getTrajectory());
            priorities.put(inform.getAgentName(), inform.getPriority());

            //LOGGER.info(getName() + ": Inbox: " + getInboxSize() + " Received inform: "+inform);
            afterInfoMessageCheck(inform);
        }
    }

    protected void afterInfoMessageCheck(InformNewTrajectory message) { }

    protected void broadcast(Content content) {
        long startNanos = System.nanoTime();
        Message newPathMsg = getCommunicator().createMessage(content);
        LinkedList<String> receivers = new LinkedList<String>(agents);
        receivers.remove(getName());
        newPathMsg.addReceivers(receivers);
        getCommunicator().sendMessage(newPathMsg);

        communicationCumulativeRuntime += (System.nanoTime() - startNanos);

        if (content instanceof InformNewTrajectory) {
            infoBroadcastMessageCounter++;
        }
    }

    protected EvaluatedTrajectoryWrapper getBestResponse(String myName,
            Waypoint startWaypoint, Waypoint destWaypoint, double startTime,
            Graph<Waypoint, SpatialManeuver> maneuvers, CandidateSolution candidateSolution,
            Map<String, Double> priorities,
            double separation, double maxTime, double approxSamplingInterval) {

        // Get the trajectories of all higher-priority agents
        List<Trajectory> hardConstraints = new LinkedList<Trajectory>();

        for (String agentName : candidateSolution.getTrajectories().keySet()) {
            if (priorities.get(myName) > priorities.get(agentName)) {
                hardConstraints.add(candidateSolution.getTrajectories().get(agentName));
            }
        }

        plannerInvocationCounter++;
        plannerConstraintCounter += hardConstraints.size();

        List<Trajectory> softConstraints = new LinkedList<Trajectory>();

        long start = System.nanoTime();

        Planner4d planner = new Planner4d(startWaypoint, startTime,
                destWaypoint, maneuvers, separation, maxTime, approxSamplingInterval, vmax,
                 hardConstraints, softConstraints);

        EvaluatedTrajectoryWrapper foundTrajectory = planner.solveTrajectory();

        long duration = System.nanoTime() - start;
        LOGGER.debug(myName + ": planning best response (" + hardConstraints.size() + " constraints) took " + duration/1e6 + "ms, " + planner.getExpandedStatesCounter() + " states expanded.");

        plannerCumulativeRuntime += duration;
        plannerCumulativeExpandedStates += planner.getExpandedStatesCounter();

        return foundTrajectory;
    }


    protected boolean isConsistent(Trajectory myTrajectory, List<Trajectory> hardConstraints, double separation) {
        //LOGGER.info(getName() + " isConsistent check");
        long startNanos = System.nanoTime();
        conflictCheckCounter++;

        if (myTrajectory == null) {
            // no trajectory is consistent with anything...
            return false;
        }

        boolean result = !SeparationDetector.hasConflict(myTrajectory, hardConstraints, separation);
        conflictCheckingCumulativeRuntime += System.nanoTime() - startNanos;
        return result;
    }

    protected boolean isConsistent(String myName, CandidateSolution candidateSolution, Map<String, Double> priorities, double separation) {
        Trajectory myTrajectory = candidateSolution.get(myName);

        // Get the trajectories of all higher-priority agents
        List<Trajectory> hardConstraints = new LinkedList<Trajectory>();

        for (String agentName : candidateSolution.getTrajectories().keySet()) {
            if (priorities.get(myName) > priorities.get(agentName)) {
                hardConstraints.add(candidateSolution.getTrajectories().get(agentName));
            }
        }

        return isConsistent(myTrajectory, hardConstraints, separation);
    }

    protected boolean isConsistent(CandidateSolution candidateSolution, double separation) {
        long startNanos = System.nanoTime();
        List<Trajectory> trajectoriesList = new LinkedList<Trajectory>();
        for (String agentName : candidateSolution.getTrajectories().keySet()) {
                trajectoriesList.add(candidateSolution.getTrajectories().get(agentName));
        }

        boolean result = !SeparationDetector.hasConflict(trajectoriesList, separation);
        conflictCheckingCumulativeRuntime += System.nanoTime() - startNanos;
        return result;
    }

    public int getBroadcastMessageCounter() {
        return infoBroadcastMessageCounter;
    }

    public int getInboxSize() {
        if (communicator instanceof InboxBasedCommunicator) {
            return ((InboxBasedCommunicator) communicator).getInboxSize();
        } else {
            throw new IllegalArgumentException("getInboxSize() can only be called on an InboxBasedCommunicator.");
        }
    }

    public int getPlannerInvocationCounter() {
        return plannerInvocationCounter;
    }

    public int getPlannerConstraintCounter() {
        return plannerConstraintCounter;
    }

    public int getInfoReceivedCounter() {
        return infoReceivedCounter;
    }

    public long getConflictCheckingCumulativeRuntime() {
        return conflictCheckingCumulativeRuntime;
    }

}

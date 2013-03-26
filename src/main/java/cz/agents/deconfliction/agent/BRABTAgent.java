package cz.agents.deconfliction.agent;

import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.apache.log4j.Logger;
import org.jgrapht.GraphPath;

import cz.agents.alite.communication.Communicator;
import cz.agents.alite.communication.protocol.asynchronousbacktracking.AsynchronousBacktrackingProtocol;
import cz.agents.alite.communication.protocol.asynchronousbacktracking.NogoodMessage;
import cz.agents.alite.communication.protocol.asynchronousbacktracking.OkayMessage;
import cz.agents.alite.communication.protocol.asynchronousbacktracking.StopMessage;
import cz.agents.deconfliction.configuration.Parameters;
import cz.agents.deconfliction.creator.DCSPCreator;

import org.jgrapht.Graph;
import cz.agents.deconfliction.solver.central.Waypoint;
import cz.agents.deconfliction.solver.central.ODSolver;
import cz.agents.deconfliction.trajectory.CandidateSolution;
import cz.agents.deconfliction.trajectory.PiecewiseLinearTrajectory;
import cz.agents.alite.trajectorytools.graph.spatial.SpatialGraphs;
import cz.agents.alite.trajectorytools.graph.spatial.maneuvers.SpatialManeuver;
import cz.agents.alite.trajectorytools.trajectory.Trajectory;
import cz.agents.deconfliction.trajectory.TrajectoryGenerator;
import cz.agents.deconfliction.util.AllSimpleGraphPathsTrajectoryGenerator;
import cz.agents.deconfliction.util.FastestManeuverTrajectory;
import cz.agents.alite.trajectorytools.util.SeparationDetector;
import cz.agents.deconfliction.util.KShortestPathsTrajectoryGenerator;
import cz.agents.alite.trajectorytools.util.OrientedPoint;
import cz.agents.alite.trajectorytools.util.Waypoint;

public class BRABTAgent extends CommunicatingAgent {
    private static final Logger LOGGER = Logger.getLogger(BRABTAgent.class);

    /**
     * Flag determining whether after an okay message has been received (in a new context),
     * the agent should attempt to find a new, possibly better path even if his current path
     * is conflict-free.
     */
    final boolean OPTIMIZE_ON_OKAY = true;

    public enum CheckResult {
        NO_CHANGE, NEW_ASSIGNMENT, BACKTRACKED, STOPPED
    }

    Waypoint startWaypoint;
    Waypoint destWaypoint;
    Graph<Waypoint, SpatialManeuver> maneuvers;

    Trajectory desiredTrajectory;

    AsynchronousBacktrackingProtocol abt;
    CandidateSolution agentView = new CandidateSolution();
    Set<CandidateSolution> nogoods = new HashSet<CandidateSolution>();

    private double separation;
    private Parameters params;

    public BRABTAgent(String name, Graph<Waypoint, SpatialManeuver> maneuvers, double startTime, OrientedPoint startPoint, OrientedPoint destination, Parameters params) {
        super(name, startPoint, startTime, destination);

        startWaypoint = SpatialGraphs.getNearestVertex(maneuvers,startPoint);
        destWaypoint = SpatialGraphs.getNearestVertex(maneuvers,destination);
        this.maneuvers = maneuvers;
        this.params = params;
        this.separation = params.SEPARATION;

        desiredTrajectory = FastestManeuverTrajectory.plan(maneuvers, startTime, startWaypoint, destWaypoint, params.MAX_T, params.APPROXIMATION_SAMPLING_INTERVAL);
        setCurrentTrajectory(desiredTrajectory);
        LOGGER.debug(getName() + " generated the shortest trajectory " + getCurrentTrajectory() + " which is " + getCurrentTrajectory().getDuration() + " meters long");
    }

    public void setCommunicator(Communicator communicator, List<String> agents) {
        super.setCommunicator(communicator, agents);

        abt = new AsynchronousBacktrackingProtocol(communicator, getName(),
                agents) {

            @Override
            protected void processOkayMessage(final OkayMessage content) {
                agentView
                        .set(content.getSenderAgent(), content.getTrajectory());
                LOGGER.trace(BRABTAgent.this.getName() + " received from "
                        + content.getSenderAgent() + " okay? "
                        + content.getTrajectory() + " State:" + BRABTAgent.this);

                checkAgentView();
            }

            @Override
            protected void processNogoodMessage(final NogoodMessage content) {
                CandidateSolution nogood = content.getNogood();
                nogoods.add(nogood);
                LOGGER.trace(BRABTAgent.this.getName() + " received nogood! nogoods size:" + nogoods.size() + " new nogood:"
                        + nogood + " State:" + BRABTAgent.this);

                for (String agentName : nogood.getAgentNames()) {
                    if (!agentView.getAgentNames().contains(agentName)) {
                        agentView.set(agentName, nogood.get(agentName));
                        abt.requestAddMeAsNeighbor(agentName);
                    }
                }

                CheckResult result = checkAgentView();
                if (result.equals(CheckResult.NO_CHANGE)) {
                    abt.sendOkay(content.getSender(), new OkayMessage(
                            BRABTAgent.this.getName(), getCurrentTrajectory()));
                }

            }

            @Override
            protected void processStopMessage(StopMessage content) {
                LOGGER.trace(BRABTAgent.this.getName() + " received STOP! ");
            }

            @Override
            protected void processNewNeighbor(String newNeighborName) {
                abt.sendOkay(newNeighborName,
                        new OkayMessage(BRABTAgent.this.getName(),
                                getCurrentTrajectory()));
            }
        };
    }

    protected void onMessageReceived(String content) {

    }

    @Override
    public Trajectory getCurrentTrajectory() {
        return agentView.get(BRABTAgent.this.getName());
    }

    protected void setCurrentTrajectory(Trajectory trajectory) {
        agentView.set(getName(), trajectory);
    }

    public void start() {
        abt.sendOkay(new OkayMessage(getName(), getCurrentTrajectory()));
        checkAgentView();
    }

    private CheckResult checkAgentView() {

        CheckResult searchResult = CheckResult.NO_CHANGE;

        CandidateSolution candidate = new CandidateSolution(agentView);

        if (!isConsistent(candidate, nogoods, separation)) {
            Trajectory t;
            if ((t = getBestResponse(candidate, nogoods, separation)) != null) {
                searchResult = CheckResult.NEW_ASSIGNMENT;
                candidate.set(BRABTAgent.this.getName(), t);
            } else {
                // no solution found
                searchResult = CheckResult.BACKTRACKED;
            }
        } else {
            if (OPTIMIZE_ON_OKAY) {
                Trajectory newBestResponse = getBestResponse(candidate, nogoods, separation);
                if (newBestResponse != null) {
                    if (!getCurrentTrajectory().equals(newBestResponse) && newBestResponse.getDuration() <= getCurrentTrajectory().getDuration()) {
                        LOGGER.trace(getName()
                                + " CheckAgentView: Found a better best response: " + newBestResponse
                                + " State:" + BRABTAgent.this);
                        searchResult = CheckResult.NEW_ASSIGNMENT;
                        candidate.set(BRABTAgent.this.getName(), newBestResponse);
                    }
                } else {
                    // no solution found
                    searchResult = CheckResult.BACKTRACKED;
                }
            }
        }

        if (searchResult.equals(CheckResult.NO_CHANGE)) {
            LOGGER.trace(getName()
                    + " CheckAgentView: Current Trajectory is consistent. No change."
                    + " State:" + BRABTAgent.this);
            return CheckResult.NO_CHANGE;
        } else if (searchResult.equals(CheckResult.NEW_ASSIGNMENT)) {
            agentView = candidate;
            LOGGER.trace(getName()
                    + " CheckAgentView: Generated new trajectory "
                    + getCurrentTrajectory() + " which is "
                    + getCurrentTrajectory().getDuration() + " meters long."
                    + " State:" + BRABTAgent.this);

            if (params.VISUALIZE_NEW_ASSIGNMENT) {
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            abt.sendOkay(new OkayMessage(getName(), getCurrentTrajectory()));
            return CheckResult.NEW_ASSIGNMENT;

        } else if (searchResult.equals(CheckResult.BACKTRACKED)) {
            LOGGER.trace(getName()
                    + " CheckAgentView: Found no consistent trajectory. Backtracking... AgentView: "
                    + agentView + "." + " State:" + BRABTAgent.this);

            if (params.VISUALIZE_NEW_ASSIGNMENT) {
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }

            return backtrack();
        }

        return null;

    }

    private Trajectory getBestResponse(CandidateSolution candidate,
            Collection<CandidateSolution> nogoods, double separation) {

        // Get the trajectories of all higher-priority agents
        List<Trajectory> hardConstraints = new LinkedList<Trajectory>();

        for (String agentName : candidate.getTrajectories().keySet()) {
            if (agentName.compareTo(getName()) < 0) {
                hardConstraints.add(candidate.getTrajectories().get(agentName));
            }
        }

        List<Trajectory> softConstraints = new LinkedList<Trajectory>();
        ODSolver solver = new ODSolver(new Waypoint(startWaypoint, null, startTime), new Waypoint(destWaypoint, null, 0.0), maneuvers, separation, params.APPROXIMATION_SAMPLING_INTERVAL, Double.POSITIVE_INFINITY, hardConstraints, softConstraints);

        Trajectory[] foundTrajectories = solver.solveTrajectories();

        if (foundTrajectories != null) {
            return foundTrajectories[0];
        }

        return null;
    }

    private CheckResult backtrack() {
        // TODO Take some inconsistent set using hyper-resolution or similar
        // procedure...
        CandidateSolution nogood = new CandidateSolution(agentView);

        /*
         * Since we are searching through the agents' domains systematically and
         * exhaustively, we can deduce that we tried all the combinations for
         * the variables of the lower priority agents.
         *
         * Thus, the minimal inconsistent set is the current agent view minus me
         * and all the lower priority agents.
         */

        List<String> nogoodAgents = new LinkedList<String>(nogood.getAgentNames());
        for (String nogoodAgent : nogoodAgents) {
            if (getName().compareTo(nogoodAgent) <= 0) {
                nogood.remove(nogoodAgent);
            }
        }

        if (nogood.isEmpty()) {
            // broadcast stop, no solution exists
            LOGGER.info("Generated empty nogood, stopping...");
            abt.sendStop(new StopMessage());
            return CheckResult.STOPPED;
        } else {
            String lowestPriorityAgentName = nogood.getLowestPriorityAgent();
            abt.sendNogood(lowestPriorityAgentName, new NogoodMessage(
                    getName(), nogood));
            agentView.remove(lowestPriorityAgentName);
            return checkAgentView();
        }
    }

    private boolean isConsistent(CandidateSolution candidateSolution,
            Set<CandidateSolution> nogoods, double separation) {
        for (CandidateSolution nogood : nogoods) {
            if (nogood.equals(candidateSolution))
                return false;
        }

        Trajectory myTrajectory = candidateSolution.get(getName());

        // Get the trajectories of all higher-priority agents
        List<Trajectory> hardConstraints = new LinkedList<Trajectory>();

        for (String agentName : candidateSolution.getTrajectories().keySet()) {
            if (agentName.compareTo(getName()) < 0) {
                hardConstraints.add(candidateSolution.getTrajectories().get(agentName));
            }
        }

        return !SeparationDetector.hasConflict(myTrajectory, hardConstraints, separation);
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append("DCSPAgent(name: ");
        sb.append(getName());
        sb.append(", agentView: ");
        sb.append(agentView);
        sb.append(", nogoods: ");
        sb.append(nogoods);
        sb.append(")");

        return sb.toString();
    }

}

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

import cz.agents.alite.communication.Communicator;
import cz.agents.alite.communication.protocol.asynchronousbacktracking.AsynchronousBacktrackingProtocol;
import cz.agents.alite.communication.protocol.asynchronousbacktracking.NogoodMessage;
import cz.agents.alite.communication.protocol.asynchronousbacktracking.OkayMessage;
import cz.agents.alite.communication.protocol.asynchronousbacktracking.StopMessage;
import cz.agents.deconfliction.configuration.Parameters;
import cz.agents.deconfliction.creator.DCSPCreator;
import org.jgrapht.Graph;
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

public class DCSPAgent extends CommunicatingAgent {
    private static final Logger LOGGER = Logger.getLogger(DCSPAgent.class);

    public enum CheckResult {
        NO_CHANGE, NEW_ASSIGNMENT, BACKTRACKED, STOPPED
    }

    Waypoint startWaypoint;
    Waypoint destWaypoint;
    Graph<Waypoint, SpatialManeuver> maneuvers;

    TrajectoryGenerator trajectoryGenerator;
    Trajectory desiredTrajectory;

    AsynchronousBacktrackingProtocol abt;
    CandidateSolution agentView = new CandidateSolution();
    Set<CandidateSolution> nogoods = new HashSet<CandidateSolution>();

    private double separation;
    private Parameters params;

    public DCSPAgent(String name, Graph<Waypoint, SpatialManeuver> waypoints, OrientedPoint start,
            OrientedPoint destination, double startTime, Parameters params) {
        this(name, waypoints, start, destination, startTime,
                new KShortestPathsTrajectoryGenerator(waypoints,
                		SpatialGraphs.getNearestVertex(waypoints, start),
                		SpatialGraphs.getNearestVertex(waypoints, destination), startTime,
                        params.APPROXIMATION_SAMPLING_INTERVAL,
                        params.CANDIDATES),
                params);
    }

    public DCSPAgent(String name, Graph<Waypoint, SpatialManeuver> maneuvers, OrientedPoint startPoint, OrientedPoint destination, double startTime, TrajectoryGenerator trajectoryGenerator, Parameters params) {
        super(name, startPoint, startTime, destination);

        startWaypoint = SpatialGraphs.getNearestVertex(maneuvers,startPoint);
        destWaypoint = SpatialGraphs.getNearestVertex(maneuvers,destination);
        this.maneuvers = maneuvers;
        this.params = params;
        this.separation = params.SEPARATION;

        desiredTrajectory = FastestManeuverTrajectory.plan(maneuvers, startTime, startWaypoint, destWaypoint, params.MAX_T, params.APPROXIMATION_SAMPLING_INTERVAL);
        setCurrentTrajectory(desiredTrajectory);
        LOGGER.debug(getName() + " generated the shortest trajectory " + getCurrentTrajectory() + " which is " + getCurrentTrajectory().getDuration() + " meters long");

        this.trajectoryGenerator = trajectoryGenerator;
        LOGGER.info(getName() + " generated " + ((KShortestPathsTrajectoryGenerator)trajectoryGenerator).size() + " alt. trajectories.");

        //trajectoryGenerator = new AllGraphPathsTrajectoryGenerator(waypoints, startWaypoint, destWaypoint,speed, approximationTimeStep, approximationMaxTime);
        //this.trajectoryGenerator = new KShortestPathsTrajectoryGenerator(waypoints, startWaypoint, destWaypoint, startTime, speed, params.APPROXIMATION_TIME_STEP, params.APPROXIMATION_MAX_TIME, params.CANDIDATES);
    }

    public void setCommunicator(Communicator communicator, List<String> agents) {
        super.setCommunicator(communicator, agents);

        abt = new AsynchronousBacktrackingProtocol(communicator, getName(),
                agents) {

            @Override
            protected void processOkayMessage(final OkayMessage content) {
                agentView
                        .set(content.getSenderAgent(), content.getTrajectory());
                LOGGER.trace(DCSPAgent.this.getName() + " received from "
                        + content.getSenderAgent() + " okay? "
                        + content.getTrajectory() + " State:" + DCSPAgent.this);

                checkAgentView();
            }

            @Override
            protected void processNogoodMessage(final NogoodMessage content) {
                CandidateSolution nogood = content.getNogood();
                nogoods.add(nogood);
                LOGGER.trace(DCSPAgent.this.getName() + " received nogood! nogoods size:" + nogoods.size() + " new nogood:"
                        + nogood + " State:" + DCSPAgent.this);

                for (String agentName : nogood.getAgentNames()) {
                    if (!agentView.getAgentNames().contains(agentName)) {
                        agentView.set(agentName, nogood.get(agentName));
                        abt.requestAddMeAsNeighbor(agentName);
                    }
                }

                CheckResult result = checkAgentView();
                if (result.equals(CheckResult.NO_CHANGE)) {
                    abt.sendOkay(content.getSender(), new OkayMessage(
                            DCSPAgent.this.getName(), getCurrentTrajectory()));
                }

            }

            @Override
            protected void processStopMessage(StopMessage content) {
                LOGGER.trace(DCSPAgent.this.getName() + " received STOP! ");
            }

            @Override
            protected void processNewNeighbor(String newNeighborName) {
                abt.sendOkay(newNeighborName,
                        new OkayMessage(DCSPAgent.this.getName(),
                                getCurrentTrajectory()));
            }
        };
    }

    protected void onMessageReceived(String content) {

    }

    @Override
    public Trajectory getCurrentTrajectory() {
        return agentView.get(getName());
    }

    protected void setCurrentTrajectory(Trajectory trajectory) {
        agentView.set(getName(), trajectory);
    }

    protected Trajectory getNextAlternativeTrajectory() {
        return trajectoryGenerator.nextTrajectory();
    }

    public void generateNewTrajectory() {
        setCurrentTrajectory(getNextAlternativeTrajectory());
    }

    protected void resetTrajectoryGenerator() {
        trajectoryGenerator.reset();
    }

    public void start() {
        abt.sendOkay(new OkayMessage(getName(), getCurrentTrajectory()));
        checkAgentView();
    }

    private CheckResult checkAgentView() {

        CheckResult searchResult = CheckResult.NO_CHANGE;

        resetTrajectoryGenerator();
        CandidateSolution candidate = new CandidateSolution(agentView);

        while (!isConsistent(candidate, nogoods, separation)) {
            Trajectory t;
            if ((t = getNextAlternativeTrajectory()) != null) {
                searchResult = CheckResult.NEW_ASSIGNMENT;
                candidate.set(DCSPAgent.this.getName(), t);

                if (params.VISUALISE_SEARCH) {
                    setCurrentTrajectory(t);
                    try {
                        Thread.sleep(100);
                    } catch (InterruptedException e) {
                    }
                }
            } else {
                // no more solutions
                searchResult = CheckResult.BACKTRACKED;
                break;
            }
        }

        if (searchResult.equals(CheckResult.NO_CHANGE)) {
            LOGGER.trace(getName()
                    + " CheckAgentView: Current Trajectory is consistent. No change."
                    + " State:" + DCSPAgent.this);
            return CheckResult.NO_CHANGE;
        } else if (searchResult.equals(CheckResult.NEW_ASSIGNMENT)) {
            agentView = candidate;
            LOGGER.trace(getName()
                    + " CheckAgentView: Generated new trajectory "
                    + getCurrentTrajectory() + " which is "
                    + getCurrentTrajectory().getDuration() + " meters long."
                    + " State:" + DCSPAgent.this);

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
                    + agentView + "." + " State:" + DCSPAgent.this);

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
        Map<String, Trajectory> otherTrajectoriesMap = new HashMap<String, Trajectory>(
                candidateSolution.getTrajectories());
        otherTrajectoriesMap.remove(getName());
        Collection<Trajectory> otherTrajectories = otherTrajectoriesMap
                .values();

        return !SeparationDetector.hasConflict(myTrajectory, otherTrajectories,
                separation);
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

package cz.agents.deconfliction.creator.centralsolver;

import java.text.DecimalFormat;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

import org.jgrapht.Graph;

import cz.agents.alite.trajectorytools.graph.spatial.maneuvers.SpatialManeuver;
import cz.agents.alite.trajectorytools.trajectory.Trajectory;
import cz.agents.alite.trajectorytools.util.OrientedPoint;
import cz.agents.alite.trajectorytools.util.Vector;
import cz.agents.alite.trajectorytools.util.Waypoint;
import cz.agents.deconfliction.agent.Agent;
import cz.agents.deconfliction.agent.FastestTrajectoryAgent;
import cz.agents.deconfliction.configuration.Parameters;
import cz.agents.deconfliction.creator.DeconflictionCreator;
import cz.agents.deconfliction.creator.SolverResult;
import cz.agents.deconfliction.solver.central.DeconflictionProblemSolver;
import cz.agents.deconfliction.util.AgentMissionGenerator.Mission;

public class CACustomCreator extends DeconflictionCreator {

    private Graph<Waypoint, SpatialManeuver> maneuvers;
    private Mission[] agentMissions;
    private long runtimeMs;
    private Parameters params;
    private double nominalCost;
    private double finalCost;
    private int conflictCount;


    public void create(Graph<Waypoint, SpatialManeuver> maneuvers, Mission[] agentMissions, Parameters params) {
        this.maneuvers = maneuvers;
        this.agentMissions = agentMissions;
        this.params = params;

        create();
    }


    @Override
    protected void startResolution() {
        // Solve centrally
        nominalCost = getProblem().getSumOfTrajectories();
        long startTime = System.nanoTime();
        conflictCount = getProblem().getConflicts().size();
        DeconflictionProblemSolver.solveUsingCA(getProblem(), getManeuvers(), getParams().MAX_T, getParams().APPROXIMATION_SAMPLING_INTERVAL, getParams().SPEED);
        runtimeMs = (System.nanoTime()-startTime)/1000000;
        LOGGER.info("Runtime: "+(System.nanoTime()-startTime)/1000000+"ms.");
        finalCost = getProblem().getSumOfTrajectories();
    }

    @Override
    protected List<Agent> createAgents() {
        List<Agent> agents = new LinkedList<Agent>();

        for (int i = 0; i < agentMissions.length; i++) {
            agents.add(new FastestTrajectoryAgent("A" + new DecimalFormat("00").format(i), getManeuvers(),
                    0.0, new OrientedPoint(agentMissions[i].start, new Vector(0, 1, 0)),
                    new OrientedPoint(agentMissions[i].end, new Vector(0, 1, 0)),
                    getParams().MAX_T,
                    getParams().APPROXIMATION_SAMPLING_INTERVAL));
        }
        return agents;
    }

    @Override
    protected Parameters configureParameters(Parameters params) {
        return this.params;
    }

    protected void setupAgents() {
        // Create

        List<Agent> agents = createAgents();
        Collections.sort(agents);
        getProblem().setAgents(agents);
    }

    @Override
    protected Graph<Waypoint, SpatialManeuver> generateGraph() {
        return maneuvers;
    }

    public double getRuntime() {
        return runtimeMs / 1e3d;
    }

    public double getNominalCost() {
        return nominalCost;
    }

    public double getFinalCost() {
        return finalCost;
    }


    public Trajectory[] getSolutionTrajectories() {
        return getProblem().getTrajectories();
    }

    public int getConflictCount() {
        return conflictCount;
    }

    public SolverResult getResult() {
        return new SolverResult(finalCost, runtimeMs/1000.0, runtimeMs/1000.0, this.agentMissions.length*2, 0);
    }

}

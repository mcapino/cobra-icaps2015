package cz.agents.deconfliction.creator.centralsolver;

import java.util.Collections;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Random;
import java.util.Set;

import org.apache.log4j.Level;
import org.apache.log4j.Logger;

import cz.agents.deconfliction.agent.Agent;
import cz.agents.deconfliction.agent.ShortestTrajectoryAgent;
import cz.agents.deconfliction.configuration.Parameters;
import cz.agents.deconfliction.creator.DeconflictionCreator;
import cz.agents.deconfliction.maneuvergraph.EightWayConstantSpeedGridGraph;
import cz.agents.deconfliction.maneuvergraph.FourWayConstantSpeedGridGraph;
import org.jgrapht.Graph;
import cz.agents.deconfliction.maneuvergraph.RandomWaypointGraph;
import cz.agents.deconfliction.solver.central.DeconflictionProblemSolver;
import cz.agents.deconfliction.util.AgentMissionGenerator.Mission;
import cz.agents.alite.trajectorytools.util.OrientedPoint;
import cz.agents.alite.trajectorytools.util.Waypoint;

public class ODIDCustomCreator extends DeconflictionCreator {

    private Graph<Waypoint, SpatialManeuver> maneuvers;
    private Mission[] agentMissions;
    private long runtimeMs;
    private Parameters params;
    private double nominalCost;
    private double finalCost;


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
        DeconflictionProblemSolver.solveUsingID(getProblem(), getManeuvers(), getParams().APPROXIMATION_SAMPLING_INTERVAL);
        LOGGER.info("Runtime: "+(System.nanoTime()-startTime)/1000000+"ms.");
        runtimeMs = (System.nanoTime()-startTime)/1000000;
        finalCost = getProblem().getSumOfTrajectories();
    }

    @Override
    protected List<Agent> createAgents() {
        List<Agent> agents = new LinkedList<Agent>();

        for (int i = 0; i < agentMissions.length; i++) {
            agents.add(new ShortestTrajectoryAgent("A" + i, getManeuvers(),
                    0.0, new OrientedPoint(agentMissions[i].start, 0, 1, 0),
                    new OrientedPoint(agentMissions[i].end, 0, 1, 0),
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
    protected Graph<Waypoint, SpatialManeuver> generateGraph<Waypoint, SpatialManeuver>() {
        return maneuvers;
    }

    public long getRuntimeMs() {
        return runtimeMs;
    }

    public double getNominalCost() {
        return nominalCost;
    }

    public double getFinalCost() {
        return finalCost;
    }

}

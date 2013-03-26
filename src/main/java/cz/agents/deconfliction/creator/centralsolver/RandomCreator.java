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
import cz.agents.deconfliction.util.AgentMissionGenerator;
import cz.agents.deconfliction.util.AgentMissionGenerator.Mission;
import cz.agents.alite.trajectorytools.util.OrientedPoint;
import cz.agents.alite.trajectorytools.util.Waypoint;

public class RandomCreator extends DeconflictionCreator {


    private static final int AGENTS = 50;
    private static final int SEED = 9;

    @Override
    protected void startResolution() {
        // Solve centrally
        long startTime = System.nanoTime();
        DeconflictionProblemSolver.solveUsingID(getProblem(), getManeuvers(), getParams().APPROXIMATION_SAMPLING_INTERVAL);
        LOGGER.info("Runtime: "+(System.nanoTime()-startTime)/1000000+"ms.");
    }

    @Override
    protected List<Agent> createAgents() {
        List<Agent> agents = new LinkedList<Agent>();
        Random random = new Random(SEED);
        Mission[] missions = AgentMissionGenerator.generateRandom(random, getManeuvers(), AGENTS, getParams().MAX_X, getParams().MAX_Y, getParams().SEPARATION);
        for (int i = 0; i < AGENTS; i++ ) {
            agents.add(new ShortestTrajectoryAgent("A"+i, getManeuvers(), 0.0, new OrientedPoint(missions[i].start,0,1,0), new OrientedPoint(missions[i].end,0,1,0), getParams().MAX_T, getParams().APPROXIMATION_SAMPLING_INTERVAL));
        }
        return agents;
    }

    @Override
    protected Parameters configureParameters(Parameters params) {
        Logger.getRootLogger().setLevel(Level.TRACE);
        params.GRID_X = 10;
        params.GRID_Y = 10;
        params.SEPARATION = 0.5;
        params.MAX_X = 10;
        params.MAX_Y = 10;
        params.APPROXIMATION_SAMPLING_INTERVAL = 0.1;
        params.VISUALIZE_NEW_ASSIGNMENT = false;
        params.VISUALISE_SEARCH = false;
        params.CANDIDATES = 30;
        params.SPEED = 1.0;
        return params;
    }

    protected void setupAgents() {
        // Create

        List<Agent> agents = createAgents();
        Collections.sort(agents);
        getProblem().setAgents(agents);
    }

    @Override
    protected Graph<Waypoint, SpatialManeuver> generateGraph<Waypoint, SpatialManeuver>() {
        return new FourWayConstantSpeedGridGraph(getParams().MAX_X, getParams().MAX_Y, getParams().GRID_X, getParams().GRID_Y, getParams().SPEED);
        //return new RandomWaypointGraph(getParams().MAX_X, getParams().MAX_Y, 40, 3, 12);
    }




}

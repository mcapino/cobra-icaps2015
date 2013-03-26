package cz.agents.deconfliction.creator.centralsolver;

import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

import org.apache.log4j.Level;
import org.apache.log4j.Logger;
import org.jgrapht.DirectedGraph;

import cz.agents.alite.trajectorytools.graph.spatial.SpatialGridFactory;
import cz.agents.alite.trajectorytools.graph.spatial.maneuvers.SpatialManeuver;
import cz.agents.alite.trajectorytools.util.OrientedPoint;
import cz.agents.alite.trajectorytools.util.Waypoint;
import cz.agents.deconfliction.agent.Agent;
import cz.agents.deconfliction.agent.FastestTrajectoryAgent;
import cz.agents.deconfliction.configuration.Parameters;
import cz.agents.deconfliction.creator.DeconflictionCreator;
import cz.agents.deconfliction.solver.central.DeconflictionProblemSolver;

public class CrossCreator extends DeconflictionCreator {


    @Override
    protected void startResolution() {
        // Solve centrally
        long startTime = System.nanoTime();
        //DeconflictionProblemSolver.solveUsingOD(getProblem(), getManeuvers(), getParams().APPROXIMATION_SAMPLING_INTERVAL, getParams().SPEED, getParams().MAX_T);
        DeconflictionProblemSolver.solveUsingID(getProblem(), getManeuvers(), getParams().APPROXIMATION_SAMPLING_INTERVAL, getParams().SPEED, getParams().MAX_T);

        //DeconflictionProblemSolver.solveUsingCA(getProblem(), getManeuvers(), getParams().MAX_T, getParams().APPROXIMATION_SAMPLING_INTERVAL, getParams().SPEED);

        LOGGER.info("Runtime: "+(System.nanoTime()-startTime)/1000000+"ms.");
    }


    @Override
    protected List<Agent> createAgents() {
        List<Agent> agents = new LinkedList<Agent>();
        agents.add(new FastestTrajectoryAgent("A1", getManeuvers(), 0.0, new OrientedPoint(getParams().MAX_X/2,0,0,0,1,0), new OrientedPoint(getParams().MAX_X/2,getParams().MAX_Y,0,0,1,0), getParams().MAX_T, getParams().APPROXIMATION_SAMPLING_INTERVAL));
        agents.add(new FastestTrajectoryAgent("A2", getManeuvers(), 0.0, new OrientedPoint(getParams().MAX_X/2,getParams().MAX_Y,0,0,1,0), new OrientedPoint(getParams().MAX_X/2,0,0,0,1,0), getParams().MAX_T, getParams().APPROXIMATION_SAMPLING_INTERVAL));
        agents.add(new FastestTrajectoryAgent("A3", getManeuvers(), 0.0, new OrientedPoint(0,getParams().MAX_Y/2,0,0,1,0), new OrientedPoint(getParams().MAX_X,getParams().MAX_Y/2,0,0,1,0), getParams().MAX_T, getParams().APPROXIMATION_SAMPLING_INTERVAL));
        agents.add(new FastestTrajectoryAgent("A4", getManeuvers(), 0.0, new OrientedPoint(getParams().MAX_X,getParams().MAX_Y/2,0,0,1,0), new OrientedPoint(0,getParams().MAX_Y/2,0,0,1,0), getParams().MAX_T, getParams().APPROXIMATION_SAMPLING_INTERVAL));
        return agents;
    }

    @Override
    protected Parameters configureParameters(Parameters params) {
        Logger.getRootLogger().setLevel(Level.TRACE);
        params.GRID_X = 8;
        params.GRID_Y = 8;
        params.MAX_X = 10;
        params.MAX_Y = 10;
        params.SEPARATION = (params.MAX_X/params.GRID_X)*0.8;
        params.APPROXIMATION_SAMPLING_INTERVAL = params.SEPARATION / 4.0;
        params.VISUALIZE_NEW_ASSIGNMENT = false;
        params.VISUALISE_SEARCH = false;
        params.CANDIDATES = 30;
        params.SPEED = 1.0;
        params.MAX_T = (int) ((4*params.MAX_X)/params.SPEED);
        return params;
    }

    @Override
    protected void setupAgents() {
        // Create
        List<Agent> agents = createAgents();
        Collections.sort(agents);
        getProblem().setAgents(agents);
    }

    @Override
    protected DirectedGraph<Waypoint, SpatialManeuver>  generateGraph() {
        return SpatialGridFactory.create8WayUnitStepGridAsDirectedGraph(getParams().GRID_X, getParams().GRID_Y);
    }

}

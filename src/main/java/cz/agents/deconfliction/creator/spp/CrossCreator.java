package cz.agents.deconfliction.creator.spp;

import java.util.LinkedList;
import java.util.List;

import org.apache.log4j.Level;
import org.apache.log4j.Logger;

import cz.agents.deconfliction.agent.Agent;
import cz.agents.deconfliction.agent.BRABTAgent;
import cz.agents.deconfliction.agent.SynchronousPrioritizedPlanningAgent;
import cz.agents.deconfliction.configuration.Parameters;
import cz.agents.deconfliction.creator.DCSPCreator;
import cz.agents.deconfliction.maneuvergraph.EightWayConstantSpeedGridGraph;
import org.jgrapht.Graph;
import cz.agents.alite.trajectorytools.util.OrientedPoint;

public class CrossCreator extends DCSPCreator {


    @Override
    protected List<Agent> createAgents() {
        List<Agent> agents = new LinkedList<Agent>();
        agents.add(new SynchronousPrioritizedPlanningAgent("A1", getManeuvers(), 0.0, new OrientedPoint(getParams().MAX_X/2,0,0,0,1,0), new OrientedPoint(getParams().MAX_X/2,getParams().MAX_Y,0,0,1,0),  getParams().SEPARATION, getParams().MAX_T, getParams().APPROXIMATION_SAMPLING_INTERVAL, getParams().REPLANNING_STRATEGY));
        agents.add(new SynchronousPrioritizedPlanningAgent("A2", getManeuvers(), 0.0, new OrientedPoint(getParams().MAX_X/2,getParams().MAX_Y,0,0,1,0), new OrientedPoint(getParams().MAX_X/2,0,0,0,1,0),  getParams().SEPARATION, getParams().MAX_T, getParams().APPROXIMATION_SAMPLING_INTERVAL, getParams().REPLANNING_STRATEGY));
        agents.add(new SynchronousPrioritizedPlanningAgent("A3", getManeuvers(), 0.0, new OrientedPoint(0,getParams().MAX_Y/2,0,0,1,0), new OrientedPoint(getParams().MAX_X,getParams().MAX_Y/2,0,0,1,0),  getParams().SEPARATION, getParams().MAX_T, getParams().APPROXIMATION_SAMPLING_INTERVAL, getParams().REPLANNING_STRATEGY));
        agents.add(new SynchronousPrioritizedPlanningAgent("A4", getManeuvers(), 0.0, new OrientedPoint(getParams().MAX_X,getParams().MAX_Y/2,0,0,1,0), new OrientedPoint(0,getParams().MAX_Y/2,0,0,1,0),  getParams().SEPARATION, getParams().MAX_T, getParams().APPROXIMATION_SAMPLING_INTERVAL, getParams().REPLANNING_STRATEGY));
        return agents;
    }

    @Override
    protected Parameters configureParameters(Parameters params) {
        Logger.getRootLogger().setLevel(Level.TRACE);
        params.GRID_X = 15;
        params.GRID_Y = 15;
        params.SEPARATION = 0.5;
        params.MAX_X = 10;
        params.MAX_Y = 10;
        params.APPROXIMATION_SAMPLING_INTERVAL = params.SEPARATION/4.0;
        params.VISUALIZE_NEW_ASSIGNMENT = true;
        params.SPEED = 1.0;
        return params;
    }


    @Override
    protected Graph<Waypoint, SpatialManeuver> generateGraph<Waypoint, SpatialManeuver>() {
        return new EightWayConstantSpeedGridGraph(getParams().MAX_X, getParams().MAX_Y, getParams().GRID_X, getParams().GRID_Y, getParams().SPEED);
        //return new RandomWaypointGraph(getParams().MAX_X, getParams().MAX_Y, 40, 3, 12);
    }




}

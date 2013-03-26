package cz.agents.deconfliction.creator.abt;

import java.util.LinkedList;
import java.util.List;

import org.apache.log4j.Level;
import org.apache.log4j.Logger;

import cz.agents.deconfliction.agent.Agent;
import cz.agents.deconfliction.agent.CommunicatingAgent;
import cz.agents.deconfliction.agent.DCSPAgent;
import cz.agents.deconfliction.configuration.Parameters;
import cz.agents.deconfliction.creator.DCSPCreator;
import cz.agents.deconfliction.maneuvergraph.EightWayConstantSpeedGridGraph;
import cz.agents.deconfliction.maneuvergraph.FourWayConstantSpeedGridGraph;
import cz.agents.deconfliction.maneuvergraph.Maneuver;
import org.jgrapht.Graph;
import cz.agents.deconfliction.maneuvergraph.RandomWaypointGraph;
import cz.agents.alite.trajectorytools.util.OrientedPoint;
import cz.agents.deconfliction.waypointgraph.DefaultWaypointGraph;

public class TestConflictCreator extends DCSPCreator {

    @Override
    protected Parameters configureParameters(Parameters params) {
        Logger.getRootLogger().setLevel(Level.TRACE);
        params.GRID_X = 4;
        params.GRID_Y = 4;
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

    @Override
    protected List<Agent> createAgents() {
        List<Agent> agents = new LinkedList<Agent>();
        agents.add(new DCSPAgent("A1", getManeuvers(), new OrientedPoint(0,0,0,0,1,0), new OrientedPoint(getParams().MAX_X,getParams().MAX_Y,0,0,1,0), 0, getParams()));
        agents.add(new DCSPAgent("A2", getManeuvers(), new OrientedPoint(0,getParams().MAX_Y,0,0,1,0), new OrientedPoint(getParams().MAX_X,0,0,0,1,0), 0, getParams()));
        agents.add(new DCSPAgent("A3", getManeuvers(), new OrientedPoint(getParams().MAX_X/2,0,0,0,1,0), new OrientedPoint(getParams().MAX_X/2,getParams().MAX_Y,0,0,1,0), 0, getParams()));
        agents.add(new DCSPAgent("A4", getManeuvers(), new OrientedPoint(0,getParams().MAX_Y/2,0,0,1,0), new OrientedPoint(getParams().MAX_X,getParams().MAX_Y/2,0,0,1,0), 0, getParams()));
        return agents;
    }

    @Override
    protected Graph<Waypoint, SpatialManeuver> generateGraph<Waypoint, SpatialManeuver>() {
        return new FourWayConstantSpeedGridGraph(getParams().MAX_X, getParams().MAX_Y, getParams().GRID_X, getParams().GRID_Y, getParams().SPEED);
    }
}

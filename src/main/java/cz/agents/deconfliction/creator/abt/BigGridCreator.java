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
import cz.agents.alite.trajectorytools.util.OrientedPoint;

public class BigGridCreator extends DCSPCreator {

    @Override
    protected Parameters configureParameters(Parameters params) {
        Logger.getRootLogger().setLevel(Level.TRACE);
        params.GRID_X = 20;
        params.GRID_Y = 20;
        params.SEPARATION = 100.0;
        params.MAX_X = 1000;
        params.MAX_Y = 1000;
        params.APPROXIMATION_SAMPLING_INTERVAL = 0.2;
        params.VISUALIZE_NEW_ASSIGNMENT = false;
        params.VISUALISE_SEARCH = false;
        params.CANDIDATES = 1;
        params.RESOLVE_CONFLICTS = false;
        return params;
    }

    @Override
    protected List<Agent> createAgents() {
        List<Agent> agents = new LinkedList<Agent>();
        //agents.add(new DCSPAgent("A1", getWaypoints(), new OrientedPoint(0,0,0,0,1,0), new OrientedPoint(1000,1000,0,0,1,0), 0, getParams()));
       // agents.add(new DCSPAgent("A2", getWaypoints(), new OrientedPoint(0,1000,0,0,1,0), new OrientedPoint(1000,0,0,0,1,0), 0, getParams()));
        agents.add(new DCSPAgent("A1", getManeuvers(), new OrientedPoint(500,0,0,0,1,0), new OrientedPoint(500,1000,0,0,1,0), 0, getParams()));
        agents.add(new DCSPAgent("A2", getManeuvers(), new OrientedPoint(0,500,0,0,1,0), new OrientedPoint(1000,500,0,0,1,0), 0, getParams()));
        return agents;
    }
}

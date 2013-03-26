package cz.agents.deconfliction.creator.abt;

import java.util.LinkedList;
import java.util.List;

import cz.agents.deconfliction.agent.Agent;
import cz.agents.deconfliction.agent.CommunicatingAgent;
import cz.agents.deconfliction.agent.DCSPAgent;
import cz.agents.deconfliction.configuration.Parameters;
import cz.agents.deconfliction.creator.DCSPCreator;
import cz.agents.deconfliction.maneuvergraph.EightWayConstantSpeedGridGraph;
import cz.agents.deconfliction.maneuvergraph.FourWayConstantSpeedGridGraph;
import cz.agents.deconfliction.maneuvergraph.Maneuver;
import org.jgrapht.Graph;
import cz.agents.alite.trajectorytools.util.OrientedPoint;
import cz.agents.deconfliction.waypointgraph.DefaultWaypointGraph;

public class SixGeeseCreator extends DCSPCreator {

    @Override
    protected Parameters configureParameters(Parameters params) {
        params.GRID_X = 8;
        params.GRID_Y = 8;
        params.SEPARATION = 40.0;
        params.MAX_X = 1000;
        params.MAX_Y = 1000;
        params.APPROXIMATION_SAMPLING_INTERVAL = 0.5;
        params.CANDIDATES = 50;
        params.VISUALISE_SEARCH = false;
        params.VISUALIZE_NEW_ASSIGNMENT = false;
        return params;
    }

    @Override
    protected List<Agent> createAgents() {
        List<Agent> agents = new LinkedList<Agent>();
        agents.add(new DCSPAgent("A1", getManeuvers(), new OrientedPoint(500,0,0,0,1,0), new OrientedPoint(500,1000,0,0,1,0), 0, getParams()));
        agents.add(new DCSPAgent("A2", getManeuvers(), new OrientedPoint(500,100,0,0,1,0), new OrientedPoint(500,1000,0,0,1,0), 0, getParams()));
        agents.add(new DCSPAgent("A3", getManeuvers(), new OrientedPoint(500,250,0,0,1,0), new OrientedPoint(500,1000,0,0,1,0), 0, getParams()));
        //agents.add(new DCSPAgent("A4", getWaypoints(), new OrientedPoint(500,300,0,0,1,0), new OrientedPoint(500,1000,0,0,1,0), SPEED, SEPARATION, APPROXIMATION_TIME_STEP, APPROXIMATION_MAX_TIME));

        agents.add(new DCSPAgent("B1", getManeuvers(), new OrientedPoint(0,500,0,0,1,0), new OrientedPoint(1000,500,0,0,1,0), 0, getParams()));
        agents.add(new DCSPAgent("B2", getManeuvers(), new OrientedPoint(100,500,0,0,1,0), new OrientedPoint(1000,500,0,0,1,0), 0, getParams()));
        agents.add(new DCSPAgent("B3", getManeuvers(), new OrientedPoint(250,500,0,0,1,0), new OrientedPoint(1000,500,0,0,1,0), 0, getParams()));
        //agents.add(new DCSPAgent("B4", getWaypoints(), new OrientedPoint(300,500,0,0,1,0), new OrientedPoint(1000,500,0,0,1,0), SPEED, SEPARATION, APPROXIMATION_TIME_STEP, APPROXIMATION_MAX_TIME));
        return agents;
    }

    @Override
    protected Graph<Waypoint, SpatialManeuver> generateGraph<Waypoint, SpatialManeuver>() {
        return new EightWayConstantSpeedGridGraph(getParams().MAX_X, getParams().MAX_Y, getParams().GRID_X, getParams().GRID_Y, getParams().SPEED);
    }



}

package cz.agents.deconfliction.creator.brabt;

import java.text.DecimalFormat;
import java.util.Collections;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Random;
import java.util.Set;

import org.apache.log4j.Level;
import org.apache.log4j.Logger;

import cz.agents.alite.planner.general.Problem;
import cz.agents.deconfliction.agent.Agent;
import cz.agents.deconfliction.agent.BRABTAgent;
import cz.agents.deconfliction.agent.ShortestTrajectoryAgent;
import cz.agents.deconfliction.configuration.Parameters;
import cz.agents.deconfliction.creator.DCSPCreator;
import cz.agents.deconfliction.creator.DeconflictionCreator;
import cz.agents.deconfliction.maneuvergraph.EightWayConstantSpeedGridGraph;
import cz.agents.deconfliction.maneuvergraph.FourWayConstantSpeedGridGraph;
import org.jgrapht.Graph;
import cz.agents.deconfliction.maneuvergraph.RandomWaypointGraph;
import cz.agents.deconfliction.solver.central.DeconflictionProblemSolver;
import cz.agents.alite.trajectorytools.trajectory.Trajectory;
import cz.agents.deconfliction.util.AgentMissionGenerator.Mission;
import cz.agents.alite.trajectorytools.util.OrientedPoint;
import cz.agents.alite.trajectorytools.util.Waypoint;

public class BRABTCustomCreator extends DCSPCreator {

    private Graph<Waypoint, SpatialManeuver> maneuvers;
    private Mission[] agentMissions;
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
        super.startResolution();
        finalCost = getProblem().getSumOfTrajectories();
    }

    @Override
    protected List<Agent> createAgents() {
        List<Agent> agents = new LinkedList<Agent>();

        for (int i = 0; i < agentMissions.length; i++) {
            agents.add(new BRABTAgent("A"+ new DecimalFormat("00").format(i), getManeuvers(), 0.0, new OrientedPoint(agentMissions[i].start, 0, 1, 0), new OrientedPoint(agentMissions[i].end, 0, 1, 0), getParams()));
        }
        return agents;
    }

    @Override
    protected Parameters configureParameters(Parameters params) {
        return this.params;
    }

    @Override
    protected Graph<Waypoint, SpatialManeuver> generateGraph<Waypoint, SpatialManeuver>() {
        return maneuvers;
    }

    public double getNominalCost() {
        return nominalCost;
    }

    public double getSolutionCost() {
        return finalCost;
    }
    
    public double getAggregateActiveRuntime() {
    	return getConcurrentSimulation().getCumulativeActiveRuntime() / 1e9d;
    }
    
    public double getAggregateIdleRuntime() {
    	return getConcurrentSimulation().getCumulativeIdleRuntime() / 1e9d;
    }
    
    public double getWallclockRuntime() {
    	return getConcurrentSimulation().getWallclockRuntime() / 1e9d;
    }
    
    

    public Trajectory[] getSolutionTrajectories() {
        return getProblem().getTrajectories();
    }
}

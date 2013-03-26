package cz.agents.deconfliction.creator.spp;

import java.io.IOException;

import cz.agents.deconfliction.agent.Agent;
import cz.agents.deconfliction.agent.DecentralizedPrioritizedPlanningAgent;
import cz.agents.deconfliction.configuration.Parameters;
import cz.agents.deconfliction.creator.DCSPCreator;
import cz.agents.deconfliction.creator.SolverResult;
import org.jgrapht.Graph;
import cz.agents.alite.trajectorytools.trajectory.Trajectory;
import cz.agents.deconfliction.util.AgentMissionGenerator.Mission;

public class DPPCreator extends DCSPCreator {
    protected Graph<Waypoint, SpatialManeuver> maneuvers;
    protected Mission[] agentMissions;
    protected Parameters params;
    protected double nominalCost;
    protected double finalCost;


    public void create(Graph<Waypoint, SpatialManeuver> maneuvers, Mission[] agentMissions, Parameters params) {
        this.maneuvers = maneuvers;
        this.agentMissions = agentMissions;
        this.params = params;

        create();
    }

    @Override
    protected void startResolution() {
        try {
            System.in.read();
        } catch (IOException e1) {
            e1.printStackTrace();
        }
        nominalCost = getProblem().getSumOfTrajectories();
        super.startResolution();
        finalCost = getProblem().getSumOfTrajectories();
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

    public SolverResult getResult() {
        return new SolverResult(finalCost, getWallclockRuntime(), getAggregateActiveRuntime(), 0, 0);
    }

    public int getBroadcastMessageCounter() {
        int result = 0;
        for (Agent agent : getProblem().getAgents()) {
            assert(agent instanceof DecentralizedPrioritizedPlanningAgent);
            result += ((DecentralizedPrioritizedPlanningAgent) agent).getBroadcastMessageCounter();
        }

        return result;
    }

    public int getPlannerInvovationCounter() {
        int result = 0;
        for (Agent agent: getProblem().getAgents()) {
            result += ((DecentralizedPrioritizedPlanningAgent) agent).plannerInvocationCounter;
        }
        return result;
    }

    public double getAvgConstraintsPerPlannerInvocation() {
        int constraints = 0;
        int invocations = 0;

        for (Agent agent: getProblem().getAgents()) {
            invocations += ((DecentralizedPrioritizedPlanningAgent) agent).getPlannerInvocationCounter();
            constraints += ((DecentralizedPrioritizedPlanningAgent) agent).getPlannerConstraintCounter();
        }
        return (double) constraints / (double) invocations;
    }


    public double getPlannerCumulativeRuntimeSec(){
        long result = 0;
        for (Agent agent: getProblem().getAgents()) {
            result += ((DecentralizedPrioritizedPlanningAgent) agent).plannerCumulativeRuntime;
        }
        return result / 1e9d;
    }

    public long getPlannerCumulativeExpandedStates(){
        long result = 0;
        for (Agent agent: getProblem().getAgents()) {
            result += ((DecentralizedPrioritizedPlanningAgent) agent).plannerCumulativeExpandedStates;
        }
        return result;
    }

    public double getCommunicationCumulativeRuntime(){
        long result = 0;
        for (Agent agent: getProblem().getAgents()) {
            result += ((DecentralizedPrioritizedPlanningAgent) agent).communicationCumulativeRuntime;
        }
        return result / 1e9d;
    }

    public double getConflictCheckingCumulativeRuntime(){
        long result = 0;
        for (Agent agent: getProblem().getAgents()) {
            result += ((DecentralizedPrioritizedPlanningAgent) agent).conflictCheckingCumulativeRuntime;
        }
        return result / 1e9d;
    }

    public long getConflictCheckCumulativeCounter(){
        long result = 0;
        for (Agent agent: getProblem().getAgents()) {
            result += ((DecentralizedPrioritizedPlanningAgent) agent).conflictCheckCounter;
        }
        return result;
    }
}

package cz.agents.deconfliction.creator.spp;

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
import cz.agents.deconfliction.agent.DecentralizedPrioritizedPlanningAgent;
import cz.agents.deconfliction.agent.FastestTrajectoryAgent;
import cz.agents.deconfliction.agent.SynchronousPrioritizedPlanningAgent;
import cz.agents.deconfliction.configuration.Parameters;
import cz.agents.deconfliction.creator.DCSPCreator;
import cz.agents.deconfliction.creator.DeconflictionCreator;
import cz.agents.deconfliction.creator.SolverResult;
import cz.agents.deconfliction.maneuvergraph.EightWayConstantSpeedGridGraph;
import cz.agents.deconfliction.maneuvergraph.FourWayConstantSpeedGridGraph;
import org.jgrapht.Graph;
import cz.agents.deconfliction.maneuvergraph.RandomWaypointGraph;
import cz.agents.deconfliction.solver.central.DeconflictionProblemSolver;
import cz.agents.alite.trajectorytools.trajectory.Trajectory;
import cz.agents.deconfliction.util.GraphAgentMissionGenerator.Mission;
import cz.agents.alite.trajectorytools.util.OrientedPoint;
import cz.agents.alite.trajectorytools.util.Waypoint;

public class SPPCustomCreator extends DPPCreator {
    @Override
    protected List<Agent> createAgents() {
        List<Agent> agents = new LinkedList<Agent>();

        for (int i = 0; i < agentMissions.length; i++) {
            agents.add(new SynchronousPrioritizedPlanningAgent("A" + new DecimalFormat("00").format(i),
                    getManeuvers(),
                    0.0,
                    new OrientedPoint(agentMissions[i].start, 0, 1, 0),
                    new OrientedPoint(agentMissions[i].end, 0, 1, 0),
                    getParams().SEPARATION,
                    getParams().MAX_T,
                    getParams().APPROXIMATION_SAMPLING_INTERVAL,
                    getParams().REPLANNING_STRATEGY));
        }
        return agents;
    }

    public int getRoundCounter() {
        return ((SynchronousPrioritizedPlanningAgent) getProblem().getAgents().get(getProblem().getAgents().size()-1)).getRound();
    }
}

package cz.agents.deconfliction.creator.iapp;

import java.text.DecimalFormat;
import java.util.LinkedList;
import java.util.List;

import cz.agents.alite.trajectorytools.util.OrientedPoint;
import cz.agents.deconfliction.agent.Agent;
import cz.agents.deconfliction.agent.InterruptibleAsynchronousPrioritizedPlanningAgent;
import cz.agents.deconfliction.agent.InterruptibleAsynchronousPrioritizedPlanningAgent.InterruptingStrategy;
import cz.agents.deconfliction.creator.spp.DPPCreator;

public class IAPPCustomCreator extends DPPCreator {

    InterruptingStrategy interruptionStrategy;



    public IAPPCustomCreator(InterruptingStrategy interruptionStrategy) {
        super();
        this.interruptionStrategy = interruptionStrategy;
    }



    @Override
    protected List<Agent> createAgents() {
        List<Agent> agents = new LinkedList<Agent>();

        for (int i = 0; i < agentMissions.length; i++) {
            agents.add(new InterruptibleAsynchronousPrioritizedPlanningAgent("A" + new DecimalFormat("00").format(i),
                    getManeuvers(),
                    0.0,
                    new OrientedPoint(agentMissions[i].start, 0, 1, 0),
                    new OrientedPoint(agentMissions[i].end, 0, 1, 0),
                    getParams().SEPARATION,
                    getParams().MAX_T,
                    getParams().APPROXIMATION_SAMPLING_INTERVAL,
                    getParams().REPLANNING_STRATEGY, getConcurrentSimulation()));
        }
        return agents;
    }
}

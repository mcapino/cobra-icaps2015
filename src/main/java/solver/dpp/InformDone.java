package cz.agents.deconfliction.solver.dpp;

import cz.agents.alite.communication.content.Content;

public class InformDone extends Content {
    final String agentName;
    final int round;

    public InformDone(String agentName, int round) {
        super(round);
        this.agentName = agentName;
        this.round = round;
    }


    public synchronized String getAgentName() {
        return agentName;
    }

    public int getRound() {
        return round;
    }


    @Override
    public String toString() {
        return "InformDone [agentName=" + agentName + ", round=" + round + "]";
    }

}

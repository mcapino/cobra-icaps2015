package cz.agents.admap.msg;

import cz.agents.alite.communication.content.Content;

public class InformAgentFinishedRound extends Content {
    final String agentName;
    final int round;
    
    public InformAgentFinishedRound(String agentName, int round) {
        super(null);
        this.agentName = agentName;
        this.round = round;
    }

    public String getAgentName() {
        return agentName;
    }
    
    public int getRound() {
		return round;
	}

	@Override
	public String toString() {
		return "InformAgentFinishedRound [agentName=" + agentName + ", round="
				+ round + "]";
	}
    
}

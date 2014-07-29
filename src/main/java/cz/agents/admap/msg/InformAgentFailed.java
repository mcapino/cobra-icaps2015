package cz.agents.admap.msg;

import cz.agents.alite.communication.content.Content;

public class InformAgentFailed extends Content {
    final String agentName;

    public InformAgentFailed(String agentName) {
        super(null);
        this.agentName = agentName;
    }

    public String getAgentName() {
        return agentName;
    }

	@Override
	public String toString() {
		return "InformAgentFailed [agentName=" + agentName + "]";
	}
}

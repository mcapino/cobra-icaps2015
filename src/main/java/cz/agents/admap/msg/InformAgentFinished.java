package cz.agents.admap.msg;

import cz.agents.alite.communication.content.Content;

public class InformAgentFinished extends Content {
    final String agentName;

    public InformAgentFinished(String agentName) {
        super(null);
        this.agentName = agentName;
    }

    public String getAgentName() {
        return agentName;
    }

	@Override
	public String toString() {
		return "InformAgentFinished [agentName=" + agentName + "]";
	}
}

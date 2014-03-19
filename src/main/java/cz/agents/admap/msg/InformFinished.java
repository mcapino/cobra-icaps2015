package cz.agents.admap.msg;

import cz.agents.alite.communication.content.Content;

public class InformFinished extends Content {
    final String agentName;

    public InformFinished(String agentName) {
        super(null);
        this.agentName = agentName;
    }

    public String getAgentName() {
        return agentName;
    }

	@Override
	public String toString() {
		return "InformFinished [agentName=" + agentName + "]";
	}

}

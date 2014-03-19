package cz.agents.admap.msg;

import tt.euclidtime3i.Region;
import cz.agents.alite.communication.content.Content;

public class InformNewTrajectory extends Content {
    final String agentName;
    final tt.euclidtime3i.Region region;

    public InformNewTrajectory(String agentName, Region region) {
        super(null);
        this.agentName = agentName;
        this.region = region;
    }

    public String getAgentName() {
        return agentName;
    }

    public tt.euclidtime3i.Region getRegion() {
        return region;
    }

	@Override
	public String toString() {
		return "InformNewTrajectory [agentName=" + agentName + ", region="
				+ region + "" + "]";
	}




}

package cz.agents.admap.msg;

import tt.euclidtime3i.Region;
import cz.agents.alite.communication.content.Content;

public class InformNewTrajectory extends Content {
    final String agentName;
    final tt.euclidtime3i.Region region;
    final double weight;

    public InformNewTrajectory(String agentName, Region region) {
        super(null);
        this.agentName = agentName;
        this.region = region;
        this.weight = Double.POSITIVE_INFINITY;
    }
    
    public InformNewTrajectory(String agentName, Region region, double weight) {
        super(null);
        this.agentName = agentName;
        this.region = region;
        this.weight = weight;
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
				+ region + ", weight=" + weight + "]";
	}

	public double getWeight() {
		return weight;
	}
}

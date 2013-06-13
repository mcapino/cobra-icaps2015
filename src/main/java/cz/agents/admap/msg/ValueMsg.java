package cz.agents.admap.msg;

import tt.euclidtime3i.Region;

public class ValueMsg extends InformNewTrajectory {

    public ValueMsg(String agentName, Region region) {
        super(agentName, region);
    }

    @Override
    public String toString() {
        return "Value [agentName=" + agentName + ", region=" + region + "]";
    }

}

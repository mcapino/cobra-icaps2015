package cz.agents.admap.msg;

import tt.euclidtime3i.Region;
import cz.agents.admap.agent.adopt.Context;
import cz.agents.alite.communication.content.Content;

public class CostMsg extends Content {
    final String agentName;
    final tt.euclidtime3i.Region occupiedRegion;
    final Context context;
    final double lb;
    final double ub;

    public CostMsg(String agentName, Region occupiedRegion,
            Context context, double lb, double ub) {
        super(null);
        this.agentName = agentName;
        this.occupiedRegion = occupiedRegion;
        this.context = context;
        this.lb = lb;
        this.ub = ub;
    }

    public String getAgentName() {
        return agentName;
    }

    public tt.euclidtime3i.Region getOccupiedRegion() {
        return occupiedRegion;
    }

    public Context getContext() {
        return context;
    }

    public double getLb() {
        return lb;
    }

    public double getUb() {
        return ub;
    }

    @Override
    public String toString() {
        return "Cost [agentName=" + agentName + ", occupiedRegion="
                + occupiedRegion + ", context=" + context + ", lb=" + lb
                + ", ub=" + ub + "]";
    }
}

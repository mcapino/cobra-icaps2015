package cz.agents.deconfliction.agent;

import cz.agents.deconfliction.trajectory.Trajectory;
import cz.agents.deconfliction.util.OrientedPoint;

/**
 * Enables gathering of statistics about the exploitation
 * of computational power of each agent. Allow to compute how
 * long would system take to come up with a solution if the agents
 * were executed in parallel.
 */
public abstract class ComputationalAgent extends Agent {

    private static final long INACTIVE = (-1);

    private long activityStartedSystemTime = INACTIVE;
    private long totalActiveTimeNs = 0;
    private long toalIdleTimeNs = 0;

    public ComputationalAgent(String name, OrientedPoint start,
            double startTime, OrientedPoint destination) {
        super(name, start, startTime, destination);
    }

    protected void activityStarted(long idlePeriod) {
        activityStartedSystemTime = System.nanoTime();
        toalIdleTimeNs += idlePeriod;
    }

    protected void activityFinished() {
        totalActiveTimeNs = System.nanoTime() - activityStartedSystemTime;
        activityStartedSystemTime = 0;
    }

    public long getAgentsRelativeTimeNs() {
        if (activityStartedSystemTime == INACTIVE) {
            return totalActiveTimeNs + toalIdleTimeNs;
        } else {
            return totalActiveTimeNs + toalIdleTimeNs + (System.nanoTime() - activityStartedSystemTime);
        }
    }
}

package cz.agents.admap.agent;

import tt.euclid2i.EvaluatedTrajectory;
import tt.euclid2i.Point;
import tt.euclid2i.probleminstance.Environment;

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

	EvaluatedTrajectory trajectory;

    public ComputationalAgent(String name, Point start, Point goal,
            Environment environment, int agentSizeRadius) {
        super(name, start, goal, environment, agentSizeRadius);
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

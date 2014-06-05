package cz.agents.admap.agent.adopt;

import java.util.Collection;
import java.util.Collections;
import java.util.Random;

import tt.euclid2i.EvaluatedTrajectory;
import tt.euclid2i.Point;
import tt.euclid2i.region.Rectangle;
import cz.agents.admap.agent.BestResponse;

public class FeasibleTrajectoriesDomain {
    Collection<tt.euclid2i.Region> inflatedSpaceObstacles;
    Collection<tt.euclidtime3i.Region> inflatedSpacetimeObstacles;

    private Point start;
    private Point goal;

    EvaluatedTrajectory shortestTraj;
    Rectangle bounds;
    private boolean shortestTrajExamined;
    private Random random;
	private int maxTime;

    public FeasibleTrajectoriesDomain(
            Point start, Point goal,
            Collection<tt.euclid2i.Region> inflatedSpaceObstacles,
            Collection<tt.euclidtime3i.Region> inflatedSpacetimeObstacles,
            Rectangle bounds, int maxTime, int waitMoveDuration, Random random) {
        super();
        this.inflatedSpaceObstacles = inflatedSpaceObstacles;
        this.inflatedSpacetimeObstacles = inflatedSpacetimeObstacles;
        this.start = start;
        this.goal = goal;
        this.bounds = bounds;
        this.shortestTraj = BestResponse.computeBestResponse(start, goal, inflatedSpaceObstacles, bounds, Collections.EMPTY_SET, inflatedSpacetimeObstacles, maxTime, waitMoveDuration);
        this.shortestTrajExamined = false;
        this.random = random;
        this.maxTime = maxTime;
    }

    public EvaluatedTrajectory getNewTrajectory(double maxCost) {
        if (!shortestTrajExamined) {
            shortestTrajExamined = true;
            return shortestTraj;
        } else {
            EvaluatedTrajectory randomTraj
                = BestResponse.computeRandomRoute(start, goal, inflatedSpaceObstacles, bounds, inflatedSpacetimeObstacles, random, maxTime);
            assert randomTraj != null;
            return randomTraj;
        }

    }

    public double getShortestTrajCost() {
        if (shortestTraj == null) {
            return Double.POSITIVE_INFINITY;
        } else {
            return shortestTraj.getCost();
        }
    }

    public boolean isEmpty() {
        return shortestTraj == null;
    }

}

package cz.agents.admap.agent.adopt;

import tt.euclidtime3i.Region;
import tt.euclidtime3i.region.MovingCircle;
import tt.euclidtime3i.util.IntersectionChecker;

public class NotCollidingConstraint implements Constraint {

    @Override
    public double getCost(Region t1, Region t2) {
        assert t1 instanceof MovingCircle && t2 instanceof MovingCircle;

        if (IntersectionChecker.intersect(t1, t2)) {
            return Double.POSITIVE_INFINITY;
        } else {
            return 0;
        }
    }

}

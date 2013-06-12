package cz.agents.admap.agent.adopt;

import java.util.Collection;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;

import tt.euclid2i.EvaluatedTrajectory;


public class Bounds {
    Map<EvaluatedTrajectory, Collection<ChildBounds>> valueBounds = new HashMap<EvaluatedTrajectory, Collection<ChildBounds>>();

    public double getUpperBound(LocalCost localCost) {
        double upperBound = Double.POSITIVE_INFINITY;
        for (EvaluatedTrajectory value : valueBounds.keySet()) {
            double boundOfThisValue = getUpperBoundOf(localCost, value);
            if (boundOfThisValue < upperBound) {
                upperBound = boundOfThisValue;
            }
        }
        return upperBound;
    }

    public double getLowerBound(LocalCost localCost) {
        double lowerBound = Double.POSITIVE_INFINITY;
        for (EvaluatedTrajectory value : valueBounds.keySet()) {
            double boundOfThisValue = getLowerBoundOf(localCost, value);
            if (boundOfThisValue < lowerBound) {
                lowerBound = boundOfThisValue;
            }
        }
        return lowerBound;
    }

    public double getUpperBoundOf(LocalCost localCost, EvaluatedTrajectory value) {
        Collection<ChildBounds> subtrees = valueBounds.get(value);
        if (subtrees != null) {
            //  we have bounds for this values
            // TODO: it is still possible that we don't have all the children yet
            double cost = localCost.getCost(value);
            for (ChildBounds subtree : subtrees) {
                cost += subtree.upperBound;
            }
            return cost;
        } else {
            // we dont have bounds for this value
            return Double.POSITIVE_INFINITY;
        }
    }

    public double getLowerBoundOf(LocalCost localCost, EvaluatedTrajectory value) {
        Collection<ChildBounds> subtrees = valueBounds.get(value);
        if (subtrees != null) {
            // we have bounds for this values
            // TODO: it is still possible that we don't have all the children yet
            double cost = localCost.getCost(value);
            for (ChildBounds subtree : subtrees) {
                cost += subtree.lowerBound;
            }
            return cost;
        } else {
            // we dont have bounds for this value
            return 0;
        }
    }

    public void resetAfterContextChange(Context context) {
        for (Collection<ChildBounds> childBoundsCollection : valueBounds.values()) {
            assert childBoundsCollection != null;
            Collection<ChildBounds> childBoundsCollectionCopy = new LinkedList<ChildBounds>(childBoundsCollection);
            for (ChildBounds bounds : childBoundsCollectionCopy) {
                if (!bounds.context.compatibleWith(context)) {
                    childBoundsCollection.remove(bounds);
                }
            }
        }
    }
}

package cz.agents.admap.agent.adopt;

import java.util.Collection;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;

import tt.euclid2i.EvaluatedTrajectory;


public class ValueBounds {
    Map<EvaluatedTrajectory,  Map<String, ChildBounds>> valueBounds = new HashMap<EvaluatedTrajectory, Map<String, ChildBounds>>();

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
        Map<String, ChildBounds> subtrees = valueBounds.get(value);
        if (subtrees != null) {
            //  we have bounds for this values
            // TODO: it is still possible that we don't have all the children yet
            double cost = localCost.getCost(value);
            for (ChildBounds subtree : subtrees.values()) {
                cost += subtree.upperBound;
            }
            return cost;
        } else {
            // we dont have bounds for this value
            return Double.POSITIVE_INFINITY;
        }
    }

    public double getLowerBoundOf(LocalCost localCost, EvaluatedTrajectory value) {
        Map<String, ChildBounds> subtrees = valueBounds.get(value);
        double cost = localCost.getCost(value);

        if (subtrees != null) {
            // we have bounds for this values
            // TODO: it is still possible that we don't have all the children yet
            for (ChildBounds subtree : subtrees.values()) {
                cost += subtree.lowerBound;
            }
        }

        return cost;
    }

    public void resetAfterContextChange(Context context) {
        for (Map<String, ChildBounds> childBoundsMap : valueBounds.values()) {
            if (childBoundsMap != null) {
                Collection<String> keys = new LinkedList<String>(childBoundsMap.keySet());
                for (String key : keys) {
                    if (!childBoundsMap.get(key).context.compatibleWith(context)) {
                        childBoundsMap.remove(key);
                    }
                }
            }
        }
    }

    public int size() {
        return valueBounds.size();
    }

    public void set(EvaluatedTrajectory value, String agentName, double lb, double ub, double threshold, Context context) {
        if (!valueBounds.containsKey(value) || valueBounds.get(value) == null) {
            valueBounds.put(value, new HashMap<String,ChildBounds>());
        }
        valueBounds.get(value).put(agentName, new ChildBounds(lb, ub, threshold, context)) ;
    }

    public void introduceNewValue(EvaluatedTrajectory value) {
        valueBounds.put(value, null);
    }

    @Override
    public String toString() {
        return valueBounds.toString();
    }
}

package cz.agents.admap.agent.adopt;

import java.util.Collection;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;

import tt.euclid2i.EvaluatedTrajectory;


public class ValueBounds {
    Map<EvaluatedTrajectory,  Map<String, ChildBounds>> valueBounds = new HashMap<EvaluatedTrajectory, Map<String, ChildBounds>>();
    Collection<String> children = new LinkedList<String>();

    public void updateChildren(Collection<String> updatedChildren) {
        children = updatedChildren;
        for (Map<String, ChildBounds> childBounds : valueBounds.values()) {
            for (String newChild : children) {
                if (!childBounds.containsKey(newChild)) {
                    childBounds.put(newChild, new ChildBounds());
                }
            }
        }
    }

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
        if (valueBounds.isEmpty()) {
            return Double.NaN;
        }
        else {
            double lowerBound = Double.POSITIVE_INFINITY;
            for (EvaluatedTrajectory value : valueBounds.keySet()) {
                double boundOfThisValue = getLowerBoundOf(localCost, value);
                if (boundOfThisValue < lowerBound) {
                    lowerBound = boundOfThisValue;
                }
            }
            return lowerBound;
        }
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

    public void resetAfterContextChange(Context newContext) {
        Collection<EvaluatedTrajectory> valuesToRemove = new LinkedList<EvaluatedTrajectory>();
        for (EvaluatedTrajectory value : valueBounds.keySet()) {
            Map<String, ChildBounds> childBoundsMap = valueBounds.get(value);
            if (childBoundsMap != null) {
                Collection<String> keys = new LinkedList<String>(childBoundsMap.keySet());
                for (String key : keys) {
                    if (!childBoundsMap.get(key).context.compatibleWith(newContext)) {
                        childBoundsMap.remove(key);
                    }
                }

                if (childBoundsMap.isEmpty()) {
                    valuesToRemove.add(value);
                }
            } else {
                valuesToRemove.add(value);
            }
        }

        for (EvaluatedTrajectory valueToRemove : valuesToRemove) {
            valueBounds.remove(valueToRemove);
        }
    }

    public int size() {
        return valueBounds.size();
    }

    public void set(EvaluatedTrajectory value, String agentName, double lb, double ub, double threshold, Context context) {
        if (!valueBounds.containsKey(value) || valueBounds.get(value) == null) {
            valueBounds.put(value, new HashMap<String,ChildBounds>());
        }
        valueBounds.get(value).put(agentName, new ChildBounds(lb, ub, threshold, new Context(context))) ;
    }

    public void introduceNewValue(EvaluatedTrajectory value) {

        Map<String, ChildBounds> childBounds = new HashMap<String, ChildBounds>();

        for (String newChild : children) {
            childBounds.put(newChild, new ChildBounds());
        }

        valueBounds.put(value, childBounds);
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        for (EvaluatedTrajectory value : valueBounds.keySet()) {
            sb.append(Integer.toHexString(value.hashCode()) + " " + valueBounds.get(value) + "\n");
        }

        return sb.toString();
    }

    public String toString(LocalCost localCost) {
        StringBuilder sb = new StringBuilder();
        for (EvaluatedTrajectory value : valueBounds.keySet()) {
            sb.append(Integer.toHexString(value.hashCode()) + " (" + getLowerBoundOf(localCost, value) + ", " + getUpperBoundOf(localCost, value) + ")" + valueBounds.get(value) + "\n");
        }

        return sb.toString();
    }

    public boolean isEmpty() {
        return valueBounds.isEmpty();
    }
}

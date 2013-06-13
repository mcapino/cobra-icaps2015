package cz.agents.admap.agent.adopt;

import java.util.Collection;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;

import tt.euclidtime3i.Region;
import tt.euclidtime3i.region.MovingCircle;

public class Context {
    Map<String, tt.euclidtime3i.Region> variables = new HashMap<String, tt.euclidtime3i.Region>();

    public void put(String var, tt.euclidtime3i.Region traj) {
        variables.put(var, traj);
    }

    public tt.euclidtime3i.Region get(String agenName) {
        return variables.get(agenName);
    }

    public Set<String> vars() {
        return variables.keySet();
    }

    public boolean contains(String var) {
        return variables.containsKey(var);
    }

    public Collection<tt.euclidtime3i.Region> getOccupiedRegions(int enlargeBy) {
        Collection<tt.euclidtime3i.Region> avoidRegions = new LinkedList<tt.euclidtime3i.Region>();
        for (tt.euclidtime3i.Region region : variables.values()) {
            assert region instanceof MovingCircle;
            MovingCircle movingCircle =  (MovingCircle) region;
            avoidRegions.add(new MovingCircle(movingCircle.getTrajectory(), movingCircle.getRadius() + enlargeBy));
        }
        return avoidRegions;
    }

    // two contexts are compatible if they agree on the variables they share
    public boolean compatibleWith(Context context) {

        for (Map.Entry<String, tt.euclidtime3i.Region> entry : variables.entrySet()) {
            if (context.contains(entry.getKey())) {
                if (!context.get(entry.getKey()).equals(entry.getValue())) {
                    return false;
                }
            }
        }

        return true;
    }

    public void removeVar(String varName) {
        variables.remove(varName);
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append("{");
        for (Entry<String, Region> entry : variables.entrySet()) {
            sb.append(entry.getKey() + ":" + entry.getValue() + " ");
        }
        sb.append("}");
        return sb.toString();
    }
}

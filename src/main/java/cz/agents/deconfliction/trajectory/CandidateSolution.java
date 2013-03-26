package cz.agents.deconfliction.trajectory;

import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

import cz.agents.alite.trajectorytools.trajectory.Trajectory;

public class CandidateSolution {

    Map<String, Trajectory> trajectories;
    Map<String, Double> costs;

    public CandidateSolution() {
        trajectories = new HashMap<String, Trajectory>();
        costs = new HashMap<String, Double>();

    }

    public CandidateSolution(CandidateSolution solution) {
        trajectories = new HashMap<String, Trajectory>(solution.getTrajectories());
        costs = new HashMap<String, Double>(solution.getCosts());
        assert trajectories.size() == costs.size();
    }


    public void set(String name, Trajectory trajectory) {
        trajectories.put(name, trajectory);
    }

    public Trajectory get(String name) {
        return trajectories.get(name);
    }

    public Map<String, Trajectory> getTrajectories() {
        return trajectories;
    }

    public Map<String, Double> getCosts() {
        return costs;
    }

    public Collection<Trajectory> getTrajectoriesCollection() {
        return trajectories.values();
    }



    @Override
    public boolean equals(Object obj) {
        if (obj instanceof CandidateSolution) {
            CandidateSolution other = (CandidateSolution) obj;
            return trajectories.equals(other.trajectories) &&
                   costs.equals(other.costs);
        }

        return false;
    }

    @Override
    public int hashCode() {
        return trajectories.hashCode();
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        for (String agentName : getAgentNames()) {
            sb.append("[" + agentName + ": (" + String.format("%1$,.2f", getCost(agentName)) +  ") "  + getTrajectory(agentName) + "] ");
        }
        return sb.toString();
    }

    public boolean isEmpty() {
        return trajectories.isEmpty();
    }

    public void remove(String agentName) {
        trajectories.remove(agentName);
    }

    public String getLowestPriorityAgent() {
        return Collections.max(trajectories.keySet());
    }

    public boolean containsHigherPriorityAgent(String myName) {
        for (String agentName : trajectories.keySet()) {
            if (agentName.compareTo(myName) < 0) {
                return true;
            }
        }
        return false;
    }

    public Collection<String> getAgentNames() {
        return trajectories.keySet();
    }

    public double getCost(String agentName) {
        return costs.get(agentName);
    }

    public Trajectory getTrajectory(String agentName) {
        return trajectories.get(agentName);
    }

}

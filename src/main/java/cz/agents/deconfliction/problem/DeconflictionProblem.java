package cz.agents.deconfliction.problem;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import cz.agents.alite.trajectorytools.trajectory.Trajectory;
import cz.agents.alite.trajectorytools.util.SeparationDetector;
import cz.agents.alite.trajectorytools.util.TimePoint;
import cz.agents.deconfliction.agent.Agent;

public class DeconflictionProblem {

    private List<Agent> agents;
    private double separation;

    public DeconflictionProblem(double separation) {
        this.agents = new LinkedList<Agent>();
        this.separation = separation;
    }

    public List<TimePoint> getConflicts() {

        List<Trajectory> trajectories = new ArrayList<Trajectory>();
        for (Agent agent  : getAgents()) {
            if (agent.getCurrentTrajectory() != null) {
                trajectories.add(agent.getCurrentTrajectory());
            }
        }

        return SeparationDetector.computeAllPairwiseConflicts(trajectories, separation, separation/4);
    }

    public void setAgents(List<Agent> agents) {
        this.agents = agents;
    }

    public List<Agent> getAgents() {
        return agents;
    }

    public double getSeparation() {
        return separation;
    }

    public double getSumOfTrajectories() {
        double sum = 0.0;
        for (Agent agent : agents) {
            if (agent.getCurrentTrajectory() != null) {
                sum += agent.getCurrentTrajectory().getCost();
            }
        }
        return sum;
    }

    public Map<String, Trajectory> getAgentsTrajectories() {
        Map<String, Trajectory> trajectories = new HashMap<String, Trajectory>();

        for (Agent agent : agents) {
            trajectories.put(agent.getName(), agent.getCurrentTrajectory());
        }

        return trajectories;
    }

    public Trajectory[] getTrajectories() {

        Trajectory[] result = new Trajectory[agents.size()];

        for (int i = 0; i < agents.size(); i++) {
            result[i] = agents.get(i).getCurrentTrajectory();
        }

        return result;
    }

}

package cz.agents.deconfliction.solver.central;

import java.util.ArrayList;
import java.util.List;

import org.jgrapht.DirectedGraph;

import cz.agents.alite.trajectorytools.graph.spatial.SpatialGraphs;
import cz.agents.alite.trajectorytools.graph.spatial.maneuvers.SpatialManeuver;
import cz.agents.alite.trajectorytools.trajectory.EvaluatedSampledTrajectory;
import cz.agents.alite.trajectorytools.trajectory.EvaluatedTrajectory;
import cz.agents.alite.trajectorytools.util.Waypoint;
import cz.agents.deconfliction.agent.Agent;
import cz.agents.deconfliction.agent.FixedTrajectoryAgent;
import cz.agents.deconfliction.problem.DeconflictionProblem;
import cz.agents.deconfliction.solver.NoSolutionFoundException;

public class DeconflictionProblemSolver {

    public static DeconflictionProblem solveUsingOD(
            DeconflictionProblem problem,
            DirectedGraph<Waypoint, SpatialManeuver> maneuvers,
            double approxSamplingInterval, double vmax, double maxTime) {

        // Solve centrally

        Waypoint[] goals = getGoalWaypoints(problem, maneuvers);
        ODState[] starts = getStartStates(problem, maneuvers);
        ODSolver solver = new ODSolver(starts, goals, maneuvers,
                problem.getSeparation(), maxTime, vmax);

        EvaluatedTrajectory[] trajs = solver.solveTrajectories();

        if (trajs == null) {
            throw new NoSolutionFoundException();
        }

        List<Agent> agents = new ArrayList<Agent>(problem.getAgents().size());

        for (int i=0; i < trajs.length; i++) {

            EvaluatedTrajectory sampledTraj = new EvaluatedSampledTrajectory(trajs[i], approxSamplingInterval);
            Agent agent = new FixedTrajectoryAgent(problem.getAgents().get(i)
                    .getName(), problem.getAgents().get(i).getStart(), problem
                    .getAgents().get(i).getStartTime(), problem.getAgents()
                    .get(i).getDestination(), sampledTraj);
            agents.add(i, agent);
        }
        problem.setAgents(agents);
        return problem;
    }

    public static DeconflictionProblem solveUsingID(DeconflictionProblem problem, DirectedGraph<Waypoint, SpatialManeuver> maneuvers, double approxSamplingInterval, double vmax, double maxTime) {

        // Solve centrally

        Waypoint[] goals = getGoalWaypoints(problem, maneuvers);
        ODState[] starts = getStartStates(problem, maneuvers);
        IDSolver solver = new IDSolver(starts, goals, maneuvers,
                problem.getSeparation(), maxTime, vmax);

        EvaluatedTrajectory[] trajectories = solver.solve();

        if (trajectories == null) {
            throw new NoSolutionFoundException();
        }

        List<Agent> agents = new ArrayList<Agent>(problem.getAgents().size());

        for (int i=0; i < trajectories.length; i++) {
            EvaluatedTrajectory sampledTraj = new EvaluatedSampledTrajectory(trajectories[i], approxSamplingInterval);
            Agent agent = new FixedTrajectoryAgent(problem.getAgents().get(i)
                    .getName(), problem.getAgents().get(i).getStart(), problem
                    .getAgents().get(i).getStartTime(), problem.getAgents()
                    .get(i).getDestination(), sampledTraj);
            agents.add(i, agent);
        }
        problem.setAgents(agents);
        return problem;
    }

    /*
    public static DeconflictionProblem solveUsingCA(DeconflictionProblem problem, DirectedGraph<Waypoint, SpatialManeuver> maneuvers, double maxTime, double approxSamplingInterval, double vmax) {

        // Solve centrally

        ODState[] goals = getGoalStates(problem, maneuvers);
        ODState[] starts = getStartStates(problem, maneuvers);
        CASolver solver = new CASolver(starts, goals, maneuvers, problem.getSeparation(), maxTime, approxSamplingInterval, vmax);
        Trajectory[] trajectories = solver.solve();

        if (trajectories == null) {
            throw new NoSolutionFoundException();
        }

        List<Agent> agents = new ArrayList<Agent>(problem.getAgents().size());

        for (int i=0; i < trajectories.length; i++) {
            Agent agent = new FixedTrajectoryAgent(problem.getAgents().get(i).getName(), problem.getAgents().get(i).getStart(), problem.getAgents().get(i).getStartTime(), problem.getAgents().get(i).getDestination(), trajectories[i]);
            agents.add(i, agent);
        }
        problem.setAgents(agents);
        return problem;
    }*/

    protected static ODState[] getStartStates(DeconflictionProblem problem, DirectedGraph<Waypoint, SpatialManeuver> maneuvers) {
        ODState[] agentStates = new ODState[problem.getAgents().size()];
        int i=0;
        for(Agent agent : problem.getAgents()) {
            Waypoint startWaypoint = SpatialGraphs.getNearestVertex(maneuvers,agent.getStart());
            agentStates[i] = new ODState(startWaypoint, agent.getStartTime(), null, null, 0);
            i++;
        }

        return agentStates;
    }

    protected static Waypoint[] getGoalWaypoints(DeconflictionProblem problem, DirectedGraph<Waypoint, SpatialManeuver> maneuvers) {
        Waypoint[] waypoints = new Waypoint[problem.getAgents().size()];
        int i=0;
        for(Agent agent : problem.getAgents()) {
            Waypoint goalWaypoint = SpatialGraphs.getNearestVertex(maneuvers,agent.getDestination());
            waypoints[i] = goalWaypoint;
            i++;
        }

        return waypoints;
    }
}

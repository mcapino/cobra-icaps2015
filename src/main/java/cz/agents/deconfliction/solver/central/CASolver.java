package cz.agents.deconfliction.solver.central;

import java.util.LinkedList;
import java.util.List;

import org.apache.log4j.Logger;
import org.jgrapht.Graph;

import cz.agents.alite.trajectorytools.graph.spatial.maneuvers.SpatialManeuver;
import cz.agents.alite.trajectorytools.trajectory.Trajectory;
import cz.agents.alite.trajectorytools.util.Waypoint;
import cz.agents.deconfliction.planner4d.Planner4d;
/**
 * A Cooperative A* from Silver 2005 (Cooperative Pathfinding)
 */
public class CASolver extends CentralSolver {

    Logger LOGGER = Logger.getLogger(this.getClass());
    final int nAgents;
    private ODState[] starts;
    private Waypoint[] ends;
    private double maxTime;

    public CASolver(ODState starts[], Waypoint ends[], Graph<Waypoint, SpatialManeuver> maneuvers, double separation, double maxTime, double approxSamplingInterval, double vmax) {

        super(maneuvers, separation, approxSamplingInterval, vmax);

        this.starts = starts;
        this.ends = ends;
        this.maxTime = maxTime;

        if (starts.length != ends.length) {
            throw new IllegalArgumentException("Start waypoints and end waypoints have different length.");
        }

        this.nAgents = starts.length;
    }

    public Trajectory[] solve() {
        Trajectory[] trajectories = new Trajectory[nAgents];

        for (int i = 0; i < trajectories.length; i++) {

            // Get the trajectories of all higher-priority agents
            List<Trajectory> hardConstraints = new LinkedList<Trajectory>();

            for (int j = 0; j < i; j++) {
                if (trajectories[j] != null) {
                    hardConstraints.add(trajectories[j]);
                }
            }

            long start = System.nanoTime();

            Planner4d planner = new Planner4d(starts[i].getCurrentWaypoint(),
                    starts[i].getTime(), ends[i].getCurrentWaypoint(), maneuvers, separation, maxTime,
                    approxSamplingInterval, vmax,
                    hardConstraints, new LinkedList<Trajectory>());

            Trajectory result = planner.solveTrajectory();

            long duration = System.nanoTime() - start;
            LOGGER.debug("CA Solver, step " + i + " planning (" + hardConstraints.size() + " constraints) took " + duration/1e6 + "ms");

                if (result != null) {
                    trajectories[i] = result;
                } else {
                    return null;
                }
        }
        return trajectories;
    }
}

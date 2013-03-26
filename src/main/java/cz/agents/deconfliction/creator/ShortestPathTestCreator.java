package cz.agents.deconfliction.creator;

import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

import org.apache.log4j.Level;
import org.apache.log4j.Logger;
import org.jgrapht.alg.AStarShortestPath;
import org.jgrapht.alg.AStarShortestPath.Heuristic;
import org.jgrapht.alg.DijkstraShortestPath;

import cz.agents.deconfliction.agent.Agent;
import cz.agents.deconfliction.agent.FixedTrajectoryAgent;
import cz.agents.deconfliction.agent.ShortestTrajectoryAgent;
import cz.agents.deconfliction.configuration.Parameters;
import cz.agents.deconfliction.maneuvergraph.EightWayConstantSpeedGridGraph;
import cz.agents.deconfliction.maneuvergraph.FourWayConstantSpeedGridGraph;
import cz.agents.deconfliction.maneuvergraph.Maneuver;
import org.jgrapht.Graph;
import cz.agents.deconfliction.maneuvergraph.RandomWaypointGraph;
import cz.agents.deconfliction.solver.central.DeconflictionProblemSolver;
import cz.agents.deconfliction.trajectory.PiecewiseLinearTrajectory;
import cz.agents.alite.trajectorytools.util.OrientedPoint;
import cz.agents.deconfliction.util.Point;
import cz.agents.alite.trajectorytools.util.Waypoint;

public class ShortestPathTestCreator extends DeconflictionCreator {

    @Override
    protected void startResolution() {
        //Solve centrally
        //DeconflictionProblemSolver.solve(getProblem(), getManeuvers(), getParams().APPROXIMATION_TIME_STEP);
    }

    @Override
    protected List<Agent> createAgents() {
        Waypoint start = getManeuvers().getNearestWaypoint(new Point(0, 0, 0));
        Waypoint end = getManeuvers().getNearestWaypoint(new Point(5.0, 5.0, 0));

        DijkstraShortestPath<Waypoint, SpatialManeuver> dijsktra = new DijkstraShortestPath<Waypoint, SpatialManeuver>(getManeuvers(), start, end);
        Heuristic<Waypoint> heuristics = new Heuristic<Waypoint>() {
            @Override
            public double getHeuristicEstimate(Waypoint current, Waypoint goal) {
                return current.distance(goal);
            }
        };
        AStarShortestPath<Waypoint, SpatialManeuver> astar = new AStarShortestPath<Waypoint, SpatialManeuver>(getManeuvers(), start, end, heuristics);

        List<Agent> agents = new LinkedList<Agent>();

        agents.add(new ShortestTrajectoryAgent("A1", getManeuvers(), 0.0, new OrientedPoint(0,0,0,0,1,0), new OrientedPoint(5,5,0,0,1,0), getParams().MAX_T, getParams().APPROXIMATION_SAMPLING_INTERVAL));
        FixedTrajectoryAgent astarAgent = new FixedTrajectoryAgent("A2", new OrientedPoint(0,0,0,0,1,0), 0.0, new OrientedPoint(5,5,0,0,1,0), new PiecewiseLinearTrajectory(0.0, astar.getPath(), getParams().APPROXIMATION_SAMPLING_INTERVAL));
        agents.add(astarAgent);

        LOGGER.debug(astarAgent.getName() + " generated the shortest trajectory " + astarAgent.getCurrentTrajectory() + " which is " + astarAgent.getCurrentTrajectory().getDuration() + " meters long");

        return agents;
    }

    @Override
    protected Parameters configureParameters(Parameters params) {
        Logger.getRootLogger().setLevel(Level.TRACE);
        params.GRID_X = 4;
        params.GRID_Y = 4;
        params.SEPARATION = 0.5;
        params.MAX_X = 5;
        params.MAX_Y = 5;
        params.MAX_T = 15;
        params.APPROXIMATION_SAMPLING_INTERVAL = 0.1;
        params.VISUALIZE_NEW_ASSIGNMENT = false;
        params.VISUALISE_SEARCH = false;
        params.CANDIDATES = 30;
        params.SPEED = 1.0;
        return params;
    }

    protected void setupAgents() {
        // Create

        List<Agent> agents = createAgents();
        Collections.sort(agents);
        getProblem().setAgents(agents);
    }

    @Override
    protected Graph<Waypoint, SpatialManeuver> generateGraph<Waypoint, SpatialManeuver>() {
        return new RandomWaypointGraph(5, 5, 9, 4, 41183);
    }




}

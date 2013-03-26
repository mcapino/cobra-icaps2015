package cz.agents.deconfliction.agent;

import org.apache.log4j.Logger;
import org.jgrapht.Graph;

import cz.agents.alite.trajectorytools.graph.spatial.SpatialGraphs;
import cz.agents.alite.trajectorytools.graph.spatial.maneuvers.SpatialManeuver;
import cz.agents.alite.trajectorytools.trajectory.EvaluatedTrajectory;
import cz.agents.alite.trajectorytools.util.OrientedPoint;
import cz.agents.alite.trajectorytools.util.Waypoint;
import cz.agents.deconfliction.util.FastestManeuverTrajectory;


public class FastestTrajectoryAgent extends Agent {

    Logger LOGGER = Logger.getLogger(FastestTrajectoryAgent.class);

    EvaluatedTrajectory trajectory;

    public FastestTrajectoryAgent(String name, Graph<Waypoint, SpatialManeuver> maneuvers, double startTime,  OrientedPoint start, OrientedPoint destination, double maxTime, double approxSamplingInterval) {
        super(name, start, startTime, destination);

        Waypoint startWaypoint = SpatialGraphs.getNearestVertex(maneuvers,start);
        Waypoint destWaypoint = SpatialGraphs.getNearestVertex(maneuvers,destination);


        trajectory = FastestManeuverTrajectory.plan(maneuvers, startWaypoint, startTime, destWaypoint, approxSamplingInterval, maxTime);
        LOGGER.debug(getName() + " generated the fastest trajectory " + getCurrentTrajectory() + ".");

    }

    @Override
    public EvaluatedTrajectory getCurrentTrajectory() {
        return trajectory;
    }

    @Override
    public void start() {}

}

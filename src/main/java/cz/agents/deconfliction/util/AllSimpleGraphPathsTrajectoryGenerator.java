package cz.agents.deconfliction.util;

import org.jgrapht.GraphPath;
import org.jgrapht.graph.DefaultWeightedEdge;

import cz.agents.deconfliction.maneuvergraph.Maneuver;
import org.jgrapht.Graph;
import cz.agents.deconfliction.trajectory.PiecewiseLinearTrajectory;
import cz.agents.alite.trajectorytools.trajectory.Trajectory;
import cz.agents.deconfliction.trajectory.TrajectoryGenerator;
import cz.agents.alite.trajectorytools.util.Waypoint;
import cz.agents.deconfliction.waypointgraph.DefaultWaypointGraph;

public class AllSimpleGraphPathsTrajectoryGenerator extends AllSimpleGraphPathsGenerator
        implements TrajectoryGenerator {


    private double approximationMaxTime = 0;
    private double approximationTimeStep;
    private double startTime;

    public AllSimpleGraphPathsTrajectoryGenerator(Graph<Waypoint, SpatialManeuver> graph,
            Waypoint start, Waypoint end, double startTime, double approximationMaxTime, double approximationTimeStep) {
        super(graph, start, end);
        this.approximationTimeStep = approximationTimeStep;
        this.approximationMaxTime = approximationMaxTime;
        this.startTime = startTime;
    }

    @Override
    public Trajectory nextTrajectory() {
        GraphPath<Waypoint, SpatialManeuver> path = nextGraphPath();
        if (path != null) {
            return new PiecewiseLinearTrajectory(startTime, path, approximationTimeStep);
        }
        else
            return null;
    }


}

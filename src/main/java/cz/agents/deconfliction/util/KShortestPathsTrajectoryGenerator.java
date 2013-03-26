package cz.agents.deconfliction.util;

import org.jgrapht.GraphPath;
import org.jgrapht.alg.KShortestPaths;
import org.jgrapht.graph.DefaultWeightedEdge;

import cz.agents.deconfliction.maneuvergraph.Maneuver;
import org.jgrapht.Graph;
import cz.agents.deconfliction.trajectory.PiecewiseLinearTrajectory;
import cz.agents.alite.trajectorytools.trajectory.Trajectory;
import cz.agents.deconfliction.trajectory.TrajectoryGenerator;
import cz.agents.alite.trajectorytools.util.Waypoint;
import cz.agents.deconfliction.waypointgraph.DefaultWaypointGraph;

public class KShortestPathsTrajectoryGenerator extends KShortestPathsGenerator
        implements TrajectoryGenerator {


    private double approximationTimeStep;
    private double startTime;

    public KShortestPathsTrajectoryGenerator(Graph<Waypoint, SpatialManeuver> graph,
            Waypoint start, Waypoint end, double startTime,
            double approximationTimeStep, int k) {
        super(graph, start, end, k);
        this.approximationTimeStep = approximationTimeStep;
        this.startTime = startTime;
    }

    public KShortestPathsTrajectoryGenerator(KShortestPathsGenerator generator,
            double startTime,  double approximationTimeStep) {
        super(generator);

        this.approximationTimeStep = approximationTimeStep;
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

    public int size() {
        return paths.size();
    }
}

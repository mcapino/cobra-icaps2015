package cz.agents.deconfliction.util;

import org.jgrapht.Graph;

import cz.agents.alite.trajectorytools.graph.spatial.FreeWhenOnTargetGraph;
import cz.agents.alite.trajectorytools.graph.spatial.maneuvers.SpatialManeuver;
import cz.agents.alite.trajectorytools.planner.AStarPlanner;
import cz.agents.alite.trajectorytools.planner.HeuristicFunction;
import cz.agents.alite.trajectorytools.planner.NullGoalPenaltyFunction;
import cz.agents.alite.trajectorytools.planner.PlannedPath;
import cz.agents.alite.trajectorytools.trajectory.EvaluatedSampledTrajectory;
import cz.agents.alite.trajectorytools.trajectory.EvaluatedTrajectory;
import cz.agents.alite.trajectorytools.trajectory.EvaluatedTrajectoryWrapper;
import cz.agents.alite.trajectorytools.trajectory.SpatialManeuverTrajectory;
import cz.agents.alite.trajectorytools.util.SpatialPoint;

public class FastestManeuverTrajectory {
    public static <V extends SpatialPoint, E extends SpatialManeuver>
    EvaluatedTrajectory plan(Graph<V,E> graph, V start, double startTime, V end, double samplingInterval, double maxTime) {
        AStarPlanner<V, E> astar = new AStarPlanner<V, E>();

        PlannedPath<V,E> path = astar.planPath(new FreeWhenOnTargetGraph<V, E>(graph), start, end, new NullGoalPenaltyFunction<V>(), new HeuristicFunction<V>() {
            @Override
            public double getHeuristicEstimate(V current, V goal) {
                return current.distance(goal);
            }
        });

        EvaluatedTrajectory trajectory = new EvaluatedSampledTrajectory(
                new EvaluatedTrajectoryWrapper(
                        new SpatialManeuverTrajectory<V, E>(startTime, path, maxTime),
                        path.getWeight()),
                samplingInterval);

        return trajectory;

    }
}

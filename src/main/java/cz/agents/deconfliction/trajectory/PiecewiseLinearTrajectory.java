package cz.agents.deconfliction.trajectory;

import java.text.DecimalFormat;
import java.util.List;

import org.jgrapht.GraphPath;
import org.jgrapht.alg.DijkstraShortestPath;
import org.jgrapht.graph.DefaultWeightedEdge;

import cz.agents.deconfliction.maneuvergraph.Maneuver;
import org.jgrapht.Graph;
import cz.agents.alite.trajectorytools.util.OrientedPoint;
import cz.agents.deconfliction.util.Point;
import cz.agents.deconfliction.util.Vector;
import cz.agents.alite.trajectorytools.util.Waypoint;
import cz.agents.deconfliction.waypointgraph.DefaultWaypointGraph;
/**
 * Makes a trajectory from a path of maneuvers found in a maneuver graph.
 *
 * Start time can be given. Thus, the trajectory parameters should be interpreted as follows:
 *
 *                      duration
 *       | -------------------------------> |--- stays at goal position until maxTime ----> |
 *  start time                                                                             maxTime
 *
 */

public class PiecewiseLinearTrajectory implements Trajectory {

    private List<SpatialManeuver> pathEdges = null;

    private TrajectoryApproximation approximation;
    private Waypoint startWaypoint;
    private Waypoint endWaypoint;

    private double startTime;
    private double duration = Double.POSITIVE_INFINITY;
    private double maxTime;

    public PiecewiseLinearTrajectory(double startTime, GraphPath<Waypoint, SpatialManeuver> graphPath, double approximationTimeStep) {
        this(startTime, graphPath, startTime+graphPath.getWeight(), approximationTimeStep);
    }

    public PiecewiseLinearTrajectory(double startTime, GraphPath<Waypoint, SpatialManeuver> graphPath, double maxTime, double approximationTimeStep) {

        this.startWaypoint = graphPath.getStartVertex();
        this.endWaypoint = graphPath.getEndVertex();
        this.pathEdges = graphPath.getEdgeList();

        this.startTime = startTime;
        this.duration = graphPath.getWeight();
        this.maxTime = maxTime;
        approximation = new TrajectoryApproximation(this, maxTime, approximationTimeStep);
    }

    @Override
    public OrientedPoint getPosition(double t) {
        Waypoint currentWaypoint = startWaypoint;
        double currentWaypointTime = startTime;
        Vector currentDirection = new Vector(1,0,0);

        if (t < startTime) {
            return null;
        }


        if (pathEdges != null)  {
            for (SpatialManeuver maneuver: pathEdges) {
                Waypoint nextWaypoint = maneuver.getOtherWaypoint(currentWaypoint);
                double duration  = maneuver.getDuration();
                double nextWaypointTime = currentWaypointTime + duration;


                if ( currentWaypointTime <= t && t <= nextWaypointTime) {
                    // linear approximation
                    Point pos = Point.interpolate(currentWaypoint, nextWaypoint, (t-currentWaypointTime) / duration);

                    if (!currentWaypoint.equals(nextWaypoint)) {
                        currentDirection = Vector.subtract(nextWaypoint, currentWaypoint);
                        currentDirection.normalize();
                    } else {
                        currentDirection = new Vector(1,0,0); // wait move
                    }

                    return new OrientedPoint(pos, currentDirection);
                }
                currentWaypoint = nextWaypoint;
                currentWaypointTime = nextWaypointTime;
            }
        }
        if (t >= currentWaypointTime) {
            return new OrientedPoint(currentWaypoint, currentDirection);
        }

        return null;
    }

    @Override
    public double getStartTime() {
        return startTime;
    }

    @Override
    public double getDuration() {
        return duration;
    }

    @Override
    public TrajectoryApproximation getApproximation() {
        return approximation;
    }

    @Override
    public boolean equals(Object obj) {
        if (obj instanceof PiecewiseLinearTrajectory){
            PiecewiseLinearTrajectory other = (PiecewiseLinearTrajectory) obj;
            if (startWaypoint.equals(other.startWaypoint) &&
                    endWaypoint.equals(other.endWaypoint) &&
                    pathEdges.equals(other.pathEdges) &&
                    Math.abs(startTime - other.startTime) < 0.01 ) {
                return true;
            }
        }

        if (obj instanceof Trajectory){
            Trajectory other = (Trajectory) obj;
            if (approximation.equals(other)) {
                return true;
            }
        }

        return false;
    }

    @Override
    public int hashCode() {
        int hashCode = startWaypoint.hashCode();

        for (DefaultWeightedEdge edge: pathEdges) {
            hashCode = hashCode ^ edge.hashCode();
        }

        return hashCode;
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        Waypoint currentWaypoint = startWaypoint;
        sb.append("PLT(@");


        DecimalFormat f = new DecimalFormat("#0.00");
        sb.append(f.format(startTime));
        for (SpatialManeuver maneuver: pathEdges) {
            sb.append(" " + currentWaypoint + " ");
            Waypoint nextWaypoint = maneuver.getOtherWaypoint(currentWaypoint);
            currentWaypoint = nextWaypoint;
        }
        sb.append(currentWaypoint);
        sb.append(" )");
        return sb.toString();
    }
}

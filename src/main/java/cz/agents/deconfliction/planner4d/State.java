package cz.agents.deconfliction.planner4d;

import java.util.Collection;
import java.util.LinkedList;
import java.util.List;

import org.jgrapht.DirectedGraph;
import org.jgrapht.Graph;
import org.jgrapht.SingleVertexGraphPath;

import cz.agents.alite.trajectorytools.graph.spatial.maneuvers.SpatialManeuver;
import cz.agents.alite.trajectorytools.trajectory.SpatialManeuverTrajectory;
import cz.agents.alite.trajectorytools.trajectory.Trajectory;
import cz.agents.alite.trajectorytools.util.SeparationDetector;
import cz.agents.alite.trajectorytools.util.Waypoint;

public class State implements Comparable<State> {

    final double TIME_EPSILON = 0.00001;

    Waypoint waypoint;
    double time;

    State parent;
    SpatialManeuver maneuver;

    double g;
    double h;
    double f;
    int v = 0;

    public State(Waypoint waypoint, double time, State parent, SpatialManeuver maneuver, double g,
            double h, int v) {
        this.waypoint = waypoint;
        this.time = time;
        this.parent = parent;
        this.maneuver = maneuver;
        this.g = g;
        this.h = h;
        this.f = g + h;
        this.v = v;
    }

    public State(Waypoint waypoint, double time, State parent, SpatialManeuver maneuver, GoalState goal,
            double vmax, int v) {
        this(waypoint, time, parent, maneuver, 0.0, 0.0, 0);
        this.g = getCost();
        this.h = getHeuristicEstimateTo(goal, vmax);
        this.f = this.g + this.h;
    }

    public SpatialManeuver getManeuver() {
        return maneuver;
    }

    @Override
    public int compareTo(State other) {

        // if (this.equals(other)) return 0;

        int fDiff = (int) Math.signum(this.f - other.f);
        if (fDiff == 0) {
            // the two states have the same f value
            int vDiff = this.v - other.v;
            if (vDiff == 0) {
                int hDiff = (int) Math.signum(this.h - other.h);
                return hDiff;
            } else {
                return vDiff;
            }
        } else {
            return fDiff;
        }
    }

    public double getCost() {
        return time;
    }

    public double getHeuristicEstimateTo(GoalState goal, double vmax) {
        return waypoint.distance(goal.waypoint) / vmax;

    }

    public List<State> getChildren(Graph<Waypoint, SpatialManeuver> maneuvers, GoalState goal,
            double vmax, double separation, double approxSamplingInterval,
            Collection<Trajectory> hardConstrainingTrajectories,
            Collection<Trajectory> softConstrainingTrajectories) {

        List<State> children = new LinkedList<State>();

        // Hack!!! cast to DirectedGraph
        for (SpatialManeuver nextManeuver : ((DirectedGraph<Waypoint, SpatialManeuver>) maneuvers).outgoingEdgesOf(waypoint)) {
            if (isApplicable(nextManeuver, separation,
                    hardConstrainingTrajectories)) {
                int softViolations = countSoftConstraintViolations(
                        nextManeuver, separation,
                        softConstrainingTrajectories);

                children.add(new State(maneuvers.getEdgeTarget(nextManeuver),
                        this.time + nextManeuver.getDuration(), this, nextManeuver,
                        goal, vmax, this.v	+ softViolations));
            }
        }
        return children;
    }

    @Override
    public boolean equals(Object obj) {
        if (obj instanceof State) {
            State otherState = (State) obj;
            return  this.waypoint.equals(otherState.waypoint) &&
                    Math.abs(this.time - otherState.time) < TIME_EPSILON;
        }
        return false;
    }

    @Override
    public int hashCode() {
        return waypoint.hashCode();
    }

    public boolean equalsIgnoreTime(Object obj) {
        if (obj instanceof State) {
            State otherState = (State) obj;
            return this.waypoint.equals(otherState.waypoint);
        }
        return false;
    }

    public boolean isApplicable(SpatialManeuver candidateManeuver,
            double separation,
            Collection<Trajectory> hardConstrainingTrajectories) {

        List<Trajectory> constrainingTrajectories = new LinkedList<Trajectory>(hardConstrainingTrajectories);

        Trajectory candidateTrajectory = candidateManeuver.getTrajectory(time);

        return !SeparationDetector.hasConflict(candidateTrajectory,
                constrainingTrajectories, separation, separation/4.0);
    }

    public int countSoftConstraintViolations(SpatialManeuver candidateManeuver, double separation,
            Collection<Trajectory> softConstrainingTrajectories) {

        List<Trajectory> constrainingTrajectories = new LinkedList<Trajectory>(softConstrainingTrajectories);

        Trajectory candidateTrajectory = candidateManeuver.getTrajectory(time);

        return SeparationDetector.countConflictingTrajectories(
                candidateTrajectory, constrainingTrajectories, separation, separation/4.0);
    }

    @Override
    public String toString() {
        //return waypoint + " (" + time + ") g:" + g + " h:" + h + " f:" + f + " v: " + v + "parent: " + parent.waypoint + " (" + parent.time + ")";
        //return String.format("%s(%.4f) g:%.4f h:%.4f f:%4f v:%d maneuver:%s parent:%s", waypoint.toString(), time, g, h, f, v, maneuver, parent == null ? "null" : String.format("%s(%.4f)", parent.waypoint, parent.time));
        return String.format("%s(%.2f) f:%.2f", waypoint.toString(), time, f);

    }

    public double getEvaluation() {
        return f;
    }

    public Waypoint getWaypoint() {
        return waypoint;
    }

    public State getParent() {
        return parent;
    }

    public double getTime() {
        return time;
    }

    public boolean isApplicableAsGoal(double separation,
            double maxTime,
            List<Trajectory> hardConstrainingTrajectories) {
        List<Trajectory> constrainingTrajectories = new LinkedList<Trajectory>(hardConstrainingTrajectories);

        Trajectory candidateTrajectory = new SpatialManeuverTrajectory<Waypoint, SpatialManeuver>(
            time,
            new SingleVertexGraphPath<Waypoint, SpatialManeuver>(null,
                    getWaypoint()),
            maxTime);


        return !SeparationDetector.hasConflict(candidateTrajectory,
                constrainingTrajectories, separation, separation/4.0);
    }

}
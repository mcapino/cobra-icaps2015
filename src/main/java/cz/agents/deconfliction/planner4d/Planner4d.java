package cz.agents.deconfliction.planner4d;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Set;

import org.jgrapht.Graph;
import org.jgrapht.GraphPath;
import org.jgrapht.graph.GraphPathImpl;

import cz.agents.alite.trajectorytools.graph.spatial.maneuvers.SpatialManeuver;
import cz.agents.alite.trajectorytools.trajectory.SpatialManeuverTrajectory;
import cz.agents.alite.trajectorytools.trajectory.Trajectory;
import cz.agents.alite.trajectorytools.util.Waypoint;

public class Planner4d {

    final long INF = Long.MAX_VALUE;

    State start;
    GoalState goal;

    protected Graph<Waypoint, SpatialManeuver> maneuvers;
    private double vmax;
    protected double separation;
    double maxTime;
    protected double approxSamplingInterval;


    PriorityQueue<State> open = new PriorityQueue<State>();
    Set<State> closed = new HashSet<State>();

    List<Trajectory> hardConstrainingTrajectories;
    List<Trajectory> softConstrainingTrajectories;

    long expandedStatesCounter = 0;
    long runtimeCounterNanos = 0;


    public Planner4d(Waypoint startWaypoint, double startTime,
            Waypoint endWaypoint, Graph<Waypoint, SpatialManeuver> maneuvers,
            double separation, double maxTime, double approxSamplingInterval, double vmax) {
        this(startWaypoint, startTime, endWaypoint, maneuvers, separation,  maxTime,
                approxSamplingInterval, vmax,
                new ArrayList<Trajectory>(), new ArrayList<Trajectory>());
    }

    public Planner4d(Waypoint startWaypoint, double startTime, Waypoint endWaypoint,
            Graph<Waypoint, SpatialManeuver> maneuvers, double separation, double maxTime,
            double approxSamplingInterval, double vmax,
            List<Trajectory> hardConstrainingTrajectories,
            List<Trajectory> softConstrainingTrajectories) {

        this.maneuvers = maneuvers;
        this.vmax = vmax;

        this.separation = separation;
        this.approxSamplingInterval = approxSamplingInterval;
        this.maxTime = maxTime;

        this.hardConstrainingTrajectories = hardConstrainingTrajectories;
        this.softConstrainingTrajectories = softConstrainingTrajectories;


        this.goal = new GoalState(endWaypoint);
        this.start = new State(startWaypoint, startTime, null, null, goal, vmax, 0);

        open.add(start);
    }

    public GraphPath<Waypoint, SpatialManeuver> solve() {
        return searchStep(INF).getPath();
    }

    public static class SearchStepResult{
        public boolean finished;
        public GraphPath<Waypoint, SpatialManeuver> path;

        public SearchStepResult(boolean finished,
                GraphPath<Waypoint, SpatialManeuver> path) {
            super();
            this.finished = finished;
            this.path = path;
        }

        public GraphPath<Waypoint, SpatialManeuver> getPath() {
            return path;
        }

        public boolean isFinished() {
            return finished;
        }

        public boolean foundSolution() {
            return path != null;
        }

        public Trajectory getAsTrajectory(double startTime, double maxTime, double approxSamplingInterval) {
            return new SpatialManeuverTrajectory<Waypoint, SpatialManeuver>(startTime, path, maxTime);
        }

    }

    public SearchStepResult searchStep(long timeLimitNs) {
        expandedStatesCounter = 0;
        long startedAtNanos = System.nanoTime();
        long interruptAt = startedAtNanos + timeLimitNs;

        while (!open.isEmpty()) {
            State current = open.poll();

            if (current.equalsIgnoreTime(goal) && current.isApplicableAsGoal(separation, maxTime, hardConstrainingTrajectories)) {
                // Found solution - reconstruct path
                runtimeCounterNanos += (System.nanoTime() - startedAtNanos);
                return new SearchStepResult(true, reconstructPath(current));
            }

            closed.add(current);

            List<State> children = current.getChildren(maneuvers, goal, vmax, separation, approxSamplingInterval, hardConstrainingTrajectories, softConstrainingTrajectories);
            expandedStatesCounter++;

            for (State child : children ) {

                if (!closed.contains(child) && child.getEvaluation() <= maxTime) {
                    updateStateInOpen(child);
                }

            }

            if (System.nanoTime() >= interruptAt && timeLimitNs != INF) {
                runtimeCounterNanos += (System.nanoTime() - startedAtNanos);
                return new SearchStepResult(false, null);
            }
         }
         runtimeCounterNanos += (System.nanoTime() - startedAtNanos);
         return new SearchStepResult(true, null);
    }


    public Trajectory solveTrajectory() {
        GraphPath<Waypoint, SpatialManeuver> path = solve();
        if (path != null) {
          return new SpatialManeuverTrajectory<Waypoint, SpatialManeuver>(start.getTime(), path, maxTime);
        } else {
          return null;
        }
    }

    private GraphPath<Waypoint, SpatialManeuver> reconstructPath(State goalstate) {
        LinkedList<SpatialManeuver> path = new LinkedList<SpatialManeuver>();

        State current = goalstate;
        while (current.getManeuver() != null) {
                SpatialManeuver maneuver = current.getManeuver();
                path.addFirst(maneuver);
                current = current.getParent();
        }

        GraphPath<Waypoint, SpatialManeuver> graphPath = null;

        if (path != null) {
            double cost = 0.0;
            for(SpatialManeuver maneuver : path) {
                cost += maneuver.getDuration();
            }
            graphPath = new GraphPathImpl<Waypoint, SpatialManeuver>(maneuvers, start.getWaypoint(), goalstate.getWaypoint(), path, cost);
        }

        return graphPath;
    }

    private void updateStateInOpen(State candidateState) {
        Iterator<State> iterator = open.iterator();
        boolean alreadyContainsBetter = false;

        while (iterator.hasNext()) {
            State state = (State) iterator.next();
            if (state.equals(candidateState)) {
                if (state.compareTo(candidateState) <= 0) {
                    alreadyContainsBetter = true;
                } else {
                    iterator.remove();
                }
            }
        }

        if (!alreadyContainsBetter) {
            open.add(candidateState);
        }
    }

    public long getExpandedStatesCounter() {
        return expandedStatesCounter;
    }

    public long getRuntimeCounterNanos() {
        return runtimeCounterNanos;
    }

}

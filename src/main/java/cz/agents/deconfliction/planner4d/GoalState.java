package cz.agents.deconfliction.planner4d;

import java.util.Arrays;

import cz.agents.alite.trajectorytools.util.Waypoint;

public class GoalState extends State {

    public GoalState(Waypoint waypoint) {
        super(waypoint, Double.NaN, null, null, 0.0, 0.0, 0);
    }
}

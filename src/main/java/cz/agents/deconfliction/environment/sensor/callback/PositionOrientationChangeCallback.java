package cz.agents.deconfliction.environment.sensor.callback;

import cz.agents.alite.trajectorytools.util.OrientedPoint;

public interface PositionOrientationChangeCallback {

    public void sensePositionOrientationChange(OrientedPoint orientedPoint);

}

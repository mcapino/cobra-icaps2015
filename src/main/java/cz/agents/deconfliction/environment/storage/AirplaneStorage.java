package cz.agents.deconfliction.environment.storage;

import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;

import cz.agents.alite.environment.Storage;
import cz.agents.alite.trajectorytools.util.OrientedPoint;
import cz.agents.deconfliction.environment.embodiment.Airplane;
import cz.agents.deconfliction.environment.entity.PositionedEntity;
import cz.agents.deconfliction.environment.sensor.callback.PositionOrientationChangeCallback;

public class AirplaneStorage extends Storage implements PositionedEntityStorage {

    private Map<String, Airplane> airplanes = new LinkedHashMap<String, Airplane>();

    private Map<String, PositionOrientationChangeCallback> positionOrientationChangeCallback = new HashMap<String, PositionOrientationChangeCallback>();

    public Map<String, Airplane> getAirplanes() {
        return Collections.unmodifiableMap(airplanes);
    }

    @Override
    public Map<String, ? extends PositionedEntity> getPositionedEntities() {
        return getAirplanes();
    }

    public void addAirplane(Airplane airplaneInfo) {
        airplanes.put(airplaneInfo.name, airplaneInfo);

        moveAirplane(airplaneInfo.name, airplaneInfo.targetPosition);
    }

    public void moveAirplane(String name, OrientedPoint newPosition) {
        airplanes.put(name, airplanes.get(name).move(newPosition));

        if (positionOrientationChangeCallback.containsKey(name)) {
            positionOrientationChangeCallback.get(name).sensePositionOrientationChange(newPosition);
        }
    }

    public void interpolatedMoveAirplane(String name, OrientedPoint currentPositionOrientation) {
        airplanes.put(name, airplanes.get(name).interpolatedMove(currentPositionOrientation));
    }

    public void registerPositionOrientationChangeCallback(String name, PositionOrientationChangeCallback positionOrientationChangeSensing) {
        positionOrientationChangeCallback.put(name, positionOrientationChangeSensing);
    }

}

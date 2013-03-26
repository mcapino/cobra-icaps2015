package cz.agents.deconfliction.environment.embodiment;

import cz.agents.alite.trajectorytools.util.OrientedPoint;
import cz.agents.alite.trajectorytools.util.SpatialPoint;
import cz.agents.deconfliction.environment.entity.PositionedEntity;

public class Airplane implements PositionedEntity {
    public final String name;
    public final OrientedPoint targetPosition;
    public final OrientedPoint currentPositionDirection;

    public Airplane(String name, OrientedPoint target, OrientedPoint currentPositionDirection) {
        this.name = name;
        this.targetPosition = target;
        this.currentPositionDirection = currentPositionDirection;
    }

    public Airplane move(OrientedPoint newPosition) {
        return new Airplane(name, newPosition, newPosition);
    }

    public Airplane interpolatedMove(OrientedPoint newCurrentPositionDirection) {
        return new Airplane(name, targetPosition, newCurrentPositionDirection);
    }

    @Override
    public SpatialPoint getPosition() {
        return currentPositionDirection;
    }

    @Override
    public String toString() {
        return "Airplane(" + name + ", " + targetPosition + ", " + currentPositionDirection + ")";
    }

}

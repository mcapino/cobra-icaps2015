package cz.agents.deconfliction.environment.actuator;

import cz.agents.alite.common.entity.Entity;
import cz.agents.alite.common.event.DurativeEvent;
import cz.agents.alite.common.event.Event;
import cz.agents.alite.environment.eventbased.EventBasedAction;
import cz.agents.deconfliction.environment.SimpleAirplaneEnvironment;
import cz.agents.deconfliction.environment.storage.AirplaneStorage;
import cz.agents.deconfliction.util.AgentScoutEventType;
import cz.agents.alite.trajectorytools.util.OrientedPoint;

public class AirplaneActuator extends EventBasedAction {

    public final static long MOVE_DURATION = 5000;
    public final static int INTERPOLATED_MOVES = 10;

    private final AirplaneStorage airplaneStorage;

    public AirplaneActuator(SimpleAirplaneEnvironment environment, Entity relatedEntity) {
        super(environment, relatedEntity);
        airplaneStorage = environment.getAirplaneStorage();
    }

    public void actSynchronizedMove(OrientedPoint targetPosition) {
        long duration = MOVE_DURATION;
        getEventProcessor().addEvent(AgentScoutEventType.MOVE, this, null, targetPosition, duration);
    }

    @Override
    public void handleEvent(Event event) {
        if (event.isType(AgentScoutEventType.MOVE)) {
            OrientedPoint p = (OrientedPoint) event.getContent();
            airplaneStorage.moveAirplane(getRelatedEntity().getName(), p);
        } else if (event.isType(AgentScoutEventType.INTERPOLATED_MOVE)) {
            OrientedPoint p = (OrientedPoint) event.getContent();
            airplaneStorage.interpolatedMoveAirplane(getRelatedEntity().getName(), p);
        } else {
            super.handleEvent(event);
        }
    }

}

package cz.agents.deconfliction.environment.storage;

import java.util.Map;

import cz.agents.deconfliction.environment.entity.PositionedEntity;


public interface PositionedEntityStorage {

    Map<String, ? extends PositionedEntity> getPositionedEntities();

}

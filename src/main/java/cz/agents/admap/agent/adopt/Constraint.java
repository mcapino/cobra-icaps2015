package cz.agents.admap.agent.adopt;


public interface Constraint {
    double getCost(tt.euclidtime3i.Region t1, tt.euclidtime3i.Region t2);
}

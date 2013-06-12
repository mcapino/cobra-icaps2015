package cz.agents.admap.agent.adopt;

import tt.euclid2i.EvaluatedTrajectory;

public interface LocalCost {
    double getCost(EvaluatedTrajectory traj);
}

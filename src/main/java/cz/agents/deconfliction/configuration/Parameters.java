package cz.agents.deconfliction.configuration;

import cz.agents.deconfliction.agent.DecentralizedPrioritizedPlanningAgent.ReplanningStrategy;

public class Parameters {
     public int MAX_X = 10; // m
     public int MAX_Y = 10; // m
     public int MAX_T = 50;  // s

     public int GRID_X = 10;   // number of columns
     public int GRID_Y = 10;   // number of rows

     public double SPEED = 1.0;       // the speed of moving vehicles in m/s
     public double SEPARATION = 0.5; // the minimal separation distance
     public double APPROXIMATION_SAMPLING_INTERVAL = (SEPARATION/4)/SPEED;

     public double SIMULATION_SPEED_RATIO = 1.0;

     public int  CANDIDATES = 20;

     public boolean ENABLE_VISUALISATION = true;
     public boolean VISUALISE_SEARCH = true;
     public boolean VISUALIZE_NEW_ASSIGNMENT = true;
     public boolean RESOLVE_CONFLICTS = true;

     public ReplanningStrategy REPLANNING_STRATEGY = ReplanningStrategy.IF_INCONSISTENT;
}

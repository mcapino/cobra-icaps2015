package cz.agents.deconfliction.creator;

import org.apache.log4j.Level;
import org.apache.log4j.Logger;
import org.jgrapht.Graph;

import cz.agents.alite.creator.Creator;
import cz.agents.alite.trajectorytools.graph.spatial.SpatialGridFactory;
import cz.agents.alite.trajectorytools.graph.spatial.maneuvers.SpatialManeuver;
import cz.agents.alite.trajectorytools.util.Point;
import cz.agents.alite.trajectorytools.util.Waypoint;
import cz.agents.deconfliction.agent.DecentralizedPrioritizedPlanningAgent.ReplanningStrategy;
import cz.agents.deconfliction.agent.InterruptibleAsynchronousPrioritizedPlanningAgent.InterruptingStrategy;
import cz.agents.deconfliction.configuration.Parameters;
import cz.agents.deconfliction.creator.iapp.IAPPCustomCreator;
import cz.agents.deconfliction.solver.NoSolutionFoundException;
import cz.agents.deconfliction.util.GraphAgentMissionGenerator;
import cz.agents.deconfliction.util.GraphAgentMissionGenerator.Mission;

public class TestingCreator implements Creator {

    private Parameters params = new Parameters();

    @Override
    public void init(String[] args) {
        Logger.getRootLogger().setLevel(Level.DEBUG);
        params.MAX_X = 20;
        params.MAX_Y = 20;
        params.SPEED = 1.0;
        params.MAX_T = (int) Math.ceil((2 * params.MAX_X) / params.SPEED);
        params.GRID_X = 60;
        params.GRID_Y = 60;
        params.SEPARATION = 0.8;
        params.APPROXIMATION_SAMPLING_INTERVAL = (params.SEPARATION / 10) / params.SPEED;
        params.VISUALISE_SEARCH = false;
        params.VISUALIZE_NEW_ASSIGNMENT = false;
        params.ENABLE_VISUALISATION = true;
        params.SIMULATION_SPEED_RATIO = 0.6;
        params.REPLANNING_STRATEGY = ReplanningStrategy.IF_INCONSISTENT;
    }

    @Override
    public void create() {
        int AGENTS = 90;
        int SEED = 27;

        Graph<Waypoint, SpatialManeuver> maneuvers = SpatialGridFactory.create8WayGrid(
                params.MAX_X, params.MAX_Y, params.GRID_X, params.GRID_Y,
                params.SPEED);



        try {


            /*
            Mission[] agentMissions = AgentMissionGenerator
                    .generateRandomSimilarLength(new Random(SEED), maneuvers,
                            AGENTS, params.MAX_X, params.MAX_Y,
                            params.MAX_X / 2, params.SEPARATION);*/
            /*
            Mission[] agentMissions = AgentMissionGenerator
                    .generateRandom(new Random(SEED),
                            maneuvers, AGENTS, params.MAX_X,
                            params.MAX_Y, params.SEPARATION);*/

            /*
            Mission[] agentMissions = AgentMissionGenerator.generateSuperconflicts(
                    maneuvers,
                    new int[] {4,4,8,8},
                    new Point[] {new Point(5,5,0), new Point(5,15,0), new Point(15,5,0),  new Point(15,15,0)},
                    new int[] {4,4,2,2});*/


            Mission[] agentMissions =
            GraphAgentMissionGenerator.generateSpiralSuperconflict(maneuvers, 8, new Point(10,10,0),
            2.0, 6.0, 0);

           // Mission[] agentMissions = AgentMissionGenerator.generateSuperconflict(maneuvers, AGENTS, new Point(params.MAX_X/2,params.MAX_Y/2,0), params.MAX_X/2);

            // Mission[] agentMissions = AgentMissionGenerator.generateTest(maneuvers);
            // Mission[] agentMissions = AgentMissionGenerator.generateTwoDifferentSizeSuperconflictScenario(maneuvers);


            //ODIDCustomCreator odid = new ODIDCustomCreator();
             //odid.create(maneuvers, agentMissions, params);

            //CACustomCreator ca = new CACustomCreator();
            //ca.create(maneuvers, agentMissions, params);

            //SPPCustomCreator spp = new SPPCustomCreator();
            //spp.create(maneuvers, agentMissions, params);

            //APPCustomCreator app = new APPCustomCreator();
            //app.create(maneuvers, agentMissions, params);

            IAPPCustomCreator iapp = new IAPPCustomCreator(InterruptingStrategy.ALWAYS);
            iapp.create(maneuvers, agentMissions, params);

            //System.out.println("conflict checks:" + app.getConflictCheckCumulativeCounter());

        } catch (NoSolutionFoundException e) {
            System.out.println("****** NO SOLUTION FOUND *****");
        }
    }

}

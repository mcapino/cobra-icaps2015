package cz.agents.deconfliction.centralsolver;

import static org.junit.Assert.*;

import java.io.File;
import java.io.FileInputStream;
import java.util.Arrays;
import java.util.Properties;
import java.util.Random;

import org.apache.log4j.PropertyConfigurator;
import org.junit.BeforeClass;
import org.junit.Test;

import cz.agents.deconfliction.configuration.Parameters;
import cz.agents.deconfliction.creator.brabt.BRABTCustomCreator;
import cz.agents.deconfliction.creator.centralsolver.CACustomCreator;
import cz.agents.deconfliction.maneuvergraph.EightWayConstantSpeedGridGraph;
import org.jgrapht.Graph;
import cz.agents.deconfliction.maneuvergraph.RandomWaypointGraph;
import cz.agents.alite.trajectorytools.trajectory.Trajectory;
import cz.agents.deconfliction.util.GraphAgentMissionGenerator;
import cz.agents.deconfliction.util.GraphAgentMissionGenerator.Mission;
import cz.agents.deconfliction.util.Point;

public class CAAgainstBRBATTest {
    @BeforeClass
    public static void beforeClass() {
        Properties prop = new Properties();
        try {
            prop.load(new FileInputStream("resources" + File.separator + "log4j" + File.separator + "log4j.properties"));
        } catch (Exception ex){
            ex.printStackTrace();
        }

        PropertyConfigurator.configure(prop);
    }

    @Test
    public void test() throws InterruptedException {

        Parameters params = new Parameters();

        params.MAX_X = 10;
        params.MAX_Y = 10;
        params.GRID_X = 4;
        params.GRID_Y = 4;
        params.SPEED = 1.0;
        params.SEPARATION = 0.5;
        params.APPROXIMATION_SAMPLING_INTERVAL = (params.SEPARATION/4)/params.SPEED;
        params.VISUALISE_SEARCH = false;
        params.VISUALIZE_NEW_ASSIGNMENT = false;
        params.ENABLE_VISUALISATION = false;


        int MAX_AGENTS = 12;
        int SEED = 1;
        Graph<Waypoint, SpatialManeuver> maneuvers = new EightWayConstantSpeedGridGraph(params.MAX_X, params.MAX_Y, params.GRID_X, params.GRID_Y, params.SPEED);
        //Graph<Waypoint, SpatialManeuver> maneuvers = new RandomWaypointGraph(params.MAX_X, params.MAX_Y, 30, 5, SEED);

        for (int nAgents = 1; nAgents <= MAX_AGENTS; nAgents++) {
            //Mission[] agentMissions = AgentMissionGenerator.generateRandom(new Random(SEED), maneuvers, nAgents);
            Mission[] agentMissions = GraphAgentMissionGenerator.generateSuperconflict(maneuvers, nAgents, new Point(5,5,0), 5.0);

            // Central solver

            System.out.println("----- Agents: " + nAgents);

            CACustomCreator ca = new CACustomCreator();

            ca.create(maneuvers, agentMissions, params);

            System.out.println("CA:");
            System.out.println("Runtime: " + ca.getRuntime());
            System.out.println("Nominal: " + ca.getNominalCost());
            System.out.println("Final: " + ca.getFinalCost());

            //Thread.sleep(100);

            BRABTCustomCreator brabt = new BRABTCustomCreator();
            brabt.create(maneuvers, agentMissions, params);

            System.out.println("BRABT:");
            System.out.println("Runtime: " + brabt.getWallclockRuntime());
            System.out.println("Nominal: " + brabt.getNominalCost());
            System.out.println("Final: " + brabt.getSolutionCost());

            Trajectory[] caSolution = ca.getSolutionTrajectories();
            Trajectory[] brabtSolution = brabt.getSolutionTrajectories();

            System.out.println();

            for (int i = 0; i < caSolution.length; i++){
                System.out.println("ca:    (" + i + ") " + caSolution[i]);
                System.out.println("brabt: (" + i + ") " + brabtSolution[i]);
                System.out.println();
            }

            //assertTrue(Arrays.equals(caSolution, brabtSolution));

            //Thread.sleep(100);

            assertTrue(Math.abs(ca.getFinalCost() - brabt.getSolutionCost()) < 0.001);
        }

    }

}

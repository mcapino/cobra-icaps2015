package cz.agents.deconfliction.creator;

import java.awt.image.SampleModel;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Random;

import org.apache.log4j.Level;
import org.apache.log4j.Logger;

import cz.agents.alite.creator.Creator;
import cz.agents.deconfliction.agent.DecentralizedPrioritizedPlanningAgent.ReplanningStrategy;
import cz.agents.deconfliction.agent.InterruptibleAsynchronousPrioritizedPlanningAgent.InterruptingStrategy;
import cz.agents.deconfliction.configuration.Parameters;
import cz.agents.deconfliction.creator.app.APPCustomCreator;
import cz.agents.deconfliction.creator.brabt.BRABTCustomCreator;
import cz.agents.deconfliction.creator.centralsolver.CACustomCreator;
import cz.agents.deconfliction.creator.centralsolver.ODIDCustomCreator;
import cz.agents.deconfliction.creator.iapp.IAPPCustomCreator;
import cz.agents.deconfliction.creator.spp.SPPCustomCreator;
import cz.agents.deconfliction.maneuvergraph.EightWayConstantSpeedGridGraph;
import cz.agents.deconfliction.maneuvergraph.FourWayConstantSpeedGridGraph;
import org.jgrapht.Graph;
import cz.agents.deconfliction.solver.NoSolutionFoundException;
import cz.agents.deconfliction.util.GraphAgentMissionGenerator;
import cz.agents.deconfliction.util.GraphAgentMissionGenerator.Mission;
import cz.agents.deconfliction.util.Point;

public class ScenarioCreator implements Creator {

    private Parameters params = new Parameters();
    Logger LOGGER = Logger.getLogger(ScenarioCreator.class);

    @Override
    public void init(String[] args) {
        Logger.getRootLogger().setLevel(Level.INFO);
        params.MAX_X = 20;
        params.MAX_Y = 20;
        params.SPEED = 1.0;
        params.MAX_T = (int) ((4 * params.MAX_X) / (params.SPEED));

        params.GRID_X = 60;
        params.GRID_Y = 60;

        params.SEPARATION = 0.8;
        params.APPROXIMATION_SAMPLING_INTERVAL = (params.SEPARATION / 4)
                / params.SPEED;
        params.VISUALISE_SEARCH = false;
        params.VISUALIZE_NEW_ASSIGNMENT = false;
        params.ENABLE_VISUALISATION = false;
        params.REPLANNING_STRATEGY = ReplanningStrategy.IF_INCONSISTENT;
    }

    @Override
    public void create() {
        /*
        try {
            System.in.read();
        } catch (IOException e1) {
            e1.printStackTrace();
        } */

        int MIN_AGENTS = 90;
        int MAX_AGENTS = 90;
        int STEP = 5;
        int FIRST_SEED = 19;
        int NSEEDS = 10;
        Graph<Waypoint, SpatialManeuver> maneuvers = new EightWayConstantSpeedGridGraph(
                params.MAX_X, params.MAX_Y, params.GRID_X, params.GRID_Y,
                params.SPEED);

        try {
            // detailed results
            BufferedWriter out = new BufferedWriter(new FileWriter(
                    "results_scenario.csv"));

            // aggregated results
            BufferedWriter aggrOut = new BufferedWriter(new FileWriter(
                    "results_aggr_scenario.csv"));

            out.write("maxx=" + params.MAX_X + " maxy=" + params.MAX_Y
                    + " gridx=" + params.GRID_X + " gridy=" + params.GRID_Y
                    + " sep=" + params.SEPARATION);
            out.newLine();
            out.write("nagents;seed;conflicts;nominal;odid-runtime;odid-final;" +
                    "ca-runtime;ca-final;" +
                    "spp-wallclock;spp-active;spp-idle;spp-msgs;spp-rounds;spp-invocation;spp-states;spp-planner-runtime;spp-comm;spp-conf-checking;spp-avg-constraints;spp-final;" +
                    "app-wallclock;app-active;app-idle;app-msgs;app-invocations;app-states;app-planner-runtime;app-comm;app-conf-checking;app-avg-constraints;app-final;" +
                    "iapp-wallclock;iapp-active;iapp-idle;iapp-msgs;iapp-invocations;iapp-states;iapp-planner-runtime;iapp-comm;iapp-conf-checking;iapp-final;"  +
                    "iapp-ic-wallclock;iapp-ic-active;iapp-ic-idle;iapp-ic-msgs;iapp-ic-invocations;iapp-ic-states;iapp-ic-planner-runtime;iapp-ic-comm;iapp-ic-conf-checking;iapp-ic-final;"
                    );

            out.newLine();

            aggrOut.write("agents;suceeded;failed;avg conflicts;" +
                    "ca;spp;app;iapp;iapp2;" +
                    "spp-active;app-active;iapp-active;iapp-ic-active;" +
                    "spp-msgs;app-msgs;iapp-msgs;iapp-ic-msgs;" +
                    "ca-cost;spp-cost;app-cost;iapp-cost;iapp-ic-cost;" +
                    "ca-failrate;spp-failrate;app-failrate;iapp-failrate;iapp-ic-failrate;");
            aggrOut.newLine();

            for (int nAgents = MIN_AGENTS; nAgents <= MAX_AGENTS; nAgents += STEP) {

                double nominal;

                double caCumRuntime = 0;
                double caCost = 0;
                int caFailures = 0;

                double sppCumRuntime = 0;
                double sppCumActive = 0;
                int    sppCumMessages = 0;
                double sppCost = 0;
                int    sppFailures = 0;


                double appCumRuntime = 0;
                double appCumActive = 0;
                int    appCumMessages = 0;
                double appCost = 0;
                int    appFailures = 0;


                double iappCumRuntime = 0;
                double iappCumActive = 0;
                int    iappCumMessages = 0;
                double iappCost = 0;
                int    iappFailures = 0;

                double iapp2CumRuntime = 0;
                double iapp2CumActive = 0;
                int    iapp2CumMessages = 0;
                double iapp2Cost = 0;
                int    iapp2Failures = 0;

                int failedRuns = 0;
                int completedRuns = 0;
                int cumConflicts = 0;

                for (int seed = FIRST_SEED; seed < FIRST_SEED+NSEEDS; seed++) {
                    try {
                        boolean failedRun = false;

                        /*
                        Mission[] agentMissions = AgentMissionGenerator

                                .generateRandomSimilarLength(new Random(seed),
                                        maneuvers, nAgents, params.MAX_X,
                                        params.MAX_Y, params.MAX_X / 2,
                                        params.SEPARATION);*/

                        /*
                         Mission[] agentMissions = AgentMissionGenerator
                                .generateRandom(new Random(seed),
                                        maneuvers, nAgents, params.MAX_X,
                                        params.MAX_Y, params.SEPARATION); */


                         Mission[] agentMissions =
                         GraphAgentMissionGenerator.generateSpiralSuperconflict(maneuvers, 8, new Point(10,10,0),
                         2.0, 6.0, 0);



                        /*
                        Mission[] agentMissions =
                        AgentMissionGenerator.generateSuperconflict(maneuvers, nAgents, new Point(10,10,0), 8.0);
                        */
                        //Mission[] agentMissions = AgentMissionGenerator.generateTest(maneuvers);

                         /*
                        Mission[] agentMissions = AgentMissionGenerator.generateSuperconflicts(
                                maneuvers,
                                new int[] {4,4,8,8},
                                new Point[] {new Point(5,5,0), new Point(5,15,0), new Point(15,5,0),  new Point(15,15,0)},
                                new int[] {4,4,2,2});*/

                        // ***** Central optimal solver ODID ****

                        LOGGER.info("\n\n\n----- Agents: " + nAgents
                                + " (seed:" + seed + ") ------ \n\n\n");

                        LOGGER.info("\n\n>>>>> ODID:");

                        ODIDCustomCreator odid = new ODIDCustomCreator();

                        // odid.create(maneuvers, agentMissions, params);


                        LOGGER.info("ODID:");
                        LOGGER.info("Runtime: " + odid.getRuntimeMs());
                        LOGGER.info("Nominal: " + odid.getNominalCost());
                        LOGGER.info("Final: " + odid.getFinalCost());

                        //////////////////////////////
                        //
                        // ******* CA *******
                        //
                        //////////////////////////////

                        LOGGER.info("\n\n>>>>> Cooperative A*:");

                        CACustomCreator ca = new CACustomCreator();
                        try {
                            ca.create(maneuvers, agentMissions, params);
                        } catch (NoSolutionFoundException e) {
                            caFailures++;
                            LOGGER.info("****** NO SOLUTION FOUND *****");
                            failedRun = true;
                        }

                        LOGGER.info("CA:");
                        LOGGER.info("Runtime: " + ca.getRuntime());
                        LOGGER.info("Nominal: " + ca.getNominalCost());
                        LOGGER.info("Conflicts: " + ca.getConflictCount());
                        LOGGER.info("Final: " + ca.getFinalCost());

                        System.gc();

                        if (params.ENABLE_VISUALISATION) {
                            Thread.sleep(2000);
                        }


                        //////////////////////////////
                        //
                        // ******* SDPP *******
                        //
                        //////////////////////////////

                        LOGGER.info("\n\n>>>>> SPP:");

                        SPPCustomCreator spp = new SPPCustomCreator();

                        try {
                            spp.create(maneuvers, agentMissions, params);
                        } catch (NoSolutionFoundException e) {
                            sppFailures++;
                            LOGGER.info("****** NO SOLUTION FOUND *****");
                            failedRun = true;
                        }

                        LOGGER.info("SPP:");
                        LOGGER.info("Wallclock Runtime: "
                                + spp.getWallclockRuntime());
                        LOGGER.info("Aggregate Active Runtime: "
                                + spp.getAggregateActiveRuntime());
                        LOGGER.info("Aggregate Idle Runtime: "
                                + spp.getAggregateIdleRuntime());
                        LOGGER.info("Broadcasted messages: "
                                + spp.getBroadcastMessageCounter());
                        LOGGER.info("Expanded states: "
                                + spp.getPlannerCumulativeExpandedStates());
                        LOGGER.info("Conflict checks: "
                                + spp.getConflictCheckCumulativeCounter());
                        LOGGER.info("Rounds: " + spp.getRoundCounter());
                        LOGGER.info("Final: " + spp.getSolutionCost());

                        System.gc();

                        if (params.ENABLE_VISUALISATION) {
                            Thread.sleep(2000);
                        }

                        //////////////////////////////
                        //
                        // ******* ADPP *******
                        //
                        //////////////////////////////

                        LOGGER.info("\n\n>>>>> APP:\n");

                        APPCustomCreator app = new APPCustomCreator();

                        try {
                            app.create(maneuvers, agentMissions, params);
                        } catch (NoSolutionFoundException e) {
                            appFailures++;
                            LOGGER.info("****** NO SOLUTION FOUND *****");
                            failedRun = true;
                        }


                        LOGGER.info("APP:");
                        LOGGER.info("Wallclock Runtime: "
                                + app.getWallclockRuntime());
                        LOGGER.info("Aggregate Active Runtime: "
                                + app.getAggregateActiveRuntime());
                        LOGGER.info("Aggregate Idle Runtime: "
                                + app.getAggregateIdleRuntime());
                        LOGGER.info("Broadcasted messages: "
                                + app.getBroadcastMessageCounter());
                        LOGGER.info("Expanded states: "
                                + app.getPlannerCumulativeExpandedStates());
                        LOGGER.info("Conflict checks: "
                                + app.getConflictCheckCumulativeCounter());
                        LOGGER.info("Final: " + app.getSolutionCost());

                        if (params.ENABLE_VISUALISATION) {
                            Thread.sleep(2000);
                        }

                        //////////////////////////////
                        //
                        // ******* IADPP Interruption strategy: ALWAYS*******
                        //
                        //////////////////////////////

                        LOGGER.info("\n\n>>>>> IAPP (ALWAYS) :");

                        IAPPCustomCreator iapp = new IAPPCustomCreator(InterruptingStrategy.ALWAYS);

                        try {
                            iapp.create(maneuvers, agentMissions, params);
                        } catch (NoSolutionFoundException e) {
                            iappFailures++;
                            LOGGER.info("****** NO SOLUTION FOUND *****");
                            failedRun = true;
                        }

                        LOGGER.info("IAPP (always):");
                        LOGGER.info("Wallclock Runtime: "
                                + iapp.getWallclockRuntime());
                        LOGGER.info("Aggregate Active Runtime: "
                                + iapp.getAggregateActiveRuntime());
                        LOGGER.info("Aggregate Idle Runtime: "
                                + iapp.getAggregateIdleRuntime());
                        LOGGER.info("Broadcasted messages: "
                                + iapp.getBroadcastMessageCounter());
                        LOGGER.info("Expanded states: "
                                + iapp.getPlannerCumulativeExpandedStates());
                        LOGGER.info("Conflict checks: "
                                + iapp.getConflictCheckCumulativeCounter());
                        LOGGER.info("Final: " + iapp.getSolutionCost());

                        if (params.ENABLE_VISUALISATION) {
                            Thread.sleep(2000);
                        }

                        //////////////////////////////
                        //
                        // ******* IADPP Interruption strategy: ALWAYS *******
                        //
                        //////////////////////////////

                        LOGGER.info("\n\n>>>>> IAPP (IF INCONSISTENT):");

                        IAPPCustomCreator iapp2 = new IAPPCustomCreator(InterruptingStrategy.IF_INCONSISTENT);

                        try {
                            iapp2.create(maneuvers, agentMissions, params);
                        } catch (NoSolutionFoundException e) {
                            iapp2Failures++;
                            LOGGER.info("****** NO SOLUTION FOUND *****");
                            failedRun = true;
                        }

                        LOGGER.info("IAPP (if inconsistent):");
                        LOGGER.info("Wallclock Runtime: "
                                + iapp2.getWallclockRuntime());
                        LOGGER.info("Aggregate Active Runtime: "
                                + iapp2.getAggregateActiveRuntime());
                        LOGGER.info("Aggregate Idle Runtime: "
                                + iapp2.getAggregateIdleRuntime());
                        LOGGER.info("Broadcasted messages: "
                                + iapp2.getBroadcastMessageCounter());
                        LOGGER.info("Expanded states: "
                                + iapp2.getPlannerCumulativeExpandedStates());
                        LOGGER.info("Conflict checks: "
                                + iapp2.getConflictCheckCumulativeCounter());
                        LOGGER.info("Final: " + iapp.getSolutionCost());




                        if (!failedRun) {

                        // Gather cumulative data
                        nominal = ca.getNominalCost();

                        caCumRuntime += ca.getRuntime();
                        caCost += (ca.getFinalCost() - nominal) / nominal;

                        cumConflicts += ca.getConflictCount();

                        sppCumRuntime += spp.getWallclockRuntime();
                        sppCumActive += spp.getAggregateActiveRuntime();
                        sppCumMessages += spp.getBroadcastMessageCounter();
                        sppCost += (spp.getSolutionCost() - nominal) / nominal;

                        appCumRuntime += app.getWallclockRuntime();
                        appCumActive += app.getAggregateActiveRuntime();
                        appCumMessages += app.getBroadcastMessageCounter();
                        appCost += (app.getSolutionCost() - nominal) / nominal;

                        iappCumRuntime += iapp.getWallclockRuntime();
                        iappCumActive += iapp.getAggregateActiveRuntime();
                        iappCumMessages += iapp.getBroadcastMessageCounter();
                        iappCost += (iapp.getSolutionCost() - nominal) / nominal;

                        iapp2CumRuntime += iapp2.getWallclockRuntime();
                        iapp2CumActive += iapp2.getAggregateActiveRuntime();
                        iapp2CumMessages += iapp2.getBroadcastMessageCounter();
                        iapp2Cost += (iapp2.getSolutionCost() - nominal) / nominal;

                        out.write(nAgents + ";" + seed + ";"
                                + ca.getConflictCount() + ";"
                                + ca.getNominalCost() + ";"
                                + odid.getRuntimeMs() + ";"
                                + odid.getFinalCost() + ";"
                                + ca.getRuntime()  + ";"
                                + ca.getFinalCost() + ";"

                                + spp.getWallclockRuntime() + ";"
                                + spp.getAggregateActiveRuntime() + ";"
                                + spp.getAggregateIdleRuntime() + ";"
                                + spp.getBroadcastMessageCounter() + ";"
                                + spp.getRoundCounter() + ";"
                                + spp.getPlannerInvovationCounter() + ";"
                                + spp.getPlannerCumulativeExpandedStates() + ";"
                                + spp.getPlannerCumulativeRuntimeSec() + ";"
                                + spp.getCommunicationCumulativeRuntime() + ";"
                                + spp.getConflictCheckingCumulativeRuntime() + ";"
                                + spp.getAvgConstraintsPerPlannerInvocation() + ";"
                                + spp.getSolutionCost() + ";"

                                + app.getWallclockRuntime() + ";"
                                + app.getAggregateActiveRuntime() + ";"
                                + app.getAggregateIdleRuntime() + ";"
                                + app.getBroadcastMessageCounter() + ";"
                                + app.getPlannerInvovationCounter() + ";"
                                + app.getPlannerCumulativeExpandedStates() + ";"
                                + app.getPlannerCumulativeRuntimeSec() + ";"
                                + app.getCommunicationCumulativeRuntime() + ";"
                                + app.getConflictCheckingCumulativeRuntime() + ";"
                                + app.getAvgConstraintsPerPlannerInvocation() + ";"
                                + app.getSolutionCost() + ";"

                                + iapp.getWallclockRuntime() + ";"
                                + iapp.getAggregateActiveRuntime() + ";"
                                + iapp.getAggregateIdleRuntime() + ";"
                                + iapp.getBroadcastMessageCounter() + ";"
                                + iapp.getPlannerInvovationCounter() + ";"
                                + iapp.getPlannerCumulativeExpandedStates() + ";"
                                + iapp.getPlannerCumulativeRuntimeSec() + ";"
                                + iapp.getCommunicationCumulativeRuntime() + ";"
                                + iapp.getConflictCheckingCumulativeRuntime() + ";"
                                + iapp.getSolutionCost() + ";"

                                + iapp2.getWallclockRuntime() + ";"
                                + iapp2.getAggregateActiveRuntime() + ";"
                                + iapp2.getAggregateIdleRuntime() + ";"
                                + iapp2.getBroadcastMessageCounter() + ";"
                                + iapp2.getPlannerInvovationCounter() + ";"
                                + iapp2.getPlannerCumulativeExpandedStates() + ";"
                                + iapp2.getPlannerCumulativeRuntimeSec() + ";"
                                + iapp2.getCommunicationCumulativeRuntime() + ";"
                                + iapp2.getConflictCheckingCumulativeRuntime() + ";"
                                + iapp2.getSolutionCost()
                                );


                        out.newLine();
                        out.flush();
                        completedRuns++;

                        System.out.println("CA: " + ca.getRuntime() + " SDPP: " + spp.getWallclockRuntime() + " ADPP:" + app.getWallclockRuntime() + " IADPP:" + iapp.getWallclockRuntime());

                        } else {
                            out.write(nAgents + ";" + seed + ";");
                            out.newLine();
                            failedRuns++;
                            LOGGER.info("****** DISCARDING THIS INSTANCE: ONE OF THE ALGORITHMS FAILED *****");
                        }


                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }



                }



                aggrOut.write(nAgents + ";" + completedRuns + ";" + failedRuns + ";"
                        + cumConflicts * params.APPROXIMATION_SAMPLING_INTERVAL / (float) completedRuns + ";"

                        + caCumRuntime / completedRuns + ";"
                        + sppCumRuntime / completedRuns + ";"
                        + appCumRuntime / completedRuns + ";"
                        + iappCumRuntime / completedRuns + ";"
                        + iapp2CumRuntime / completedRuns + ";"

                        + sppCumActive / (double) completedRuns + ";"
                        + appCumActive / (double) completedRuns + ";"
                        + iappCumActive / (double) completedRuns + ";"
                        + iapp2CumActive / (double) completedRuns + ";"

                        + sppCumMessages / (double) completedRuns + ";"
                        + appCumMessages / (double) completedRuns + ";"
                        + iappCumMessages / (double) completedRuns + ";"
                        + iapp2CumMessages / (double) completedRuns + ";"

                        + caCost / (double) completedRuns + ";"
                        + sppCost / (double) completedRuns + ";"
                        + appCost / (double) completedRuns + ";"
                        + iappCost / (double) completedRuns + ";"
                        + iapp2Cost / (double) completedRuns + ";"

                        + caFailures / (double) completedRuns + ";"
                        + sppFailures / (double) completedRuns + ";"
                        + appFailures / (double) completedRuns + ";"
                        + iappFailures / (double) completedRuns + ";"
                        + iapp2Failures / (double) completedRuns + ";"

                        );

                aggrOut.newLine();
                aggrOut.flush();

            }

            out.flush();

        } catch (IOException e) {
            e.printStackTrace();
        }

        LOGGER.info("********* EXPERIMENT FINISHED ********");

    }

}

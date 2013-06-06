package cz.agents.admap.creator;
import java.awt.Color;
import java.io.File;
import java.io.FileInputStream;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Properties;

import javax.vecmath.Point2d;

import org.apache.log4j.Logger;
import org.apache.log4j.PropertyConfigurator;

import tt.discrete.vis.TrajectoryLayer;
import tt.discrete.vis.TrajectoryLayer.TrajectoryProvider;
import tt.euclid2i.Point;
import tt.euclid2i.Region;
import tt.euclid2i.probleminstance.Environment;
import tt.euclid2i.vis.ProjectionTo2d;
import tt.euclid2i.vis.RegionsLayer;
import tt.euclid2i.vis.RegionsLayer.RegionsProvider;
import tt.euclidtime3i.vis.TimeParameter;
import tt.euclidtime3i.vis.TimeParameterProjectionTo2d;
import tt.jointeuclid2ni.probleminstance.EarliestArrivalProblem;
import tt.jointeuclid2ni.probleminstance.RandomProblem;
import tt.jointeuclid2ni.probleminstance.SuperconflictProblem;
import tt.util.AgentColors;
import tt.vis.LabeledPointLayer;
import tt.vis.LabeledPointLayer.LabeledPoint;
import tt.vis.LabeledPointLayer.LabeledPointsProvider;
import tt.vis.ParameterControlLayer;
import cz.agents.admap.agent.ADPPDGAgent;
import cz.agents.admap.agent.Agent;
import cz.agents.alite.common.event.DurativeEvent;
import cz.agents.alite.common.event.DurativeEventHandler;
import cz.agents.alite.common.event.DurativeEventProcessor;
import cz.agents.alite.communication.InboxBasedCommunicator;
import cz.agents.alite.communication.channel.CommunicationChannelException;
import cz.agents.alite.communication.channel.DirectCommunicationChannel;
import cz.agents.alite.communication.channel.DirectCommunicationChannel.ReceiverTable;
import cz.agents.alite.communication.eventbased.ConcurrentProcessCommunicationChannel;
import cz.agents.alite.creator.Creator;
import cz.agents.alite.simulation.ConcurrentProcessSimulation;
import cz.agents.alite.vis.VisManager;
import cz.agents.alite.vis.VisManager.SceneParams;
import cz.agents.alite.vis.layer.common.ColorLayer;
import cz.agents.alite.vis.layer.common.VisInfoLayer;


public class ScenarioCreator implements Creator {

    ////////////////////////////////////////////////////////////////////////

    public static void main(String[] args) {
        ScenarioCreator creator = new ScenarioCreator();
        creator.init(args);
        creator.create("default", Scenario.SUPERCONFLICT, Method.ADPPDG, 6, 961, true);
    }

    ///////////////////////////////////////////////////////////////////////


    @Override
    public void create() {
        if (args.length > 1) {
            createFromArgs();
        }
    }

    static Logger LOGGER = Logger.getLogger(ScenarioCreator.class);

    private static final int ITERATION_DELAY = 50;
    private static final int AGENT_BODY_RADIUS = 40;
    private static final int MAX_TIME = 2000;

    enum Method {
        ADPP, /* Asynchronous Decentralized Prioritized Planning */
        ADPPDG, /* Asynchronous Decentralized Prioritized Planning with Dynamic Grouping */
        DSA, /* Stochastic Prioritized Planning */
        MGM}

    enum Scenario {
        RANDOM,
        SUPERCONFLICT,
        GEESE
    }

    private String[] args;
    private EarliestArrivalProblem problem;

    @SuppressWarnings("unused")
    @Override
    public void init(String[] args) {

        this.args = args;

        // Set-up the logger
        String overrideLogFileName = null;

        Properties prop = new Properties();

        String propFile = null;
        if ((new File("log4j.properties")).isFile()) {
            propFile = "log4j.properties";
        } else {
            if ((new File("resources" + File.separator + "log4j" + File.separator + "log4j.properties")).isFile()) {
                propFile = "resources" + File.separator + "log4j" + File.separator + "log4j.properties";
            }

        }
        if (propFile != null) {
            try {
                prop.load(new FileInputStream(propFile));
            } catch (Exception ex){
                ex.printStackTrace();
            }
            if (overrideLogFileName != null) {
                prop.setProperty("log4j.appender.R.File", overrideLogFileName);
            }
            PropertyConfigurator.configure(prop);

        }
    }

    public void createFromArgs() {
        //for (String arg : args) { System.out.println(arg); }
        String experimentId = args[1];
        String scenario = args[2];
        String algorithm = args[3];
        int nAgents = Integer.parseInt(args[4]);
        int seed = Integer.parseInt(args[5]);
        boolean showVis = Boolean.parseBoolean(args[6]);

        create(experimentId, Scenario.valueOf(scenario), Method.valueOf(algorithm), nAgents, seed, showVis);
    }

    public void create(String experimentId, Scenario scenario, Method method, int nAgents, int seed, boolean showVis) {

        switch (scenario) {
            case RANDOM:
                problem = new RandomProblem(new Environment(1000, 1000, 0, 300, seed), nAgents, AGENT_BODY_RADIUS, seed);
                break;
            case SUPERCONFLICT:
                problem = new SuperconflictProblem(nAgents,AGENT_BODY_RADIUS);
                break;
            default:
                throw new RuntimeException("Unknown scenario");
        }


        if (showVis) {
            createVisualization();
        }

        if (method == Method.ADPPDG) {
            solveADPPDG(problem,  showVis);
        }
    }

    private void solveADPPDG(final EarliestArrivalProblem problem, boolean showVis) {

        // Create agents
        List<Agent> agents = new LinkedList<Agent>();
        for (int i=0; i<problem.getStarts().length; i++) {
            agents.add(new ADPPDGAgent(
                    "a" + new DecimalFormat("00").format(i),
                    problem.getStart(i),
                    problem.getTargetPoint(i),
                    problem.getEnvironment(),
                    problem.getAgentSizeRadius()));
        }

        List<String> agentNames =  new ArrayList<String>(agents.size());
        for (Agent agent : agents) {
            agentNames.add(agent.getName());
        }

        // Create concurrent process simulation
        final ConcurrentProcessSimulation concurrentSimulation = new ConcurrentProcessSimulation();
        concurrentSimulation.setPrintouts(1000);


        // Create the communication channels and communicators for each agent
        ReceiverTable receiverTable = new DirectCommunicationChannel.DefaultReceiverTable();
        Map<String, List<Long>> inboxCounters = new HashMap<String, List<Long>>();
        for (Agent agent : agents) {
            InboxBasedCommunicator communicator = new InboxBasedCommunicator(agent.getName());

            try {
                communicator.setChannel(new ConcurrentProcessCommunicationChannel(communicator, concurrentSimulation, receiverTable, inboxCounters));
            } catch (CommunicationChannelException e) {
                e.printStackTrace();
            }
            agent.setCommunicator(communicator, agentNames);
        }

        // Run simulation of concurrent computation
        for (final Agent agent : agents) {
               concurrentSimulation.addEvent(0, agent.getName(), new DurativeEventHandler() {
                   @Override
                   public long handleEvent(DurativeEvent event) {
                       agent.start();
                       return COUNT_SYSTEM_NANOS;
                   }

                   @Override
                   public DurativeEventProcessor getEventProcessor() {
                       return concurrentSimulation;
                   }
               });
           }

         // *** run simulation ***

         concurrentSimulation.run();

         // **** create visio ****

         if (showVis) {
             int color = 0;
             final TimeParameter timeParameter = new TimeParameter(10);

             VisManager.registerLayer(ParameterControlLayer.create(timeParameter));

             for (final Agent agent: agents) {

                 // create visio
                 VisManager.registerLayer(TrajectoryLayer.create(new TrajectoryProvider<Point>() {

                    @Override
                    public tt.discrete.Trajectory<Point> getTrajectory() {
                        return agent.getCurrentTrajectory();
                    }
                }, new ProjectionTo2d(), AgentColors.getColorForAgent(color), 1, MAX_TIME, 'g'));

                VisManager.registerLayer(tt.euclidtime3i.vis.RegionsLayer.create(
                    new tt.euclidtime3i.vis.RegionsLayer.RegionsProvider() {
                        @Override
                        public Collection<tt.euclidtime3i.Region> getRegions() {
                             Collection<tt.euclidtime3i.Region> regions = Collections.singletonList(agent.getOccupiedRegion());
                             return regions;

                        }
                }, new TimeParameterProjectionTo2d(timeParameter), AgentColors.getColorForAgent(color), AgentColors.getColorForAgent(color)));

                color++;
            }

         }



        /*
        reportResult(Method.ADMAP.toString(), problem, path != null,
                path != null ? path.getWeight() : Double.POSITIVE_INFINITY,
                astar.getExpandedStatesCount(),
                expandedStatesLimit);*/
    }

    /*
    private void reportResult(String alg, ShortestPathProblem problem, boolean foundSolution,
            double solutionQuality, long iterationsToFirstSolution, long iterationLimit) {
        System.out.println(alg + ";" + problem.getSeed() + ";" + Analyzer.getProblemInstanceCharacteristics(problem)
                + ";" + foundSolution + ";"
                + solutionQuality + ";" + iterationsToFirstSolution + ";" + iterationLimit);
    } */


    private void createVisualization() {
        VisManager.setInitParam("Trajectory Tools Vis", 1024, 768);
        VisManager.setSceneParam(new SceneParams() {

            @Override
            public Point2d getDefaultLookAt() {
                return new Point2d(500, 500);
            }

            @Override
            public double getDefaultZoomFactor() {
                return 0.5;
            }

        });
        VisManager.init();

        // background
        VisManager.registerLayer(ColorLayer.create(Color.WHITE));

        // KeyToggleLayer graphToggle = KeyToggleLayer.create("g");
        // graphToggle.addSubLayer(GraphLayer.create(new GraphProvider<Waypoint,
        // SpatialManeuver>() {
        //
        // @Override
        // public Graph<Waypoint, SpatialManeuver> getGraph() {
        // return graph;
        // }}, Color.GRAY, Color.GRAY, 1, 4));
        // graphToggle.setEnabled(true);
        //
        // VisManager.registerLayer(graphToggle);

        // VisManager.registerLayer(NodeIdLayer.create(graph, Color.GRAY, 1,
        // "n"));

        VisManager.registerLayer(RegionsLayer.create(
                new RegionsProvider() {

                    @Override
                    public Collection<Region> getRegions() {
                        LinkedList<Region> list = new LinkedList<Region>();
                        list.add(problem.getBounds());
                        return list;
                    }

                }, Color.BLACK, Color.WHITE));

        VisManager.registerLayer(RegionsLayer.create(
                new RegionsProvider() {

                    @Override
                    public Collection<Region> getRegions() {
                        return problem.getObstacles();
                    }

                }, Color.GRAY, Color.GRAY));

        VisManager.registerLayer(LabeledPointLayer.create(new LabeledPointsProvider<Point>() {

            @Override
            public Collection<LabeledPoint<Point>> getLabeledPoints() {
                LinkedList<LabeledPoint<Point>> list = new LinkedList<LabeledPoint<Point>>();

                for (int i=0; i < problem.getStarts().length; i++) {
                    list.add(new LabeledPoint<Point>(problem.getStarts()[i], "s"+i));
                }
                return list;
            }

        }, new tt.euclid2i.vis.ProjectionTo2d(), Color.BLUE));

        VisManager.registerLayer(RegionsLayer.create(
                new RegionsProvider() {

                    @Override
                    public Collection<Region> getRegions() {
                        LinkedList<Region> list = new LinkedList<Region>();

                        for (int i=0; i < problem.getTargetRegions().length; i++) {
                            list.add(problem.getTargetRegions()[i]);
                        }

                        return list;
                    }

                }, Color.PINK, Color.PINK));

        VisManager.registerLayer(LabeledPointLayer.create(new LabeledPointsProvider<Point>() {

            @Override
            public Collection<LabeledPoint<Point>> getLabeledPoints() {
                LinkedList<LabeledPoint<Point>> list = new LinkedList<LabeledPoint<Point>>();

                for (int i=0; i < problem.getTargetRegions().length; i++) {
                    list.add(new LabeledPoint<Point>(problem.getTargetPoint(i), "g"+i));
                }
                return list;
            }

        }, new tt.euclid2i.vis.ProjectionTo2d(), Color.RED));

        // Overlay
        VisManager.registerLayer(VisInfoLayer.create());
    }
}

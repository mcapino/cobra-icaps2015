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
import cz.agents.admap.agent.ADOPTAgent;
import cz.agents.admap.agent.ADPPDGAgent;
import cz.agents.admap.agent.Agent;
import cz.agents.admap.agent.DSAAgent;
import cz.agents.admap.agent.adopt.NotCollidingConstraint;
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
        creator.create("default", Scenario.SUPERCONFLICT, Method.ADPPDG, 2, 967, true);
    }

    ///////////////////////////////////////////////////////////////////////

    static Logger LOGGER = Logger.getLogger(ScenarioCreator.class);

    private static final int AGENT_BODY_RADIUS = 50;
    private static final int MAX_TIME = 2000;

    enum Method {
        ADPP,   /* Asynchronous Decentralized Prioritized Planning */
        ADPPDG, /* Asynchronous Decentralized Prioritized Planning with Dynamic Grouping */
        DSA,    /* Stochastic Prioritized Planning */
        MGM,
        ADOPT}

    enum Scenario {
        RANDOM_IN_FREESPACE,
        RANDOM_WITH_OBSTACLES,
        SUPERCONFLICT,
        GEESE
    }

    private String[] args;
    private EarliestArrivalProblem problem;

    @Override
    public void create() {
        if (args.length > 1) {
            createFromArgs();
        }
    }

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
            case RANDOM_IN_FREESPACE:
                problem = new RandomProblem(new Environment(1000, 1000, 0, 300, seed), nAgents, AGENT_BODY_RADIUS, seed);
                break;
            case RANDOM_WITH_OBSTACLES:
                problem = new RandomProblem(new Environment(1000, 1000, 10, 250, seed), nAgents, AGENT_BODY_RADIUS, seed);
                break;
            case SUPERCONFLICT:
                problem = new SuperconflictProblem(nAgents,AGENT_BODY_RADIUS);
                break;
            default:
                throw new RuntimeException("Unknown scenario");
        }


        if (showVis) {
            initVisualization();
            visualizeProblem();
        }

        switch (method) {
            case ADPPDG:
                solveADPPDG(problem, showVis);
                break;

            case DSA:
                solveDSA(problem, showVis);
                break;

            case ADOPT:
                solveADOPT(problem, showVis);
                break;

            default:
                throw new RuntimeException("Unknown method");

        }

    }

    private void solveADPPDG(final EarliestArrivalProblem problem, boolean showVis) {
        solve(problem, new AgentFactory() {

            @Override
            public Agent createAgent(String name, Point start, Point target,
                    Environment env, int agentBodyRadius) {
                return new ADPPDGAgent(name, start, target, env, agentBodyRadius);
            }
        }, showVis);
    }

    private void solveDSA(final EarliestArrivalProblem problem, boolean showVis) {
        solve(problem, new AgentFactory() {

            @Override
            public Agent createAgent(String name, Point start, Point target,
                    Environment env, int agentBodyRadius) {
                return new DSAAgent(name, start, target, env, agentBodyRadius, 0.3);
            }
        }, showVis);
    }

    private void solveADOPT(final EarliestArrivalProblem problem, boolean showVis) {
        solve(problem, new AgentFactory() {

            @Override
            public Agent createAgent(String name, Point start, Point target,
                    Environment env, int agentBodyRadius) {
                return new ADOPTAgent(name, start, target, env, agentBodyRadius, new NotCollidingConstraint());
            }
        }, showVis);
    }

    interface AgentFactory {
        Agent createAgent(String name, Point start, Point target, Environment env, int agentBodyRadius);
    }

    private void solve(final EarliestArrivalProblem problem, final AgentFactory agentFactory, boolean showVis) {

        // Create agents
        List<Agent> agents = new LinkedList<Agent>();
        for (int i=0; i<problem.getStarts().length; i++) {
            agents.add(agentFactory.createAgent(
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

               // start agents
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

               // start periodic ticks
               final long tickPeriod = (long) 1e9;
               final long simulateUntil = 120 * (long) 1e9;
               DurativeEventHandler tickhandler =  new DurativeEventHandler() {
                   @Override
                   public long handleEvent(DurativeEvent event) {
                       long lastEventStartedAtNanos = System.nanoTime();
                       agent.tick(concurrentSimulation.getWallclockRuntime());
                       long duration = System.nanoTime() - lastEventStartedAtNanos;

                       if (concurrentSimulation.getWallclockRuntime() < simulateUntil) {
                           concurrentSimulation.addEvent(event.getTime() + tickPeriod, agent.getName(), this);
                       }
                       return tickPeriod + duration;
                   }

                   @Override
                   public DurativeEventProcessor getEventProcessor() {
                       return concurrentSimulation;
                   }
               };

               concurrentSimulation.addEvent(1, agent.getName(), tickhandler);
           }



         // **** create visio of agents ****
         if (showVis) {
             visualizeAgents(agents);
         }

         // *** run simulation ***
         concurrentSimulation.run();
    }






    /*
    private void reportResult(String alg, ShortestPathProblem problem, boolean foundSolution,
            double solutionQuality, long iterationsToFirstSolution, long iterationLimit) {
        System.out.println(alg + ";" + problem.getSeed() + ";" + Analyzer.getProblemInstanceCharacteristics(problem)
                + ";" + foundSolution + ";"
                + solutionQuality + ";" + iterationsToFirstSolution + ";" + iterationLimit);
    } */


    private void initVisualization() {
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

        // Overlay
        VisManager.registerLayer(VisInfoLayer.create());
    }

    private void visualizeProblem() {
        // background
        VisManager.registerLayer(ColorLayer.create(Color.WHITE));

        VisManager.registerLayer(RegionsLayer.create(
                new RegionsProvider() {

                    @Override
                    public Collection<Region> getRegions() {
                        LinkedList<Region> list = new LinkedList<Region>();
                        list.add(problem.getBounds());
                        return list;
                    }

                }, Color.BLACK, Color.WHITE));

        Color inflatedRegionsColor = new Color(250,250,250);
        VisManager.registerLayer(RegionsLayer.create(
                new RegionsProvider() {

                    @Override
                    public Collection<Region> getRegions() {
                        return problem.getInflatedObstacles();
                    }

                }, inflatedRegionsColor, inflatedRegionsColor));

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
    }

    private void visualizeAgents(List<Agent> agents) {
         int agentIndex = 0;
         final TimeParameter timeParameter = new TimeParameter(10);

         VisManager.registerLayer(ParameterControlLayer.create(timeParameter));

         for (final Agent agent: agents) {

             // create visio
             VisManager.registerLayer(TrajectoryLayer.create(new TrajectoryProvider<Point>() {

                @Override
                public tt.discrete.Trajectory<Point> getTrajectory() {
                    return agent.getCurrentTrajectory();
                }
            }, new ProjectionTo2d(), AgentColors.getColorForAgent(agentIndex), 1, MAX_TIME, 'g'));

            VisManager.registerLayer(tt.euclidtime3i.vis.RegionsLayer.create(
                new tt.euclidtime3i.vis.RegionsLayer.RegionsProvider() {
                    @Override
                    public Collection<tt.euclidtime3i.Region> getRegions() {
                         Collection<tt.euclidtime3i.Region> regions = Collections.singletonList(agent.getOccupiedRegion());
                         return regions;

                    }
            }, new TimeParameterProjectionTo2d(timeParameter), AgentColors.getColorForAgent(agentIndex), AgentColors.getColorForAgent(agentIndex)));

            final Point labelLocation = problem.getStart(agentIndex);
            VisManager.registerLayer(LabeledPointLayer.create(new LabeledPointsProvider<tt.euclid2i.Point>() {

                @Override
                public Collection<LabeledPoint<tt.euclid2i.Point>> getLabeledPoints() {
                    Collection<LabeledPoint<tt.euclid2i.Point>> points = new LinkedList<LabeledPointLayer.LabeledPoint<tt.euclid2i.Point>>();
                    points.add(new LabeledPoint<tt.euclid2i.Point>(labelLocation, agent.getStatus()));
                    return points;
                }

            }, new ProjectionTo2d(), Color.BLACK));

            agentIndex++;
        }
    }
}

package cz.agents.admap.creator;
import java.awt.Color;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import org.apache.log4j.Logger;
import org.jgrapht.DirectedGraph;

import tt.discrete.vis.TrajectoryLayer;
import tt.discrete.vis.TrajectoryLayer.TrajectoryProvider;
import tt.euclid2i.Line;
import tt.euclid2i.Point;
import tt.euclid2i.Region;
import tt.euclid2i.probleminstance.Environment;
import tt.euclid2i.probleminstance.RandomEnvironment;
import tt.euclid2i.vis.ProjectionTo2d;
import tt.euclid2i.vis.RegionsLayer;
import tt.euclid2i.vis.RegionsLayer.RegionsProvider;
import tt.euclidtime3i.region.MovingCircle;
import tt.euclidtime3i.util.IntersectionChecker;
import tt.euclidtime3i.vis.TimeParameter;
import tt.euclidtime3i.vis.TimeParameterProjectionTo2d;
import tt.jointeuclid2ni.probleminstance.EarliestArrivalProblem;
import tt.jointeuclid2ni.probleminstance.EarliestArrivalProblemXMLDeserializer;
import tt.jointeuclid2ni.probleminstance.RandomProblem;
import tt.jointeuclid2ni.probleminstance.SuperconflictProblem;
import tt.jointeuclid2ni.probleminstance.VisUtil;
import tt.jointeuclid2ni.solver.Parameters;
import tt.util.AgentColors;
import tt.util.Args;
import tt.vis.LabeledPointLayer;
import tt.vis.LabeledPointLayer.LabeledPoint;
import tt.vis.LabeledPointLayer.LabeledPointsProvider;
import tt.vis.ParameterControlLayer;
import tt.vis.TimeParameterHolder;
import cz.agents.admap.agent.ADOPTAgent;
import cz.agents.admap.agent.ADPPAgent;
import cz.agents.admap.agent.ADPPDGAgent;
import cz.agents.admap.agent.Agent;
import cz.agents.admap.agent.DSAAgent;
import cz.agents.admap.agent.ORCAAgent;
import cz.agents.admap.agent.PlanningAgent;
import cz.agents.admap.agent.SDPPAgent;
import cz.agents.admap.agent.adopt.NotCollidingConstraint;
import cz.agents.alite.common.event.DurativeEvent;
import cz.agents.alite.common.event.DurativeEventHandler;
import cz.agents.alite.common.event.DurativeEventProcessor;
import cz.agents.alite.communication.InboxBasedCommunicator;
import cz.agents.alite.communication.channel.CommunicationChannelException;
import cz.agents.alite.communication.channel.DirectCommunicationChannel;
import cz.agents.alite.communication.channel.DirectCommunicationChannel.ReceiverTable;
import cz.agents.alite.communication.eventbased.ConcurrentProcessCommunicationChannel;
import cz.agents.alite.simulation.ConcurrentProcessSimulation;
import cz.agents.alite.vis.VisManager;
import cz.agents.alite.vis.layer.VisLayer;


public class ScenarioCreator {

    ////////////////////////////////////////////////////////////////////////

    public static void main(String[] args) {
        if (args.length > 1) {
        	createFromArgs(args);
        } else {
        	final int AGENT_BODY_RADIUS = 50;
        	EarliestArrivalProblem problem = createProblem(Scenario.RANDOM_IN_FREESPACE, 10, AGENT_BODY_RADIUS, 999);
        	create(problem, Method.ADPPDG, 10000, true, null);
        }
    }

    ///////////////////////////////////////////////////////////////////////

    static Logger LOGGER = Logger.getLogger(ScenarioCreator.class);


    private static final int MAX_TIME = 2000;

    enum Method {
        ADPP,   /* Asynchronous Decentralized Prioritized Planning */
        SDPP,
        ADPPDG, /* Asynchronous Decentralized Prioritized Planning with Dynamic Grouping */
        DSA,    /* Stochastic Prioritized Planning */
        MGM,
        ADOPT,
        ORCA}


    private static EarliestArrivalProblem problem;

    public static void createFromArgs(String[] args) {
    	Parameters params = new Parameters();
    	String xml = Args.getArgumentValue(args, "-problemfile", true);
    	String methodStr = Args.getArgumentValue(args, "-method", true);
    	String maxTimeStr = Args.getArgumentValue(args, "-maxtime", true);
    	params.showVis = Args.isArgumentSet(args, "-showvis");
    	params.verbose = Args.isArgumentSet(args, "-verbose");
    	String timeoutStr = Args.getArgumentValue(args, "-timeout", false);
    	


		File file = new File(xml);
	    params.fileName = file.getName();
	    // Load the PNG image as a background
	    File bgImgFile = new File(xml.replace(".xml", ".png"));
	    if (!bgImgFile.exists()) {
	    	bgImgFile = null;
	    }

	    try {
			problem = EarliestArrivalProblemXMLDeserializer.deserialize(new FileInputStream(file));
		} catch (FileNotFoundException e) {
			throw new RuntimeException(e);
		}

	    Method method = Method.valueOf(methodStr);

	    if (timeoutStr != null) {
	    	int timeout = Integer.parseInt(timeoutStr);
	    	killAt(System.currentTimeMillis() + timeout);
	    }
	    
	    int maxTime = Integer.parseInt(maxTimeStr);
	    

    	create(problem, method, maxTime, params.showVis, bgImgFile);
    }


    private static void killAt(final long killAtMs) {
    	Thread t = new Thread() {
			@Override
			public void run() {
				while (System.currentTimeMillis() < killAtMs) {
					try {
						Thread.sleep(10);
					} catch (InterruptedException e) {}
				}
				printSummary(false, null, 0);
				System.exit(0);
			}
    	};
    	t.start();
	}


	public static void create(EarliestArrivalProblem problem, Method method, final int maxTime, boolean showVis, File bgImageFile) {

        if (showVis) {
            VisUtil.initVisualization(problem, "Trajectory Tools ("+method.toString()+")", bgImageFile, 2);
            VisUtil.visualizeProblem(problem);
            if (problem.getPlanningGraph() != null) {
            	VisUtil.visualizeGraph(problem.getPlanningGraph(), null);
            }
        }

//        try {
//			System.in.read();
//		} catch (IOException e) {
//			// TODO Auto-generated catch block
//			e.printStackTrace();
//		}

        switch (method) {

	        case ADPP:
	            solveADPP(problem, maxTime, showVis);
	            break;
	            
	        case SDPP:
	            solveSDPP(problem, maxTime, showVis);
	            break;

            case ADPPDG:
                solveADPPDG(problem, maxTime, showVis);
                break;

            case DSA:
                solveDSA(problem, maxTime, showVis);
                break;

            case ADOPT:
                solveADOPT(problem, showVis);
                break;

            case ORCA:
                solveORCA(problem, showVis);
                break;

            default:
                throw new RuntimeException("Unknown method");

        }
    }

    enum Scenario {
    	RANDOM_IN_FREESPACE,
    	RANDOM_WITH_OBSTACLES,
    	SUPERCONFLICT,
    	GEESE
    }

	private static EarliestArrivalProblem createProblem(Scenario scenario, int nAgents, int agentBodyRadius, int seed) {

		EarliestArrivalProblem problem;

		switch (scenario) {
            case RANDOM_IN_FREESPACE:
                problem = new RandomProblem(new RandomEnvironment(1000, 1000, 0, 300, seed), nAgents, agentBodyRadius, seed);
                break;
            case RANDOM_WITH_OBSTACLES:
                problem = new RandomProblem(new RandomEnvironment(1000, 1000, 10, 250, seed), nAgents, agentBodyRadius, seed);
                break;
            case SUPERCONFLICT:
                problem = new SuperconflictProblem(nAgents, agentBodyRadius);
                break;
            default:
                throw new RuntimeException("Unknown scenario");
        }

		return problem;
	}

    private static void solveADPP(final EarliestArrivalProblem problem, final int maxTime, boolean showVis) {
        solve(problem, new AgentFactory() {
            @Override
            public Agent createAgent(String name, Point start, Point target,
                    Environment env, DirectedGraph<Point, Line> planningGraph, int agentBodyRadius) {

            	PlanningAgent agent = new ADPPAgent(name, start, target, env, agentBodyRadius, maxTime);
            	agent.setPlanningGraph(planningGraph);
                return agent;
            }
        }, showVis);
    }
    
    private static void solveSDPP(final EarliestArrivalProblem problem, final int maxTime, boolean showVis) {
        solve(problem, new AgentFactory() {
            @Override
            public Agent createAgent(String name, Point start, Point target,
                    Environment env, DirectedGraph<Point, Line> planningGraph, int agentBodyRadius) {

            	PlanningAgent agent = new SDPPAgent(name, start, target, env, agentBodyRadius, maxTime);
            	agent.setPlanningGraph(planningGraph);
                return agent;
            }
        }, showVis);
    }

    private static void solveADPPDG(final EarliestArrivalProblem problem, final int maxTime, boolean showVis) {
        solve(problem, new AgentFactory() {
            @Override
            public Agent createAgent(String name, Point start, Point target,
                    Environment env, DirectedGraph<Point, Line> planningGraph, int agentBodyRadius) {

            	ADPPDGAgent agent = new ADPPDGAgent(name, start, target, env, agentBodyRadius, maxTime);
            	agent.setPlanningGraph(planningGraph);
                return agent;
            }
        }, showVis);
    }

    private static void solveDSA(final EarliestArrivalProblem problem, final int maxTime, boolean showVis) {
        solve(problem, new AgentFactory() {

            @Override
            public Agent createAgent(String name, Point start, Point target,
                    Environment env, DirectedGraph<Point, Line> planningGraph, int agentBodyRadius) {
                return new DSAAgent(name, start, target, env, agentBodyRadius, 0.3, maxTime);
            }
        }, showVis);
    }

    private static void solveADOPT(final EarliestArrivalProblem problem, boolean showVis) {
        solve(problem, new AgentFactory() {

            @Override
            public Agent createAgent(String name, Point start, Point target,
                    Environment env, DirectedGraph<Point, Line> planningGraph, int agentBodyRadius) {
                return new ADOPTAgent(name, start, target, env, agentBodyRadius, new NotCollidingConstraint());
            }
        }, showVis);
    }

    private static void solveORCA(final EarliestArrivalProblem problem, final boolean showVis) {
        solve(problem, new AgentFactory() {

            @Override
            public Agent createAgent(String name, Point start, Point target,
                    Environment env, DirectedGraph<Point, Line> planningGraph, int agentBodyRadius) {
            	ORCAAgent agent = new ORCAAgent(name, start, target, env, planningGraph, agentBodyRadius, showVis);
            	return agent;
            }
        }, showVis);
    }

    interface AgentFactory {
        Agent createAgent(String name, Point start, Point target, Environment env, DirectedGraph<Point, Line> planningGraph, int agentBodyRadius);
    }

    private static void solve(final EarliestArrivalProblem problem, final AgentFactory agentFactory, boolean showVis) {

        // Create agents
        final List<Agent> agents = new LinkedList<Agent>();
        for (int i=0; i<problem.getStarts().length; i++) {
            agents.add(agentFactory.createAgent(
                    "a" + new DecimalFormat("00").format(i),
                    problem.getStart(i),
                    problem.getTarget(i),
                    problem.getEnvironment(),
                    problem.getPlanningGraph(),
                    problem.getBodyRadius(i)));
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

        // The list of agents that havent reached their goals yet
        final Collection<String> unfinishedAgents = new LinkedList<String>();
        for (Agent agent : agents) {
        	unfinishedAgents.add(agent.getName());
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
               final long tickPeriod = (long) 1e8;
               final long simulateUntil = 120 * (long) 1e9;
               DurativeEventHandler tickhandler =  new DurativeEventHandler() {
                   @Override
                   public long handleEvent(DurativeEvent event) {
                       long lastEventStartedAtNanos = System.nanoTime();
                       agent.tick(event.getTime());
                       long duration = System.nanoTime() - lastEventStartedAtNanos;

                       // Check whether all agents reached their goals
                       if (agent.isFinished()) {
                    	   unfinishedAgents.remove(agent.getName());
                       }

                       if (unfinishedAgents.isEmpty()) {
                    	   concurrentSimulation.clearQueue();
                    	   // We are done!
                    	   printSummary(true, agents, concurrentSimulation.getWallclockRuntime()/1000000);
                       } else {
	                       if (concurrentSimulation.getWallclockRuntime() < simulateUntil) {
	                    	   int noOfTicksToSkip = (int) (duration / tickPeriod); // if the tick handling takes more than tickPeriod, we need to skip some ticks

	                           concurrentSimulation.addEvent(event.getTime() + (noOfTicksToSkip+1)*tickPeriod, agent.getName(), this);
	                       }
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
             visualizeAgents(problem, agents);
             visualizeConflicts(agents);
         }

         // *** run simulation ***
         concurrentSimulation.run();
    }

    private static void printSummary(boolean succeeded, List<Agent> agents, long wallClockRuntimeMs) {

    	if (succeeded) {
	    	double cost = 0;
	    	int msgsSent = 0;
	    	for (Agent agent : agents) {
	    		LOGGER.info(agent.getName() + " cost: " + String.format("%.2f", agent.getCurrentTrajectory().getCost()) + " Messages sent: " + agent.getMessageSentCounter() );
	    		cost += agent.getCurrentTrajectory().getCost();
	    		msgsSent += agent.getMessageSentCounter();
	    	}
	    	System.out.println(String.format("%.2f", cost) + ";" + wallClockRuntimeMs + ";" + msgsSent);
    	} else {
    		System.out.println("inf;0;0");
    	}

    }

    private static void visualizeAgents(EarliestArrivalProblem problem, List<Agent> agents) {
         int agentIndex = 0;

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
            }, new TimeParameterProjectionTo2d(TimeParameterHolder.time), AgentColors.getColorForAgent(agentIndex), AgentColors.getColorForAgent(agentIndex)));

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
    
    private static void visualizeConflicts(final List<Agent> agents) {
        
    	VisManager.registerLayer(RegionsLayer.create(new RegionsProvider() {
			
			@Override
			public Collection<? extends Region> getRegions() {
				
				Collection<MovingCircle> mcs = new LinkedList<MovingCircle>();
				
				for(Agent agent : agents) {
					if (agent.getCurrentTrajectory() != null) {
						mcs.add((MovingCircle) agent.getOccupiedRegion());
					}
				}
				
				return IntersectionChecker.computeAllPairwiseConflicts(mcs, 10);
			}
		}, Color.RED));
    	
    	int agentIndex = 0;

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
           }, new TimeParameterProjectionTo2d(TimeParameterHolder.time), AgentColors.getColorForAgent(agentIndex), AgentColors.getColorForAgent(agentIndex)));

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

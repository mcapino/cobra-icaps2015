package cz.agents.admap.creator;
import java.awt.Color;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import org.apache.log4j.Logger;
import org.jgrapht.DirectedGraph;
import org.jgrapht.GraphPath;
import org.jgrapht.alg.AStarShortestPathSimple;
import org.jgrapht.util.HeuristicToGoal;

import tt.discrete.vis.TrajectoryLayer;
import tt.discrete.vis.TrajectoryLayer.TrajectoryProvider;
import tt.euclid2i.EvaluatedTrajectory;
import tt.euclid2i.Line;
import tt.euclid2i.Point;
import tt.euclid2i.Region;
import tt.euclid2i.Trajectory;
import tt.euclid2i.probleminstance.Environment;
import tt.euclid2i.region.Circle;
import tt.euclid2i.trajectory.LineSegmentsConstantSpeedTrajectory;
import tt.euclid2i.vis.ProjectionTo2d;
import tt.euclid2i.vis.RegionsLayer;
import tt.euclid2i.vis.RegionsLayer.RegionsProvider;
import tt.euclidtime3i.region.MovingCircle;
import tt.euclidtime3i.util.IntersectionChecker;
import tt.jointeuclid2ni.probleminstance.EarliestArrivalProblem;
import tt.jointeuclid2ni.probleminstance.EarliestArrivalProblemXMLDeserializer;
import tt.jointeuclid2ni.probleminstance.VisUtil;
import tt.jointeuclid2ni.solver.Parameters;
import tt.util.AgentColors;
import tt.util.Args;
import tt.util.Counters;
import tt.vis.FastAgentsLayer;
import tt.vis.FastAgentsLayer.ColorProvider;
import tt.vis.FastAgentsLayer.TrajectoriesProvider;
import tt.vis.LabeledPointLayer;
import tt.vis.LabeledPointLayer.LabeledPoint;
import tt.vis.LabeledPointLayer.LabeledPointsProvider;
import tt.vis.TimeParameterHolder;
import cz.agents.admap.agent.ADOPTAgent;
import cz.agents.admap.agent.ADPPAgent;
import cz.agents.admap.agent.ADPPDGAgent;
import cz.agents.admap.agent.Agent;
import cz.agents.admap.agent.BestResponse;
import cz.agents.admap.agent.DSAAgent;
import cz.agents.admap.agent.FixedTrajectoryAgent;
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


public class ScenarioCreator {

    ////////////////////////////////////////////////////////////////////////

    public static void main(String[] args) {
        createFromArgs(args);
    }

    ///////////////////////////////////////////////////////////////////////

    static Logger LOGGER = Logger.getLogger(ScenarioCreator.class);


    private static final int MAX_TIME = 2000;

    enum Method {
    	BASE, 	/* Only computes single-agent paths, does not resolve conflicts. Uses spatial planner. */
    	BASEST, /* Only computes single-agent paths, does not resolve conflicts. Uses space-time planner. */
    	CPP,	/* Centralized Prioritized Planning */
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
    	params.maxTime = Integer.parseInt(maxTimeStr);
    	params.showVis = Args.isArgumentSet(args, "-showvis");
    	params.verbose = Args.isArgumentSet(args, "-verbose");
    	String timeoutStr = Args.getArgumentValue(args, "-timeout", false);
        params.summaryPrefix = Args.getArgumentValue(args, "-summaryprefix", false, "");

		File file = new File(xml);
	    params.fileName = file.getName();
	    // Load the PNG image as a background
	    File bgImgFile = new File(xml.replace(".xml", ".png"));
	    if (!bgImgFile.exists()) {
	    	bgImgFile = null;
	    }
	    params.bgImageFile = bgImgFile;

	    try {
			problem = EarliestArrivalProblemXMLDeserializer.deserialize(new FileInputStream(file));
		} catch (FileNotFoundException e) {
			throw new RuntimeException(e);
		}

	    Method method = Method.valueOf(methodStr);

	    if (timeoutStr != null) {
	    	int timeout = Integer.parseInt(timeoutStr);
	    	killAt(params.summaryPrefix, System.currentTimeMillis() + timeout);
	    }
	    
    	create(problem, method, params);
    }


    private static void killAt(final String summaryPrefix, final long killAtMs) {
    	Thread t = new Thread() {
			@Override
			public void run() {
				while (System.currentTimeMillis() < killAtMs) {
					try {
						Thread.sleep(10);
					} catch (InterruptedException e) {}
				}
				printSummary(summaryPrefix, false, null, 0);
				System.exit(0);
			}
    	};
    	t.start();
	}


	public static void create(EarliestArrivalProblem problem, Method method, final Parameters params) {
		
        if (params.showVis) {
            VisUtil.initVisualization(problem, "Trajectory Tools ("+method.toString()+")", params.bgImageFile, 2);
            VisUtil.visualizeProblem(problem);
            if (problem.getPlanningGraph() != null) {
            	VisUtil.visualizeGraph(problem.getPlanningGraph(), null);
            }
        }

//      try {
//			System.in.read();
//		} catch (IOException e) {
//			// TODO Auto-generated catch block
//			e.printStackTrace();
//		}

        switch (method) {
        
	        case BASE:
	            solveBASE(problem, params, false);
	            break;
	        
	        case BASEST:
	            solveBASE(problem, params, true);
	            break;
	        
	        case CPP:
	            solveCPP(problem, params);
	            break;

	        case ADPP:
	            solveADPP(problem, params);
	            break;
	            
	        case SDPP:
	            solveSDPP(problem, params);
	            break;

            case ADPPDG:
                solveADPPDG(problem, params);
                break;

            case DSA:
                solveDSA(problem, params);
                break;

            case ADOPT:
                solveADOPT(problem, params);
                break;

            case ORCA:
                solveORCA(problem, params);
                break;

            default:
                throw new RuntimeException("Unknown method");

        }
    }
	
    private static void solveBASE(final EarliestArrivalProblem problem, final Parameters params, boolean useSpaceTimePlanner) {
        final EvaluatedTrajectory[] trajs = new EvaluatedTrajectory[problem.nAgents()];
        
        long startedAtMs = System.currentTimeMillis();
        boolean suceeded = true;
        
		for (int i = 0; i < problem.nAgents(); i++) {
            final int iFinal = i;
            
            EvaluatedTrajectory traj = null;
            if (useSpaceTimePlanner) {
                Collection<tt.euclid2i.Region> sObst = new LinkedList<tt.euclid2i.Region>();
                Collection<tt.euclidtime3i.Region> dObst = new LinkedList<tt.euclidtime3i.Region>();
                traj = BestResponse.computeBestResponse(problem.getStart(i), problem.getTarget(i), problem.getPlanningGraph(), sObst, dObst, params.maxTime);
            } else{
                GraphPath<Point, Line> path = AStarShortestPathSimple.findPathBetween(problem.getPlanningGraph(), new HeuristicToGoal<Point>() {

    				@Override
    				public double getCostToGoalEstimate(Point current) {
    					return current.distance(problem.getTarget(iFinal));
    				}
    			}, problem.getStart(i), problem.getTarget(i));

                if (path != null) {
                	traj = new LineSegmentsConstantSpeedTrajectory(0, path, 1, params.maxTime);
                }
            }
			
			if (traj != null) {
				trajs[i] = traj;
			} else {
				suceeded = false;
			}
		}
		
		long finishedAtMs = System.currentTimeMillis();
		
		solve(problem, new AgentFactory() {
			int i=0;
			
			@Override
			public Agent createAgent(String name, Point start, Point target,
					Environment env, DirectedGraph<Point, Line> planningGraph,
					int agentBodyRadius) {
				FixedTrajectoryAgent agent = new FixedTrajectoryAgent("a"+i, problem.getStart(i), problem.getTarget(i) , problem.getEnvironment(), problem.getBodyRadius(i), trajs[i]);
				i++;
				return agent;
			}
		}, params);
		
		
		final List<Agent> agents = new LinkedList<Agent>();
		for (int i = 0; i < problem.nAgents(); i++) {
			agents.add(new FixedTrajectoryAgent("a"+i, problem.getStart(i), problem.getTarget(i) , problem.getEnvironment(), problem.getBodyRadius(i), trajs[i]));
		}
		printSummary(params.summaryPrefix, suceeded, agents, finishedAtMs - startedAtMs);
		
		if (!params.showVis) {
			System.exit(0);
		}
    }
	
    private static void solveCPP(final EarliestArrivalProblem problem, final Parameters params) {
    	final int RADIUS_GRACE = 1;
        final EvaluatedTrajectory[] trajs = new EvaluatedTrajectory[problem.nAgents()];
        
        long startedAtMs = System.currentTimeMillis();
        boolean suceeded = true;
        
		for (int i = 0; i < problem.nAgents(); i++) {
            Collection<tt.euclid2i.Region> sObst = new LinkedList<tt.euclid2i.Region>();
            Collection<tt.euclidtime3i.Region> dObst = new LinkedList<tt.euclidtime3i.Region>();
                        
            for (int j=0; j<problem.nAgents(); j++) {
            	if (j < i) {
            		dObst.add(new MovingCircle(trajs[j], problem.getBodyRadius(i) + problem.getBodyRadius(j) + RADIUS_GRACE));
            	} else if (j > i) {
            		sObst.add(new Circle(problem.getStart(j), problem.getBodyRadius(i) + problem.getBodyRadius(j) + RADIUS_GRACE));
            	}
            }
            
			EvaluatedTrajectory traj = BestResponse.computeBestResponse(problem.getStart(i), problem.getTarget(i), problem.getPlanningGraph(), sObst, dObst, params.maxTime);
			
			if (traj != null) {
				trajs[i] = traj;
			} else {
				suceeded = false;
			}
		}
		
		long finishedAtMs = System.currentTimeMillis();
		
		
		solve(problem, new AgentFactory() {
			int i=0;
			
			@Override
			public Agent createAgent(String name, Point start, Point target,
					Environment env, DirectedGraph<Point, Line> planningGraph,
					int agentBodyRadius) {
				FixedTrajectoryAgent agent = new FixedTrajectoryAgent("a"+i, problem.getStart(i), problem.getTarget(i) , problem.getEnvironment(), problem.getBodyRadius(i), trajs[i]);
				i++;
				return agent;
			}
		}, params);
		
		
		final List<Agent> agents = new LinkedList<Agent>();
		for (int i = 0; i < problem.nAgents(); i++) {
			agents.add(new FixedTrajectoryAgent("a"+i, problem.getStart(i), problem.getTarget(i) , problem.getEnvironment(), problem.getBodyRadius(i), trajs[i]));
		}
		printSummary(params.summaryPrefix, suceeded, agents, finishedAtMs - startedAtMs);
		
		if (!params.showVis) {
			System.exit(0);
		}
    }
    
    private static void solveADPP(final EarliestArrivalProblem problem, final Parameters params) {
        solve(problem, new AgentFactory() {
            @Override
            public Agent createAgent(String name, Point start, Point target,
                    Environment env, DirectedGraph<Point, Line> planningGraph, int agentBodyRadius) {

            	PlanningAgent agent = new ADPPAgent(name, start, target, env, agentBodyRadius, params.maxTime);
            	agent.setPlanningGraph(planningGraph);
                return agent;
            }
        }, params);
    }
    
    private static void solveSDPP(final EarliestArrivalProblem problem, final Parameters params) {
        solve(problem, new AgentFactory() {
            @Override
            public Agent createAgent(String name, Point start, Point target,
                    Environment env, DirectedGraph<Point, Line> planningGraph, int agentBodyRadius) {

            	PlanningAgent agent = new SDPPAgent(name, start, target, env, agentBodyRadius, params.maxTime);
            	agent.setPlanningGraph(planningGraph);
                return agent;
            }
        }, params);
    }

    private static void solveADPPDG(final EarliestArrivalProblem problem, final Parameters params) {
        solve(problem, new AgentFactory() {
            @Override
            public Agent createAgent(String name, Point start, Point target,
                    Environment env, DirectedGraph<Point, Line> planningGraph, int agentBodyRadius) {

            	ADPPDGAgent agent = new ADPPDGAgent(name, start, target, env, agentBodyRadius, params.maxTime);
            	agent.setPlanningGraph(planningGraph);
                return agent;
            }
        }, params);
    }

    private static void solveDSA(final EarliestArrivalProblem problem, final Parameters params) {
        solve(problem, new AgentFactory() {

            @Override
            public Agent createAgent(String name, Point start, Point target,
                    Environment env, DirectedGraph<Point, Line> planningGraph, int agentBodyRadius) {
                return new DSAAgent(name, start, target, env, agentBodyRadius, 0.3, params.maxTime);
            }
        }, params);
    }

    private static void solveADOPT(final EarliestArrivalProblem problem, final Parameters params) {
        solve(problem, new AgentFactory() {

            @Override
            public Agent createAgent(String name, Point start, Point target,
                    Environment env, DirectedGraph<Point, Line> planningGraph, int agentBodyRadius) {
                return new ADOPTAgent(name, start, target, env, agentBodyRadius, new NotCollidingConstraint());
            }
        }, params);
    }

    private static void solveORCA(final EarliestArrivalProblem problem, final Parameters params) {
        solve(problem, new AgentFactory() {

            @Override
            public Agent createAgent(String name, Point start, Point target,
                    Environment env, DirectedGraph<Point, Line> planningGraph, int agentBodyRadius) {
            	ORCAAgent agent = new ORCAAgent(name, start, target, env, planningGraph, agentBodyRadius, params.showVis);
            	return agent;
            }
        }, params);
    }

    interface AgentFactory {
        Agent createAgent(String name, Point start, Point target, Environment env, DirectedGraph<Point, Line> planningGraph, int agentBodyRadius);
    }

    private static void solve(final EarliestArrivalProblem problem, final AgentFactory agentFactory, final Parameters params) {

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
                       if (agent.isTerminated()) {
                    	   unfinishedAgents.remove(agent.getName());
                       }

                       if (unfinishedAgents.isEmpty()) {
                    	   concurrentSimulation.clearQueue();
                    	   // We are done!
                    	   printSummary(params.summaryPrefix, true, agents, concurrentSimulation.getWallclockRuntime()/1000000);
                    	   if (!params.showVis) {
                    		   System.exit(0);
                    	   }
                       } else {
	                       if (concurrentSimulation.getWallclockRuntime() < simulateUntil) {
	                    	   int noOfTicksToSkip = (int) (duration / tickPeriod); // if the tick handling takes more than tickPeriod, we need to skip some ticks

	                           concurrentSimulation.addEvent(event.getTime() + (noOfTicksToSkip+1)*tickPeriod, agent.getName(), this);
	                       }
                       }
                       return duration;
                   }

                   @Override
                   public DurativeEventProcessor getEventProcessor() {
                       return concurrentSimulation;
                   }
               };

               concurrentSimulation.addEvent(1, agent.getName(), tickhandler);
           }

         // **** create visio of agents ****
         if (params.showVis) {
             visualizeAgents(problem, agents);
             visualizeConflicts(agents);
         }

         // *** run simulation ***
         concurrentSimulation.run();
    }

    private static void printSummary(String prefix, boolean succeeded, List<Agent> agents, long timeToConvergeMs) {

    	if (succeeded) {
	    	double cost = 0;
	    	int msgsSent = 0;
	    	for (Agent agent : agents) {
	    		
	    		
				LOGGER.info(agent.getName()
						+ " cost: "
						+ String.format("%.2f", agent.getCurrentTrajectory().getCost()) + 
						" Messages sent: "+ agent.getMessageSentCounter()						
						);
	    		cost += agent.getCurrentTrajectory().getCost();
	    		msgsSent += agent.getMessageSentCounter();
	    	}
	    	System.out.println(prefix + String.format("%.2f", cost) + ";" + timeToConvergeMs + ";" + msgsSent + ";" + Counters.expandedStatesCounter);
    	} else {
    		System.out.println(prefix + "inf;NA;NA;NA");
    	}

    }

    private static void visualizeAgents(final EarliestArrivalProblem problem, final List<Agent> agents) {
         int agentIndex = 0;

         for (final Agent agent: agents) {

             // visualize trajectories
             VisManager.registerLayer(TrajectoryLayer.create(new TrajectoryProvider<Point>() {

                @Override
                public tt.discrete.Trajectory<Point> getTrajectory() {
                    return agent.getCurrentTrajectory();
                }
            }, new ProjectionTo2d(), AgentColors.getColorForAgent(agentIndex), 1, MAX_TIME, 'g'));

            VisManager.registerLayer(FastAgentsLayer.create(new TrajectoriesProvider() {
				@Override
				public tt.discrete.Trajectory<Point>[] getTrajectories() {
					tt.discrete.Trajectory<Point>[] trajs = new Trajectory[problem.nAgents()];
					int i=0;
					for (Agent agent : agents) {
						if (agent.getOccupiedRegion() != null) {
							trajs[i++] = ((MovingCircle)agent.getOccupiedRegion()).getTrajectory();
						}
					}
					return trajs;
				}
				
				@Override
				public int[] getBodyRadiuses() {
					int[] radii = new int[problem.nAgents()];
					int i=0;
					for (Agent agent : agents) {
						if (agent.getOccupiedRegion() != null) {
							radii[i++] = ((MovingCircle)agent.getOccupiedRegion()).getRadius();	
						}
					}
					return radii;
				}
			}, new ColorProvider() {
				
				@Override
				public Color getColor(int i) {
					return  AgentColors.getColorForAgent(i);
				}
			}, TimeParameterHolder.time));
            
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
   }
}

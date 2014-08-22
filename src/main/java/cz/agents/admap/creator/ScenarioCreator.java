package cz.agents.admap.creator;
import java.awt.Color;
import java.awt.image.ReplicateScaleFilter;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;

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
import tt.euclid2i.discretization.L2Heuristic;
import tt.euclid2i.probleminstance.Environment;
import tt.euclid2i.region.Circle;
import tt.euclid2i.trajectory.LineSegmentsConstantSpeedTrajectory;
import tt.euclid2i.util.SeparationDetector;
import tt.euclid2i.vis.ProjectionTo2d;
import tt.euclid2i.vis.RegionsLayer;
import tt.euclid2i.vis.RegionsLayer.RegionsProvider;
import tt.euclidtime3i.ShortestPathHeuristic;
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
import cz.agents.alite.common.event.ActivityLogEntry;
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
import cz.agents.alite.vis.layer.toggle.KeyToggleLayer;


public class ScenarioCreator {

	////////////////////////////////////////////////////////////////////////

    public static void main(String[] args) {
        createFromArgs(args);
    }

    ///////////////////////////////////////////////////////////////////////
    
    static long startedAt;
    static Logger LOGGER = Logger.getLogger(ScenarioCreator.class);
	final static int RADIUS_GRACE = 1;

    enum Method {
    	BASE, 	/* Only computes single-agent paths, does not resolve conflicts. Uses spatial planner. */
    	BASEST, /* Only computes single-agent paths, does not resolve conflicts. Uses space-time planner. */
    	PP,		/* Centralized Prioritized Planning */
    	RPP,	/* Centralized Revised Prioritized Planning */
    	ADPP,	/* Asynchronous Decentralized Prioritized Planning */
        ADRPP,  /* Asynchronous Decentralized Revised Prioritized Planning */
        SDPP,   /* Synchronous  Decentralized Prioritized Planning */
        SDRPP,	/* Asynchronous Decentralized Revised Prioritized Planning */
        ADPPDG, /* Asynchronous Decentralized Prioritized Planning with Dynamic Grouping */
        DSA,    /* Stochastic Prioritized Planning */
        MGM,
        ADOPT,
        ORCA}


    private static EarliestArrivalProblem problem;

    private static final long TICK_INTERVAL_NS = 100 /*ms*/ * 1000000;
    
    public static void createFromArgs(String[] args) {
    	startedAt = System.currentTimeMillis();
    	Parameters params = new Parameters();
    	 	
    	String xml = Args.getArgumentValue(args, "-problemfile", true);
    	String methodStr = Args.getArgumentValue(args, "-method", true);
    	String maxTimeStr = Args.getArgumentValue(args, "-maxtime", true);
    	params.maxTime = Integer.parseInt(maxTimeStr);
    	String timeStepStr = Args.getArgumentValue(args, "-timestep", true);
    	params.timeStep = Integer.parseInt(timeStepStr);
    	params.showVis = Args.isArgumentSet(args, "-showvis");
    	params.verbose = Args.isArgumentSet(args, "-verbose");
    	String timeoutStr = Args.getArgumentValue(args, "-timeout", false);
        params.summaryPrefix = Args.getArgumentValue(args, "-summaryprefix", false, "");
        params.activityLogFile = Args.getArgumentValue(args, "-activitylog", false, null);
        String bgImgFileName = Args.getArgumentValue(args, "-bgimg", false, null);
        
        
		File file = new File(xml);
	    params.fileName = file.getName();
	    
	    // Load the PNG image as a background, if provided
	    if (bgImgFileName != null) {
		    File bgImgFile = new File(bgImgFileName);
		    if (bgImgFile.exists()) {
		    	params.bgImageFile = bgImgFile;
		    }        	
        }

	    try {
			problem = EarliestArrivalProblemXMLDeserializer.deserialize(new FileInputStream(file));
		} catch (FileNotFoundException e) {
			throw new RuntimeException(e);
		}

	    Method method = Method.valueOf(methodStr);
	    
	    params.noOfClusters = computeNoOfClusters(problem, params);
	    LOGGER.debug("Number of clusters: " + params.noOfClusters);
	    
	    params.runtimeDeadlineMs = 3600*1000; /* default timeout is 1 hour */
	    if (timeoutStr != null) {
	    	int timeout = Integer.parseInt(timeoutStr);
	    	params.runtimeDeadlineMs = timeout;
	    	killAt(System.currentTimeMillis() + timeout, params.summaryPrefix, params.noOfClusters);
	    }
	    
    	create(problem, method, params);
    }


    private static void killAt(final long killAtMs, final String summaryPrefix, final int clusters) {
    	Thread t = new Thread() {
			@Override
			public void run() {
				while (System.currentTimeMillis() < killAtMs) {
					try {
						Thread.sleep(10);
					} catch (InterruptedException e) {}
				}
				printSummary(summaryPrefix, Status.TIMEOUT, null, 0, clusters);
				System.exit(0);
			}
    	};
    	t.start();
	}


	public static void create(EarliestArrivalProblem problem, Method method, final Parameters params) {
		
        if (params.showVis) {
            VisUtil.initVisualization(problem, "Trajectory Tools ("+method.toString()+")", params.bgImageFile, params.timeStep/2);
            VisUtil.visualizeProblem(problem);
        }

        switch (method) {
        
	        case BASE:
	            solveBASE(problem, params, false);
	            break;
	        
	        case BASEST:
	            solveBASE(problem, params, true);
	            break;
	        
	        case PP:
	            solvePP(problem, false, params);
	            break;

	        case RPP:
	            solvePP(problem, true, params);
	            break;
	        
	        case ADPP:
	            solveADPP(problem, false, params);
	            break;    
	            
	        case ADRPP:
	            solveADPP(problem, true, params);
	            break;
	        
	        case SDPP:
	            solveSDPP(problem, false, params);
	            break;
	            
	        case SDRPP:
	            solveSDPP(problem, true, params);
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
        Status status = Status.SUCCESS;
        
		for (int i = 0; i < problem.nAgents(); i++) {
            final int iFinal = i;
            
            EvaluatedTrajectory traj = null;
            if (useSpaceTimePlanner) {
                Collection<tt.euclid2i.Region> sObst = new LinkedList<tt.euclid2i.Region>();
                Collection<tt.euclidtime3i.Region> dObst = new LinkedList<tt.euclidtime3i.Region>();
                HeuristicToGoal<tt.euclidtime3i.Point> heuristic = new ShortestPathHeuristic(problem.getPlanningGraph(), problem.getTarget(i));
				traj = BestResponse.computeBestResponse(problem.getStart(i), problem.getTarget(i), problem.getPlanningGraph(), heuristic ,sObst, dObst, params.maxTime, params.timeStep);
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
				status = Status.FAIL;
			}
		}
		
		long finishedAtMs = System.currentTimeMillis();
		
		solve(problem, new AgentFactory() {
			int i=0;
			
			@Override
			public Agent createAgent(String name, int i, Point start, Point target,
					Environment env, DirectedGraph<Point, Line> planningGraph,
					int agentBodyRadius) {
				FixedTrajectoryAgent agent = new FixedTrajectoryAgent("a"+i, problem.getStart(i), problem.getTarget(i) , problem.getEnvironment(), problem.getBodyRadius(i), trajs[i]);
				i++;
				return agent;
			}
		}, TICK_INTERVAL_NS, (long) (params.runtimeDeadlineMs*1e6), params);
		
		
		final List<Agent> agents = new LinkedList<Agent>();
		for (int i = 0; i < problem.nAgents(); i++) {
			agents.add(new FixedTrajectoryAgent("a"+i, problem.getStart(i), problem.getTarget(i) , problem.getEnvironment(), problem.getBodyRadius(i), trajs[i]));
		}
		printSummary(params.summaryPrefix, status, agents, finishedAtMs - startedAtMs, params.noOfClusters);
		
		if (!params.showVis) {
			System.exit(0);
		}
    }
	
    private static void solvePP(final EarliestArrivalProblem problem, boolean avoidStartRegions, final Parameters params) {
        final EvaluatedTrajectory[] trajs = new EvaluatedTrajectory[problem.nAgents()];
        
        long programStartedAtNs = System.nanoTime();
        Status status = Status.SUCCESS;
        
        Collection<ActivityLogEntry> activityLog = new LinkedList<ActivityLogEntry>();
        
		for (int i = 0; i < problem.nAgents(); i++) {
			long activityStart = System.nanoTime();
			int expandedStatesBefore = Counters.expandedStatesCounter;
            Collection<tt.euclid2i.Region> sObst = new LinkedList<tt.euclid2i.Region>();
            Collection<tt.euclidtime3i.Region> dObst = new LinkedList<tt.euclidtime3i.Region>();
                        
            for (int j=0; j<problem.nAgents(); j++) {
            	if (j < i) {
            		dObst.add(new MovingCircle(trajs[j], problem.getBodyRadius(i) + problem.getBodyRadius(j) + RADIUS_GRACE));
            	} else if (j > i && avoidStartRegions) {
            		sObst.add(new Circle(problem.getStart(j), problem.getBodyRadius(i) + problem.getBodyRadius(j) + RADIUS_GRACE));
            	}
            }
            
			HeuristicToGoal<tt.euclidtime3i.Point> heuristic = new ShortestPathHeuristic(problem.getPlanningGraph(), problem.getTarget(i));
			EvaluatedTrajectory traj = BestResponse.computeBestResponse(problem.getStart(i), problem.getTarget(i), problem.getPlanningGraph(), heuristic , sObst, dObst, params.maxTime, params.timeStep);
			
			int expandedStatesAfter = Counters.expandedStatesCounter;
			
			long activityDuration = System.nanoTime() - activityStart;
			
			activityLog.add(new ActivityLogEntry("a"+ new DecimalFormat("00").format(i), 
					ActivityLogEntry.Type.EVENT_HANDLED, activityStart - programStartedAtNs, activityDuration, expandedStatesAfter - expandedStatesBefore));
			
			if (traj != null) {
				trajs[i] = traj;
				LOGGER.debug("Agent " + i + " successfully finished planning in " + activityDuration/1000000 + "ms");
			} else {
				LOGGER.debug("Agent " + i + " FAILED to find a trajectory! Spent " + activityDuration/1000000 + "ms planning.");
				status = Status.FAIL;
				break;
			}
			
		}
		
		long finishedAtMs = System.nanoTime();
		
		
		solve(problem, new AgentFactory() {
			@Override
			public Agent createAgent(String name, int i,Point start, Point target,
					Environment env, DirectedGraph<Point, Line> planningGraph,
					int agentBodyRadius) {
				FixedTrajectoryAgent agent = new FixedTrajectoryAgent("a" + new DecimalFormat("00").format(i), problem.getStart(i), problem.getTarget(i) , problem.getEnvironment(), problem.getBodyRadius(i), trajs[i]);
				i++;
				return agent;
			}
		}, TICK_INTERVAL_NS, (long) (params.runtimeDeadlineMs*1e6), params);
		
		
		final List<Agent> agents = new LinkedList<Agent>();
		for (int i = 0; i < problem.nAgents(); i++) {
			agents.add(new FixedTrajectoryAgent("a"+i, problem.getStart(i), problem.getTarget(i) , problem.getEnvironment(), problem.getBodyRadius(i), trajs[i]));
		}
		printSummary(params.summaryPrefix, status, agents, (finishedAtMs - programStartedAtNs)/1000000, params.noOfClusters);
		
		if (params.activityLogFile != null) {
			saveActivityLog(activityLog, params.activityLogFile);
		}		
		
		if (!params.showVis) {
			System.exit(0);
		}
    }
    
    private static void solveADPP(final EarliestArrivalProblem problem, final boolean avoidStartRegions, final Parameters params) {
        solve(problem, new AgentFactory() {
            @Override
            public Agent createAgent(String name, int i, Point start, Point target,
                    Environment env, DirectedGraph<Point, Line> planningGraph, int agentBodyRadius) {

            	Collection<Region> sObst = new LinkedList<>();
            	
                for (int j=0; j<problem.nAgents(); j++) {
                	if (j > i && avoidStartRegions) {
                		sObst.add(new Circle(problem.getStart(j), problem.getBodyRadius(j)));
                	}
                }
            	
				PlanningAgent agent = new ADPPAgent(name, start, target, env, agentBodyRadius, params.maxTime, params.timeStep, sObst);
            	agent.setPlanningGraph(planningGraph);
                return agent;
            }
        }, TICK_INTERVAL_NS, (long) (params.runtimeDeadlineMs*1e6), params);
    }
    
    private static void solveSDPP(final EarliestArrivalProblem problem, final boolean avoidStartRegions, final Parameters params) {
        solve(problem, new AgentFactory() {
            @Override
            public Agent createAgent(String name, int i, Point start, Point target,
                    Environment env, DirectedGraph<Point, Line> planningGraph, int agentBodyRadius) {

            	Collection<Region> sObst = new LinkedList<>();
            	
                for (int j=0; j<problem.nAgents(); j++) {
                	if (j > i && avoidStartRegions) {
                		sObst.add(new Circle(problem.getStart(j), problem.getBodyRadius(j)));
                	}
                }
                
				PlanningAgent agent = new SDPPAgent(name, start, target, env, agentBodyRadius, params.maxTime, params.timeStep, sObst);
            	agent.setPlanningGraph(planningGraph);
                return agent;
            }
        }, TICK_INTERVAL_NS, (long) (params.runtimeDeadlineMs*1e6), params);
    }

    private static void solveADPPDG(final EarliestArrivalProblem problem, final Parameters params) {
        solve(problem, new AgentFactory() {
            @Override
            public Agent createAgent(String name, int i, Point start, Point target,
                    Environment env, DirectedGraph<Point, Line> planningGraph, int agentBodyRadius) {

            	ADPPDGAgent agent = new ADPPDGAgent(name, start, target, env, agentBodyRadius, params.maxTime, params.timeStep);
            	agent.setPlanningGraph(planningGraph);
                return agent;
            }
        }, TICK_INTERVAL_NS, (long) (params.runtimeDeadlineMs*1e6), params);
    }

    private static void solveDSA(final EarliestArrivalProblem problem, final Parameters params) {
        solve(problem, new AgentFactory() {

            @Override
            public Agent createAgent(String name, int i,  Point start, Point target,
                    Environment env, DirectedGraph<Point, Line> planningGraph, int agentBodyRadius) {
                return new DSAAgent(name, start, target, env, agentBodyRadius, 0.3, params.maxTime, params.timeStep);
            }
        }, TICK_INTERVAL_NS, (long) (params.runtimeDeadlineMs*1e6), params);
    }

    private static void solveADOPT(final EarliestArrivalProblem problem, final Parameters params) {
        solve(problem, new AgentFactory() {

            @Override
            public Agent createAgent(String name, int i, Point start, Point target,
                    Environment env, DirectedGraph<Point, Line> planningGraph, int agentBodyRadius) {
                return new ADOPTAgent(name, start, target, env, agentBodyRadius, new NotCollidingConstraint());
            }
        }, TICK_INTERVAL_NS, (long) (params.runtimeDeadlineMs*1e6), params);
    }

    private static void solveORCA(final EarliestArrivalProblem problem, final Parameters params) {
        solve(problem, new AgentFactory() {

            @Override
            public Agent createAgent(String name, int i, Point start, Point target,
                    Environment env, DirectedGraph<Point, Line> planningGraph, int agentBodyRadius) {
            	ORCAAgent agent = new ORCAAgent(name, start, target, env, planningGraph, agentBodyRadius, params.maxTime, params.timeStep, params.showVis);
            	return agent;
            }
        }, 1000000000, (long) (params.maxTime*1e9), params);
    }

    interface AgentFactory {
        Agent createAgent(String name, int priority, Point start, Point target, Environment env, DirectedGraph<Point, Line> planningGraph, int agentBodyRadius);
    }

    private static void solve(final EarliestArrivalProblem problem, final AgentFactory agentFactory, final long tickPeriodNs, final long simulateTicksUntilNs, final Parameters params) {
    	
        // Create agents
        final List<Agent> agents = new LinkedList<Agent>();
        for (int i=0; i<problem.getStarts().length; i++) {
            agents.add(agentFactory.createAgent(            		
                    "a" + new DecimalFormat("00").format(i),
                    i,
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
        concurrentSimulation.setPrintouts(1000000);
        concurrentSimulation.setKeepActivityLog(params.activityLogFile != null);

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
               DurativeEventHandler tickhandler =  new DurativeEventHandler() {
                   @Override
                   public long handleEvent(DurativeEvent event) {
                       long lastEventStartedAtNanos = System.nanoTime();
                       agent.tick(event.getTime());
                       long duration = System.nanoTime() - lastEventStartedAtNanos;

                       // Check whether all agents reached their goals
                       if (agent.isGlobalTerminationDetected()) {
                    	   unfinishedAgents.remove(agent.getName());
                       }

                       if (unfinishedAgents.isEmpty()) {
                    	   concurrentSimulation.clearQueue();
                    	   // We are done!
                    	   printSummary(params.summaryPrefix, agent.hasSucceeded() ? Status.SUCCESS : Status.FAIL, agents, concurrentSimulation.getWallclockRuntime()/1000000, params.noOfClusters);
                    	   
                           if (params.activityLogFile != null) {
                        	   saveActivityLog(concurrentSimulation.getActivityLog(), params.activityLogFile);
                           }
                    	   
                    	   if (!params.showVis) {
                    		   System.exit(0);
                    	   }
                       } else {
	                       if (concurrentSimulation.getWallclockRuntime() < simulateTicksUntilNs) {
	                    	   int noOfTicksToSkip = (int) (duration / tickPeriodNs); // if the tick handling takes more than tickPeriod, we need to skip some ticks

	                           concurrentSimulation.addEvent(event.getTime() + (noOfTicksToSkip+1)*tickPeriodNs, agent.getName(), this);
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

         // **** create Visio of agents ****
         if (params.showVis) {
             visualizeAgents(problem, agents);
             visualizeConflicts(agents);
         }

// wait for a key press before solving ...         
//      try {
//			System.in.read();
//		} catch (IOException e) {}

         // *** run simulation ***
         concurrentSimulation.run();

    }

    private static void saveActivityLog(Collection<ActivityLogEntry> activityLog, String activityLogFile) {
    	try {
			File file = new File(activityLogFile);
			BufferedWriter writer = new BufferedWriter(new FileWriter(file));		
			writer.write("process;type;start;duration;expstates\n");
	
	    	for (ActivityLogEntry entry : activityLog) {
	    		writer.write(entry.processName + ";" + entry.type + ";" + 
	    				String.format("%.4f", entry.startTime/1000000000f) + ";" +  
	    				String.format("%.4f", entry.duration/1000000000f) + ";" +
	    				entry.expandedStates + "\n");
	    	}
	    	writer.close();
		} catch (IOException e) {
			e.printStackTrace();
		} finally {
		}
	}
    
    enum Status {SUCCESS, FAIL, TIMEOUT}

	private static void printSummary(String prefix, Status status, List<Agent> agents, long timeToConvergeMs, int clusters) {

    	
	    	double cost = 0;
	    	int msgsSent = 0;
	    	int totalReplannings = 0;
	    	if (agents != null) {
		    	for (Agent agent : agents) {
					LOGGER.info(agent.getName()
							+ " cost: "
							+ (agent.getCurrentTrajectory() != null ? String.format("%.2f", agent.getCurrentTrajectory().getCost()) : "inf") + 
							" Messages sent: "+ agent.getMessageSentCounter()						
							);
					
		    		cost += agent.getCurrentTrajectory() != null ? agent.getCurrentTrajectory().getCost() : 0;
		    		msgsSent += agent.getMessageSentCounter();
		    		
		    		if (agent instanceof PlanningAgent) {
		    			totalReplannings += ((PlanningAgent) agent).replanningCounter;
		    		}
		    		
		    	}
	    	}
	    	long scenarioSimulationRuntime = System.currentTimeMillis() - startedAt; 
	    	
	    	
	    	System.out.println(prefix + (status == Status.SUCCESS ? String.format("%.2f", cost) : "inf") + ";" + 
	    			status + ";" + scenarioSimulationRuntime + ";" + 
	    			timeToConvergeMs + ";" + msgsSent + ";" + 
	    			Counters.expandedStatesCounter + ";" + clusters + ";" + totalReplannings);
    }

    private static void visualizeAgents(final EarliestArrivalProblem problem, final List<Agent> agents) {
    	
    	 int MAX_TRAJ_DURATION = 5000;
         int agentIndex = 0;

         for (final Agent agent: agents) {
        	 
			 // visualize trajectories
             VisManager.registerLayer(KeyToggleLayer.create("t", true, TrajectoryLayer.create(new TrajectoryProvider<Point>() {

                @Override
                public tt.discrete.Trajectory<Point> getTrajectory() {
                    return agent.getCurrentTrajectory();
                }
            }, new ProjectionTo2d(), AgentColors.getColorForAgent(agentIndex), 1, MAX_TRAJ_DURATION, 'g')));

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
            
            agentIndex++;
        }
    }
    
    private static void visualizeConflicts(final List<Agent> agents) {
        
    	KeyToggleLayer conflictLayerToggle = KeyToggleLayer.create("c", false, RegionsLayer.create(new RegionsProvider() {
			
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
    	
    	VisManager.registerLayer(conflictLayerToggle);
   }
    
   static private int computeNoOfClusters(EarliestArrivalProblem problem, Parameters params) {
	   MovingCircle[] mcs = new MovingCircle[problem.nAgents()];
	   Set<Set<MovingCircle>> clusters = new HashSet<Set<MovingCircle>>(); 
	   
	   for (int i = 0; i < mcs.length; i++) {
           Collection<tt.euclid2i.Region> sObst = new LinkedList<tt.euclid2i.Region>();
           Collection<tt.euclidtime3i.Region> dObst = new LinkedList<tt.euclidtime3i.Region>();
           HeuristicToGoal<tt.euclidtime3i.Point> heuristic = new ShortestPathHeuristic(problem.getPlanningGraph(), problem.getTarget(i));
		   EvaluatedTrajectory traj = BestResponse.computeBestResponse(problem.getStart(i), problem.getTarget(i), problem.getPlanningGraph(), heuristic ,sObst, dObst, params.maxTime, params.timeStep);

		   mcs[i] = new MovingCircle(traj, problem.getBodyRadius(i));
		   
		   Set<Set<MovingCircle>> conflicting = getConflictingClusters(mcs[i], clusters);
		   
		   if (conflicting.size() == 0) {
			   clusters.add(Collections.singleton(mcs[i]));
		   } else {
			   assert conflicting.size() > 0;
			   
			   Set<MovingCircle> joinedCluster = new HashSet<MovingCircle>();
			   joinedCluster.add(mcs[i]);
			   // join all clusters
			   for (Set<MovingCircle> conflictingCluster : conflicting) {
				   clusters.remove(conflictingCluster);
				   joinedCluster.addAll(conflictingCluster);
			   }
			   
			   clusters.add(joinedCluster);
		   }
	   }
	   
	   return clusters.size();
   }


	static private Set<Set<MovingCircle>> getConflictingClusters(MovingCircle movingCircle, Set<Set<MovingCircle>> clusters) {
	
		Set<Set<MovingCircle>> conflictingClusters = new HashSet<Set<MovingCircle>>(); 
		for (Set<MovingCircle> cluster : clusters) {
			
			LinkedList<tt.euclidtime3i.Region> regions = new LinkedList<tt.euclidtime3i.Region>();
			for(MovingCircle mc : cluster) {
				regions.add(mc);
			}
			
			if (IntersectionChecker.intersect(movingCircle, regions)) {
				conflictingClusters.add(cluster);
			}
		}
		
		return conflictingClusters;
	}

}

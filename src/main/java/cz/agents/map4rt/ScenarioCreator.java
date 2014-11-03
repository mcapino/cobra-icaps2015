package cz.agents.map4rt;
import java.awt.Color;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import org.apache.log4j.Logger;
import org.jgrapht.DirectedGraph;

import tt.euclid2i.Line;
import tt.euclid2i.Point;
import tt.euclid2i.Trajectory;
import tt.euclid2i.probleminstance.Environment;
import tt.jointeuclid2ni.probleminstance.RelocationTask;
import tt.jointeuclid2ni.probleminstance.RelocationTaskCoordinationProblem;
import tt.jointeuclid2ni.probleminstance.TrajectoryCoordinationProblemXMLDeserializer;
import tt.jointeuclid2ni.probleminstance.VisUtil;
import tt.jointeuclid2ni.solver.Parameters;
import tt.util.AgentColors;
import tt.util.Args;
import tt.vis.FastTrajectoriesLayer;
import tt.vis.FastTrajectoriesLayer.ColorProvider;
import tt.vis.FastTrajectoriesLayer.TrajectoriesProvider;
import tt.vis.LabeledCircleLayer;
import tt.vis.TrajectoriesLayer;
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
import cz.agents.map4rt.agent.Agent;
import cz.agents.map4rt.agent.BaselineAgent;
import cz.agents.map4rt.agent.DFCFSAgent;
import cz.agents.map4rt.agent.ORCAAgent;
import cz.agents.map4rt.agent.PlanningAgent;


public class ScenarioCreator {
	
	/* Units: 
	 * time: 1 time unit = 1ms; 
	 * distance: 1 distance unit = depending on the map, 2cm typically. 
	 * speed: in du/ms (distance units / millisecond), typically 0.05 du/ms represents roughly 1m/1s 
	 */

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
    	DFCFS,  /* Distributed First-come First-served */
        ORCA}


    private static RelocationTaskCoordinationProblem problem;

    private static final long TICK_INTERVAL_NS = 100 /*ms*/ * 1000000;
    private static final float MAX_SPEED = 0.05f;
    
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
        String simSpeedStr = Args.getArgumentValue(args, "-simspeed", false, "1");
        params.simSpeed = Double.parseDouble(simSpeedStr);
                
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
			problem = TrajectoryCoordinationProblemXMLDeserializer.deserialize(new FileInputStream(file));
		} catch (FileNotFoundException e) {
			throw new RuntimeException(e);
		}

	    Method method = Method.valueOf(methodStr);
	    
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
				printSummary(summaryPrefix, Status.TIMEOUT, -1);
				System.exit(0);
			}
    	};
    	t.start();
	}


	public static void create(RelocationTaskCoordinationProblem problem, Method method, final Parameters params) {
		
        if (params.showVis) {
            VisUtil.initVisualization(problem.getEnvironment(), "Trajectory Tools ("+method.toString()+")", params.bgImageFile, params.timeStep/2);
            VisUtil.visualizeRelocationTaskCoordinationProblem(problem);
        }
        
        switch (method) {
        
	        case BASE:
	            solveBASE(problem, params);
	            break;

	        case DFCFS:
	            solveDFCFS(problem, params);
	            break;

	        case ORCA:
                solveORCA(problem, params);
                break;

            default:
                throw new RuntimeException("Unknown method");

        }
    }

	private static void solveBASE(final RelocationTaskCoordinationProblem problem, final Parameters params) {
        simulate(problem, new AgentFactory() {
            @Override
            public Agent createAgent(String name, int i, Point start, List<RelocationTask> tasks,
                    Environment env, DirectedGraph<Point, Line> planningGraph, int agentBodyRadius) {

				PlanningAgent agent = new BaselineAgent(name, start, tasks, env, planningGraph, agentBodyRadius, MAX_SPEED, params.maxTime, params.timeStep);
                return agent;
            }
        }, TICK_INTERVAL_NS, (long) (params.maxTime*1e6), params);
    }  
	
	private static void solveDFCFS(final RelocationTaskCoordinationProblem problem, final Parameters params) {
        simulate(problem, new AgentFactory() {
            @Override
            public Agent createAgent(String name, int i, Point start, List<RelocationTask> tasks,
                    Environment env, DirectedGraph<Point, Line> planningGraph, int agentBodyRadius) {

				PlanningAgent agent = new DFCFSAgent(name, start, tasks, env, planningGraph, agentBodyRadius, MAX_SPEED, params.maxTime, params.timeStep);
                return agent;
            }
        }, TICK_INTERVAL_NS, (long) (params.maxTime*1e6), params);
    } 
	
	private static void solveORCA(final RelocationTaskCoordinationProblem problem, final Parameters params) {
        simulate(problem, new AgentFactory() {
            @Override
            public Agent createAgent(String name, int i, Point start, List<RelocationTask> tasks,
                    Environment env, DirectedGraph<Point, Line> planningGraph, int agentBodyRadius) {

				Agent agent = new ORCAAgent(name, start, tasks, env, planningGraph, agentBodyRadius, MAX_SPEED, params.maxTime, params.timeStep, params.showVis);
				return agent;
            }
        }, TICK_INTERVAL_NS, (long) (params.maxTime*1e6), params);
    }    

    interface AgentFactory {
        Agent createAgent(String name, int priority, Point start, List<RelocationTask> tasks, Environment env, DirectedGraph<Point, Line> planningGraph, int agentBodyRadius);
    }

    private static void simulate(final RelocationTaskCoordinationProblem problem, final AgentFactory agentFactory, final long tickPeriodNs, 
    		final long simulateTicksUntilNs, final Parameters params) {
    	
        // Create agents
        final List<Agent> agents = new LinkedList<Agent>();
        for (int i=0; i<problem.getStarts().length; i++) {
            agents.add(agentFactory.createAgent(            		
                    "a" + new DecimalFormat("00").format(i),
                    i,
                    problem.getStart(i),
                    problem.getRelocationTasks(i),
                    problem.getEnvironment(),
                    problem.getPlanningGraph(),
                    problem.getBodyRadius(i)));
        }
        
        initAgentVisualization(agents);
        
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
        
        final long simulationStartedAtMs = System.currentTimeMillis();
        
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
                       
                	   // artificial delay to make the simulation run in real-time
                	   long eventTimeMs = (long) (event.getTime()/1000000);
                	   long shouldHappenAt = (long)(eventTimeMs/params.simSpeed) + simulationStartedAtMs;
                	   	
						if ((shouldHappenAt - System.currentTimeMillis()) > 0) {
							try {
								Thread.sleep(Math.max(
										shouldHappenAt - System.currentTimeMillis(),
										0));
							} catch (InterruptedException e) {
							}
						} 
                	   
                	   long lastEventStartedAtNanos = System.nanoTime();
                       agent.tick((int) (event.getTime() / 1000000));
                       long duration = System.nanoTime() - lastEventStartedAtNanos;

                       // Check whether all agents reached their goals
                       if (agent.hasCompletedAllTasks()) {
                    	   unfinishedAgents.remove(agent.getName());
                       }

                       if (unfinishedAgents.isEmpty()) {
                    	   concurrentSimulation.clearQueue();
                    	   // We are done!
                    	   printSummary(params.summaryPrefix, Status.SUCCESS, concurrentSimulation.getWallclockRuntime()/1000000);
                    	   
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

// wait for a key press before solving ...         
//      try {
//			System.in.read();
//		} catch (IOException e) {}
        
        
         // *** run simulation ***
         concurrentSimulation.run();

    }
    
    private static void initAgentVisualization(final List<Agent> agents) {
        
        // starts
        VisManager.registerLayer(LabeledCircleLayer.create(new LabeledCircleLayer.LabeledCircleProvider<tt.euclid2i.Point>() {

             @Override
             public Collection<LabeledCircleLayer.LabeledCircle<tt.euclid2i.Point>> getLabeledCircles() {
                 LinkedList<LabeledCircleLayer.LabeledCircle<tt.euclid2i.Point>> list = new LinkedList<LabeledCircleLayer.LabeledCircle<tt.euclid2i.Point>>();

                 for (int i = 0; i < agents.size(); i++) {
                     list.add(new LabeledCircleLayer.LabeledCircle<tt.euclid2i.Point>(agents.get(i).getCurrentPos(), agents.get(i).getAgentBodyRadius(), "" + i  , AgentColors.getColorForAgent(i)));
                 }

                 return list;
             }

         }, new tt.euclid2i.vis.ProjectionTo2d()));
        
        VisManager.registerLayer(FastTrajectoriesLayer.create(new TrajectoriesProvider() {
			
			@Override
			public Trajectory[] getTrajectories() {
				Trajectory[] trajsArr = new Trajectory[agents.size()];
				for (int i = 0; i < trajsArr.length; i++) {
					trajsArr[i] = agents.get(i).getCurrentTrajectory();
				}
				return trajsArr;
			}
		},new ColorProvider() {
			
			@Override
			public Color getColor(int i) {
				return AgentColors.getColorForAgent(i);
			}
		}, 3, 1000));
            	
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

	private static void printSummary(String prefix, Status status, long completedAt) {
	    	System.out.println(prefix + (status == Status.SUCCESS ? completedAt : "inf") + ";" );
    }

}

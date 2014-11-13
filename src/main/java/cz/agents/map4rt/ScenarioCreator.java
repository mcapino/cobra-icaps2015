package cz.agents.map4rt;
import java.awt.Color;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.text.DecimalFormat;
import java.util.Arrays;
import java.util.Collection;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Random;

import org.apache.log4j.Logger;
import org.jgrapht.DirectedGraph;

import tt.euclid2i.Line;
import tt.euclid2i.Point;
import tt.euclid2i.Trajectory;
import tt.euclid2i.probleminstance.Environment;
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
import cz.agents.alite.simulation.vis.SimulationControlLayer;
import cz.agents.alite.simulation.vis.SimulationControlLayer.SimulationControlProvider;
import cz.agents.alite.vis.VisManager;
import cz.agents.alite.vis.layer.toggle.KeyToggleLayer;
import cz.agents.map4rt.agent.Agent;
import cz.agents.map4rt.agent.CurrentTasks;
import cz.agents.map4rt.agent.COBRAAgent;
import cz.agents.map4rt.agent.ORCAAgent;
import cz.agents.map4rt.agent.PlanningAgent;


public class ScenarioCreator {
	
	/* 
	 * Units:
	 * time: 1 time unit = 1ms; 
	 * distance: 1 distance unit = depending on the map, 2cm typically. 
	 * speed: in du/ms (distance units / millisecond), typically 0.05 du/ms represents roughly 1m/1s 
	 */

	////////////////////////////////////////////////////////////////////////

    public static void main(String[] args) {
    	createFromArgs(args);
    }

    ///////////////////////////////////////////////////////////////////////
    
    static long simulationStartedAt;
    static Logger LOGGER = Logger.getLogger(ScenarioCreator.class);
		
    enum Method {
    	COBRA,  
        ORCA }


    private static RelocationTaskCoordinationProblem problem;
    
    public static void createFromArgs(String[] args) {
    	simulationStartedAt = System.currentTimeMillis();
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
        String nTasksStr = Args.getArgumentValue(args, "-ntasks", true);
    	params.nTasks = Integer.parseInt(nTasksStr);
        String seedStr = Args.getArgumentValue(args, "-seed", true);
    	params.random = new Random(Integer.parseInt(seedStr));
    	
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
						Thread.sleep(50);
					} catch (InterruptedException e) {}
				}
				printSummary(summaryPrefix, Status.TIMEOUT, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1);
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
        
	        case COBRA:
	            solveCOBRA(problem, params);
	            break;

	        case ORCA:
                solveORCA(problem, params);
                break;

            default:
                throw new RuntimeException("Unknown method");

        }
    }

	private static void solveCOBRA(final RelocationTaskCoordinationProblem problem, final Parameters params) {
        simulate(problem, new AgentFactory() {
            @Override
            public Agent createAgent(String name, int i, Point start, int nTasks,
                    Environment env, DirectedGraph<Point, Line> planningGraph, int agentBodyRadius, float speed) {

				PlanningAgent agent = new COBRAAgent(name, start, nTasks, env, planningGraph, agentBodyRadius, speed, params.maxTime, params.timeStep, params.random);
                return agent;
            }
        }, params);
    } 
	
	private static void solveORCA(final RelocationTaskCoordinationProblem problem, final Parameters params) {
        simulate(problem, new AgentFactory() {
            @Override
            public Agent createAgent(String name, int i, Point start, int nTasks,
                    Environment env, DirectedGraph<Point, Line> planningGraph, int agentBodyRadius, float speed) {

				Agent agent = new ORCAAgent(name, start, nTasks, env, planningGraph, agentBodyRadius, speed, params.maxTime, params.timeStep, params.showVis, params.random);
				return agent;
            }
        }, params);
    }    

    interface AgentFactory {
		Agent createAgent(String name, int priority, Point start,
				int nTasks, Environment env,
				DirectedGraph<Point, Line> planningGraph, int agentBodyRadius,
				float speed);
    }

    private static void simulate(final RelocationTaskCoordinationProblem problem, final AgentFactory agentFactory, final Parameters params) {
    	
    	simulationStartedAt = System.currentTimeMillis();
    	CurrentTasks.docks =  Arrays.asList(problem.getDocks());
    	
        // Create agents
        final List<Agent> agents = new LinkedList<Agent>();
        for (int i=0; i<problem.getStarts().length; i++) {
            agents.add(agentFactory.createAgent(            		
                    "a" + new DecimalFormat("00").format(i),
                    i,
                    problem.getStart(i),
                    params.nTasks,
                    problem.getEnvironment(),
                    problem.getPlanningGraph(),
                    problem.getBodyRadius(i),
                    problem.getMaxSpeed(i)));
        }
        
        // Simulation Control Layer 
        VisManager.registerLayer(SimulationControlLayer.create(new SimulationControlProvider() {
 			
 			@Override
 			public void setSpeed(float f) {}
 			
 			@Override
 			public void setRunning(boolean running) {}
 			
 			@Override
 			public boolean isRunning() { return true; }
 			
 			@Override
 			public double getTime() {
 				return (CommonTime.currentTimeMs() / 1000.0);
 			}
 			
 			@Override
 			public float getSpeed() { return 1; }
 		}));
        
        initAgentVisualization(agents, params.timeStep);

        LOGGER.info("=== All Agents initialized");
        final int FIRST_TASK_WINDOW = 30000;
        final long initalizedAtMs = CommonTime.currentTimeMs();
        
        for (final Agent agent : agents) {
        	
			Thread thread = new Thread(agent.getName()) {
				
				@Override
				public void run() {
					agent.setIssueFirstTaskAt(initalizedAtMs + params.random.nextInt(FIRST_TASK_WINDOW));
					agent.start();
					while (!allDone(agents)) {
						try {
							Thread.sleep(100);
						} catch (InterruptedException e) {}
						
						agent.tick(CommonTime.currentTimeMs());
					}
					LOGGER.info("Agent " + agent.getName() + " completed all tasks");
				}
			};
			
			thread.start();
		}
        
        while (!allDone(agents)) {
        	try {
				Thread.sleep(100);
			} catch (InterruptedException e) {}
        }
        
        
    	long baseSum = 0;
    	long baseSumSq = 0;
    	long waitSum = 0;
    	long waitSumSq = 0;	
    	long planSum = 0;
    	long planSumSq = 0;
    	long pWindowSum = 0;
    	long pWindowSumSq = 0;	
    	long prolongTSum = 0;
    	long prolongTSumSq = 0;	
    	long prolongRSum = 0;
    	long prolongRSumSq = 0;
        
        for (Agent agent : agents) {
        	baseSum +=  agent.baseSum;
        	baseSumSq += agent.baseSumSq ;
        	
        	waitSum += agent.waitSum;
        	waitSumSq += agent.waitSumSq;	
        	
        	planSum += agent.planSum;
        	planSumSq += agent.planSumSq;
        	
        	pWindowSum += agent.pWindowSum;
        	pWindowSumSq += agent.pWindowSumSq;	
        	
        	prolongTSum += agent.prolongTSum;
        	prolongTSumSq += agent.prolongTSumSq;	
        	
        	prolongRSum += agent.prolongRSum;
        	prolongRSumSq += agent.prolongRSumSq;
		}
        
        long n = agents.size() * params.nTasks;
        
        long avgBase = avg(baseSum, n);
        long varBase = sd(baseSumSq, avgBase, n);
        
        long avgWait = avg(waitSum, n);
        long varWait = sd(waitSumSq, avgWait, n);
        
        long avgPlan = avg(planSum, n);
        long varPlan = sd(planSumSq, avgPlan, n);
        
        long avgPWindow = avg(pWindowSum, n);
        long varPWindow = sd(pWindowSumSq,  avgPWindow, n);
        
        long avgProlongT = avg(prolongTSum, n);
        long varProlongT = sd(prolongTSumSq, avgProlongT, n);
        
        long avgProlongR = avg(prolongRSum, n);
        long varProlongR = sd(prolongRSumSq, avgProlongR, n);
        // avgBase;varBase;avgWait;varWait;avgPlan;varPlan;avgPWindow;varPWindow;avgProlongT;varProlongT;avgProlongR;varProlongR;makespan

		printSummary(params.summaryPrefix, Status.SUCCESS, 
				avgBase,varBase,
				avgWait,varWait,
				avgPlan,varPlan,
				avgPWindow,varPWindow,
				avgProlongT,varProlongT,
				avgProlongR,varProlongR,
				CommonTime.currentTimeMs() - initalizedAtMs);
    }
    
    private static boolean allDone(List<Agent> agents) {
    	for (final Agent agent : agents) {
    		if (!agent.hasCompletedAllTasks()) 
    			return false;
    	}
    	return true;
	}


	private static void initAgentVisualization(final List<Agent> agents, int timeStep) {
        // trajectories
        VisManager.registerLayer(
    		KeyToggleLayer.create("t", true, 
		        FastTrajectoriesLayer.create(new TrajectoriesProvider() {
					
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
				}, 3, timeStep)));
		
        // positions
        VisManager.registerLayer(
        	KeyToggleLayer.create("b", true, 
        	LabeledCircleLayer.create(new LabeledCircleLayer.LabeledCircleProvider<tt.euclid2i.Point>() {

             @Override
             public Collection<LabeledCircleLayer.LabeledCircle<tt.euclid2i.Point>> getLabeledCircles() {
                 LinkedList<LabeledCircleLayer.LabeledCircle<tt.euclid2i.Point>> list = new LinkedList<LabeledCircleLayer.LabeledCircle<tt.euclid2i.Point>>();
                 for (int i = 0; i < agents.size(); i++) {
               		 Point pos = agents.get(i).getCurrentPos();
                	 list.add(new LabeledCircleLayer.LabeledCircle<tt.euclid2i.Point>(pos, agents.get(i).getAgentBodyRadius(), "" + i  , AgentColors.getColorForAgent(i), AgentColors.getColorForAgent(i), Color.WHITE));
                 }

                 return list;
             }

         }, new tt.euclid2i.vis.ProjectionTo2d())));
        
        // tasks
        VisManager.registerLayer(LabeledCircleLayer.create(new LabeledCircleLayer.LabeledCircleProvider<tt.euclid2i.Point>() {

             @Override
             public Collection<LabeledCircleLayer.LabeledCircle<tt.euclid2i.Point>> getLabeledCircles() {
            	 
                 LinkedList<LabeledCircleLayer.LabeledCircle<tt.euclid2i.Point>> list = new LinkedList<LabeledCircleLayer.LabeledCircle<tt.euclid2i.Point>>();
                 for (Map.Entry<String, Point> entry : CurrentTasks.getTasks().entrySet()) {
               		 list.add(new LabeledCircleLayer.LabeledCircle<tt.euclid2i.Point>(entry.getValue(), 27, "\n"+entry.getKey() , Color.GREEN));
                 }

                 return list;
             }

         }, new tt.euclid2i.vis.ProjectionTo2d()));
            	
 	}

    enum Status {SUCCESS, FAIL, TIMEOUT}

	private static void printSummary(String prefix, Status status, 
	        long avgBase, long varBase, 
	        long avgWait, long varWait, 
	        long avgPlan, long varPlan, 
	        long avgPWindow, long varPWindow, 
	        long avgProlongT, long varProlongT, 
	        long avgProlongR,  long varProlongR,
	        long makespan
			) {
	    	System.out.println(prefix + status.toString() 
	    	         + ";" + avgBase  + ";" + varBase 
	    	         + ";" + avgWait  + ";" + varWait 
	    	         + ";" + avgPlan  + ";" + varPlan 
	    	         + ";" + avgPWindow  + ";" + varPWindow 
	    	         + ";" + avgProlongT  + ";" + varProlongT 
	    	         + ";" + avgProlongR   + ";" + varProlongR
	    	         + ";" + makespan
	    			);
	    	System.exit(0);
    }
	
	static long avg(long sum, long n) {
		return sum / n;
	}
	
	static long sd(long sumSq, long mean, long n) {
		double var = (sumSq/n) - (mean*mean); 
		return (long) Math.round(Math.sqrt(var));
	}

}

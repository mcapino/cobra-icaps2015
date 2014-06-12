package cz.agents.admap.agent;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;

import org.apache.log4j.Logger;
import org.jgrapht.DirectedGraph;
import org.jgrapht.alg.GraphBasedController;

import rvolib.KdTree;
import rvolib.RVOAgent;
import rvolib.RVOObstacle;
import rvolib.RVOUtil;
import rvolib.Vector2;
import tt.euclid2d.Vector;
import tt.euclid2i.EvaluatedTrajectory;
import tt.euclid2i.Line;
import tt.euclid2i.Point;
import tt.euclid2i.Region;
import tt.euclid2i.probleminstance.Environment;
import tt.euclid2i.region.Polygon;
import tt.euclid2i.trajectory.TimePointArrayTrajectory;
import tt.euclid2i.util.Util;
import util.DesiredControl;
import cz.agents.admap.msg.InformNewPosition;
import cz.agents.alite.communication.Message;

public class ORCAAgent extends Agent {

	static final Logger LOGGER = Logger.getLogger(ORCAAgent.class);

	private static final int MAX_NEIGHBORS = 50;
	private static final float MAX_SPEED = 1f;
	private static final float NEIGHBOR_DIST = 200;
	private static final float TIME_HORIZON_AGENT = 100;
	private static final float TIME_HORIZON_OBSTACLE = 10;

	private static final double NEAR_GOAL_EPS = 1.0f;

	private RVOAgent rvoAgent;
    private HashMap<String, RVOAgent> neighbors;

	private KdTree kdTree;
	private ArrayList<RVOObstacle> obstacles;

	DesiredControl desiredControl;

	private static final long UNKNOWN = -1;
	private static final double DESIRED_CONTROL_NODE_SEARCH_RADIUS = 250.0;
	private long lastTickTime = UNKNOWN;

	private boolean showVis;

	private int simulationSpeedMultiplier;
	final static int RADIUS_GRACE = +1;
	
	final static int TIME_MULTIPLIER = 10;

	private static final int SIMULATION_SPEED_MULTIPLIER = 10;

    public ORCAAgent(String name, Point start, Point goal, Environment environment, DirectedGraph<Point, Line> planningGraph, int agentBodyRadius, boolean showVis) {
        super(name, start, goal, environment, agentBodyRadius);

        this.showVis = showVis;
        this.simulationSpeedMultiplier = SIMULATION_SPEED_MULTIPLIER;

        rvoAgent = new RVOAgent();

        rvoAgent.position_ = new Vector2(start);
        rvoAgent.velocity_ = new Vector2(0,0);

        rvoAgent.maxNeighbors_ = MAX_NEIGHBORS;
        rvoAgent.maxSpeed_ = MAX_SPEED * simulationSpeedMultiplier;
        rvoAgent.neighborDist_ = NEIGHBOR_DIST;
        rvoAgent.radius_ = agentBodyRadius + RADIUS_GRACE;
        rvoAgent.timeHorizon_ = TIME_HORIZON_AGENT;
        rvoAgent.timeHorizonObst_ = TIME_HORIZON_OBSTACLE;

        rvoAgent.clearTrajectory();

        rvoAgent.id_ = Integer.parseInt(name.substring(1));

        rvoAgent.showVis = showVis;
        if (showVis) {
        	rvoAgent.initVisualization();
        }

        
        LinkedList<tt.euclid2i.Region> ttObstacles = new LinkedList<tt.euclid2i.Region>();
        ttObstacles.add(environment.getBoundary());
        ttObstacles.addAll(environment.getObstacles());
        
        Collection<Region> ttObstaclesInflated = Util.inflateRegions(ttObstacles, agentBodyRadius);

        desiredControl = new GraphBasedController(planningGraph, goal, ttObstaclesInflated, 
        		MAX_SPEED * simulationSpeedMultiplier, DESIRED_CONTROL_NODE_SEARCH_RADIUS, false);

        kdTree = new KdTree();

        // Add obstacles
        obstacles = new ArrayList<RVOObstacle>();

 		for (tt.euclid2i.Region region : ttObstacles) {
 			if (!(region instanceof Polygon)) {
 				throw new RuntimeException("Only polygons are supported");
 			}
 			ArrayList<Vector2> obstacle = RVOUtil.regionToVectorList((Polygon) region);
 			RVOUtil.addObstacle(obstacle, obstacles);
 		}

 		kdTree.buildObstacleTree(obstacles.toArray(new RVOObstacle[0]), this.obstacles);

 		neighbors = new HashMap<String, RVOAgent>();
 		neighbors.put(getName(), rvoAgent);
    }

    public synchronized Point getGoal() {
        return goal;
    }

    @Override
	public EvaluatedTrajectory getCurrentTrajectory() {
		return getEvaluatedTrajectory(goal);
	}

	public EvaluatedTrajectory getEvaluatedTrajectory(Point goal) {
		ArrayList<tt.euclidtime3i.Point> rvoTraj = new ArrayList<tt.euclidtime3i.Point>(rvoAgent.timePointTrajectory);
    	tt.euclidtime3i.Point[] timePointArray = new tt.euclidtime3i.Point[rvoTraj.size()];
    	for (int i=0; i<rvoTraj.size(); i++) {
    		timePointArray[i] = new tt.euclidtime3i.Point(
    				rvoTraj.get(i).getPosition(),
    				(int) Math.round(rvoTraj.get(i).getTime() * (double) simulationSpeedMultiplier));
    	}
    	
		TimePointArrayTrajectory traj = new TimePointArrayTrajectory(timePointArray, RVOAgent.evaluateCost(timePointArray, goal));
    	return traj;
    }
    
    @Override
    public void start() {

    }


    @Override
	public void tick(long time) {
		super.tick(time);

        if (showVis) {
//	        try {
//	            Thread.sleep(5);
//	        } catch (InterruptedException e) {}
        }
        
		if (lastTickTime == UNKNOWN) {
			lastTickTime = time;
			return;
		}

		float timeStep = (float) ((time - lastTickTime)/1e9f);
		lastTickTime = time;

		doStep(timeStep);
	}

    private void doStep(float timeStep) {
		setPreferredVelocity(timeStep);

		RVOAgent[] rvoAgents = neighbors.values().toArray(new RVOAgent[neighbors.values().size()]);

		LOGGER.debug(getName() + " neighbors: " + neighbors);

		kdTree.clearAgents();
        kdTree.buildAgentTree(rvoAgents);

        rvoAgent.computeNeighbors(kdTree);
        Vector2 newVelocity = rvoAgent.computeNewVelocity(timeStep);
        rvoAgent.update(timeStep, newVelocity);

        // broadcast to the others
        broadcast(new InformNewPosition(getName(), rvoAgent.id_, rvoAgent.position_.toPoint2d(), rvoAgent.velocity_.toVector2d(), (double) rvoAgent.radius_));
    }

	private void setPreferredVelocity(float timeStep) {
		Vector2 currentPosition = rvoAgent.position_;
		double distanceToGoal = currentPosition.toPoint2i().distance(goal);

		if (currentPosition.toPoint2i().distance(goal) < NEAR_GOAL_EPS) {
			rvoAgent.setPrefVelocity(new Vector2(0, 0));
		} else {
			Vector desiredVelocity = desiredControl.getDesiredControl(rvoAgent.position_.toPoint2d());
			double desiredSpeed = desiredVelocity.length();
			Vector desiredDir = new Vector(desiredVelocity);
			desiredDir.normalize();

			// Adjust if the agent is near the goal
			if (distanceToGoal <= timeStep * desiredSpeed) {
				// goal will be reached in the next time step
				double speed = distanceToGoal / timeStep;
				desiredVelocity = desiredDir;
				desiredVelocity.scale(speed);
			}
			
			rvoAgent.setPrefVelocity(new Vector2(desiredVelocity));
		}
	}

	@Override
    protected void notify(Message message) {
        super.notify(message);

        if (message.getContent() instanceof InformNewPosition) {
            InformNewPosition newPosMessage = (InformNewPosition) (message.getContent());
            RVOAgent neighborRVOAgent = new RVOAgent();
            neighborRVOAgent.id_ = newPosMessage.getAgentId();
            neighborRVOAgent.position_ = new Vector2(newPosMessage.getPosition());
            neighborRVOAgent.velocity_ = new Vector2(newPosMessage.getVelocity());
            neighborRVOAgent.radius_ = (float) newPosMessage.getRadius();

            if (!newPosMessage.getAgentName().equals(getName())) {
            	neighbors.put(newPosMessage.getAgentName(), neighborRVOAgent);
            }
        }
    }

	public Point getCurrentPosition() {
		return rvoAgent.position_.toPoint2i();
	}

	public Vector getCurrentVelocity() {
		return rvoAgent.velocity_.toVector2d();
	}

	@Override
	public boolean isGlobalTerminationDetected() {
		return getCurrentPosition().distance(goal) < 1;
	}



}

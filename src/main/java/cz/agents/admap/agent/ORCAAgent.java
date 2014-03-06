package cz.agents.admap.agent;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;

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
import tt.euclid2i.Trajectory;
import tt.euclid2i.probleminstance.Environment;
import tt.euclid2i.region.Polygon;
import tt.euclidtime3i.Region;
import tt.euclidtime3i.region.MovingCircle;
import tt.euclidtime3i.util.IntersectionChecker;
import util.DesiredControl;
import cz.agents.admap.msg.InformNewPosition;
import cz.agents.admap.msg.InformNewTrajectory;
import cz.agents.alite.communication.Message;

public class ORCAAgent extends Agent {

	static final Logger LOGGER = Logger.getLogger(ORCAAgent.class);

	private static final int MAX_NEIGHBORS = 50;
	private static final float MAX_SPEED = 1;
	private static final float NEIGHBOR_DIST = 100;
	private static final float TIME_HORIZON_AGENT = 10;
	private static final float TIME_HORIZON_OBSTACLE = 2;

	private static final double NEAR_GOAL_EPS = 0.5f;

	private RVOAgent rvoAgent;

	private HashMap<String, RVOAgent> neighbors;

	private KdTree kdTree;
	private ArrayList<RVOObstacle> obstacles;

	DesiredControl desiredControl;

	private static final long UNKNOWN = -1;
	private static final double DESIRED_CONTROL_NODE_SEARCH_RADIUS = 80.0;
	private long lastTickTime = UNKNOWN;


	static private int agentCounter = 0;

    public ORCAAgent(String name, Point start, Point goal, Environment environment, DirectedGraph<Point, Line> planningGraph, int agentBodyRadius) {
        super(name, start, goal, environment, agentBodyRadius);

        rvoAgent = new RVOAgent();

        rvoAgent.position_ = new Vector2(start);
        rvoAgent.velocity_ = new Vector2(0,0);

        rvoAgent.maxNeighbors_ = MAX_NEIGHBORS;
        rvoAgent.maxSpeed_ = MAX_SPEED;
        rvoAgent.neighborDist_ = NEIGHBOR_DIST;
        rvoAgent.radius_ = agentBodyRadius;
        rvoAgent.timeHorizon_ = TIME_HORIZON_AGENT;
        rvoAgent.timeHorizonObst_ = TIME_HORIZON_OBSTACLE;

        rvoAgent.id_ = agentCounter++;

        rvoAgent.initVisualization();

        LinkedList<tt.euclid2i.Region> ttObstacles = new LinkedList<tt.euclid2i.Region>();
        ttObstacles.addAll(environment.getObstacles());
        ttObstacles.add(environment.getBoundary());

        desiredControl = new GraphBasedController(planningGraph, goal, ttObstacles, MAX_SPEED, DESIRED_CONTROL_NODE_SEARCH_RADIUS, false);

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

    @Override
    public EvaluatedTrajectory getCurrentTrajectory() {
        return trajectory;
    }

    @Override
    public void start() {
    	if (getPlanningGraph() != null) {
    		trajectory = Util.computeBestResponse(start, goal, getPlanningGraph(), new LinkedList<Region>());
    	} else {
    		trajectory = Util.computeBestResponse(start, goal, inflatedObstacles, environment.getBoundary().getBoundingBox(), new LinkedList<Region>());
    	}
    	setCurrentPosition(trajectory.get(0));
    }


    @Override
	public void tick(long time) {
		super.tick(time);

		if (lastTickTime == UNKNOWN) {
			lastTickTime = time;
			return;
		}

		float timeStep = (float) ((time - lastTickTime)/1000000000.0);

		doStep(timeStep);
	}

    private void doStep(float timeStep) {
		setPreferredVelocity(timeStep);

		RVOAgent[] rvoAgents = neighbors.values().toArray(new RVOAgent[neighbors.values().size()]);

		LOGGER.debug(getName() + " neighbors: " + rvoAgents.length);

		// TODO Add other agents to the KD-tree
        kdTree.buildAgentTree(rvoAgents);

        rvoAgent.computeNeighbors(kdTree);
        rvoAgent.computeNewVelocity(timeStep);
        rvoAgent.update(timeStep);

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

			System.out.println("Setting pref. velocity for agent " + getName() + " to " + desiredVelocity);

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
            neighborRVOAgent.position_ = new Vector2(newPosMessage.getPosition());
            neighborRVOAgent.velocity_ = new Vector2(newPosMessage.getVelocity());
            neighborRVOAgent.radius_ = (float) newPosMessage.getRadius();
            neighbors.put(newPosMessage.getAgentName(), neighborRVOAgent);
        }
    }

	public Point getCurrentPosition() {
		return rvoAgent.position_.toPoint2i();
	}

	public Vector getCurrentVelocity() {
		return rvoAgent.velocity_.toVector2d();
	}



}

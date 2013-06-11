package cz.agents.admap.agent;

import java.util.Collection;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;

import org.apache.log4j.Logger;
import org.jgrapht.DirectedGraph;
import org.jgrapht.GraphPath;
import org.jgrapht.alg.AStarShortestPath;
import org.jgrapht.util.Goal;
import org.jgrapht.util.Heuristic;

import tt.euclid2i.EvaluatedTrajectory;
import tt.euclid2i.Point;
import tt.euclid2i.Region;
import tt.euclid2i.Trajectory;
import tt.euclid2i.discretization.LazyGrid;
import tt.euclid2i.discretization.ToGoalEdgeExtension;
import tt.euclid2i.probleminstance.Environment;
import tt.euclid2i.region.Rectangle;
import tt.euclid2i.trajectory.StraightSegmentTrajectory;
import tt.euclidtime3i.discretization.ConstantSpeedTimeExtension;
import tt.euclidtime3i.discretization.Straight;
import tt.euclidtime3i.region.MovingCircle;
import cz.agents.admap.msg.InformNewTrajectory;
import cz.agents.alite.communication.Message;

public class DSAAgent extends Agent {

    Logger LOGGER = Logger.getLogger(DSAAgent.class);

    private final double activationProbability;

    private static final int GRID_STEP = 10;
    private static final int MAX_TIME = 5000;

    Map<String, Objectives> group = new HashMap<String, Objectives>();
    Map<String, Trajectory> trajectories =  new HashMap<String, Trajectory>();
    Map<String, MovingCircle> avoids =  new HashMap<String, MovingCircle>();

    public DSAAgent(String name, Point start, Point goal, Environment environment, int agentSizeRadius, double activationProbability) {
        super(name, start, goal, environment, agentSizeRadius);
        this.group.put(name, new Objectives(start, goal));
        this.environment = environment;
        this.activationProbability = activationProbability;
    }

    @Override
    public EvaluatedTrajectory getCurrentTrajectory() {
        return trajectory;
    }

    @Override
    public void start() {
        replan();
    }

    private void replan() {


        // compute best response

        Collection<tt.euclidtime3i.Region> avoidRegions = new LinkedList<tt.euclidtime3i.Region>();
        for (MovingCircle movingCircle : avoids.values()) {
            avoidRegions.add(new MovingCircle(movingCircle.getTrajectory(), movingCircle.getRadius() + agentSizeRadius));
        }

        trajectory = computeBestResponse(start, goal, environment.getObstacles(), environment.getBounds(), avoidRegions);

        // broadcast to the others

        broadcast(new InformNewTrajectory(getName(), new MovingCircle(getCurrentTrajectory(), agentSizeRadius)));
    }

    private EvaluatedTrajectory computeBestResponse(final Point start, final Point goal,
            Collection<Region> obstacles, Rectangle bounds, Collection<tt.euclidtime3i.Region> avoid) {

        // create grid discretization
        final DirectedGraph<tt.euclid2i.Point, tt.euclid2i.Line> grid
            = new LazyGrid(start,
                    obstacles,
                    bounds,
                    LazyGrid.PATTERN_8_WAY,
                    GRID_STEP);

        final DirectedGraph<tt.euclid2i.Point, tt.euclid2i.Line> spatialGraph
            = new ToGoalEdgeExtension(grid, goal, GRID_STEP);

        // visualize the graph
//        VisManager.registerLayer(GraphLayer.create(new GraphProvider<tt.euclid2i.Point, tt.euclid2i.Line>() {
//            @Override
//            public Graph<tt.euclid2i.Point, tt.euclid2i.Line> getGraph() {
//                return ((ToGoalEdgeExtension) spatialGraph).generateFullGraph(start);
//            }
//        }, new tt.euclid2i.vis.ProjectionTo2d(), Color.GRAY, Color.GRAY, 1, 4));

        // time-extension
        DirectedGraph<tt.euclidtime3i.Point, Straight> graph
            = new ConstantSpeedTimeExtension(spatialGraph, MAX_TIME, new int[] {1}, avoid);


        // plan
        final GraphPath<tt.euclidtime3i.Point, Straight> path = AStarShortestPath
                .findPathBetween(graph,
                new Heuristic<tt.euclidtime3i.Point>() {
                    @Override
                    public double getCostToGoalEstimate(tt.euclidtime3i.Point current) {
                        return (current.getPosition()).distance(goal);
                    }
                },
                new tt.euclidtime3i.Point(start.x, start.y, 0),
                new Goal<tt.euclidtime3i.Point>() {
                    @Override
                    public boolean isGoal(tt.euclidtime3i.Point current) {
                        return current.getPosition().equals(goal);
                    }
                });

        EvaluatedTrajectory trajectory
            = new StraightSegmentTrajectory<tt.euclidtime3i.Point, Straight>(path, path.getEndVertex().getTime());

        return trajectory;
    }

    @Override
    protected void notify(Message message) {
        super.notify(message);
        if (message.getContent() instanceof InformNewTrajectory) {
            InformNewTrajectory newTrajectoryMessage = (InformNewTrajectory) (message.getContent());
            String agentName = newTrajectoryMessage.getAgentName();
            MovingCircle occupiedRegion = (MovingCircle) newTrajectoryMessage.getRegion();

            if (agentName.compareTo(getName()) < 0) {
                avoids.put(agentName, occupiedRegion);
            }
        }
    }

    @Override
    public void tick(long time) {
        super.tick(time);
        if (Math.random() < activationProbability) {
            LOGGER.debug(name + " was activated and replans");
            replan();
        }
    }

}

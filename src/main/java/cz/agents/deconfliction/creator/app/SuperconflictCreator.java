package cz.agents.deconfliction.creator.app;

import java.text.DecimalFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.Random;

import cz.agents.deconfliction.agent.Agent;
import cz.agents.deconfliction.agent.BlockingAsynchronousPrioritizedPlanningAgent;
import cz.agents.deconfliction.agent.BRABTAgent;
import cz.agents.deconfliction.agent.CommunicatingAgent;
import cz.agents.deconfliction.agent.DCSPAgent;
import cz.agents.deconfliction.agent.SynchronousPrioritizedPlanningAgent;
import cz.agents.deconfliction.agent.DecentralizedPrioritizedPlanningAgent.ReplanningStrategy;
import cz.agents.deconfliction.configuration.Parameters;
import cz.agents.deconfliction.creator.DCSPCreator;
import cz.agents.deconfliction.maneuvergraph.EightWayConstantSpeedGridGraph;
import cz.agents.deconfliction.maneuvergraph.EightWayPieWaypointGraph;
import cz.agents.deconfliction.maneuvergraph.FourWayConstantSpeedGridGraph;
import cz.agents.deconfliction.maneuvergraph.FourWayPieWaypointGraph;
import org.jgrapht.Graph;
import cz.agents.deconfliction.trajectory.TrajectoryGenerator;
import cz.agents.deconfliction.util.KShortestPathsGenerator;
import cz.agents.deconfliction.util.KShortestPathsTrajectoryGenerator;
import cz.agents.alite.trajectorytools.util.OrientedPoint;
import cz.agents.deconfliction.util.Point;
import cz.agents.alite.trajectorytools.util.Waypoint;
import cz.agents.deconfliction.waypointgraph.DefaultWaypointGraph;

import org.apache.log4j.Level;
import org.apache.log4j.Logger;
import org.jgrapht.alg.KShortestPaths;
import org.jgrapht.graph.DefaultWeightedEdge;


public class SuperconflictCreator extends DCSPCreator {

    //static final double NMI = 1852; // One nautical mile is 1852 meters
    // To avoid problems with alite... making nmi smaller
    static final double NMI = 10;
    static final double KN = NMI/3600.0; // One knot is 0.514 m/s

    final int NO_OF_AIRPLANES = 18;
    final double CIRCLE_RADIUS = 50 * NMI;
    final Point CENTER = new Point(50 * NMI,50 * NMI,0);
    final double ANGLE_BETWEEN_AIRPLANES = (2 * Math.PI ) / (double) NO_OF_AIRPLANES;
    final int LEVELS = 4;

    @Override
    protected Parameters configureParameters(Parameters params) {
        params.GRID_X = 10;
        params.GRID_Y = 10;
        params.SEPARATION = 5.0 * NMI;
        params.MAX_X = 100 * (int) NMI;
        params.MAX_Y = 100 * (int) NMI;
        params.SPEED = 500 * KN;
        params.APPROXIMATION_SAMPLING_INTERVAL = params.SEPARATION / 4.0;
        params.SIMULATION_SPEED_RATIO = 0.01;
        params.VISUALISE_SEARCH = false;
        params.VISUALIZE_NEW_ASSIGNMENT = false;
        params.REPLANNING_STRATEGY = ReplanningStrategy.IF_INCONSISTENT;
        return params;
    }

    @Override
    protected List<Agent> createAgents() {
        List<Agent> agents = new LinkedList<Agent>();

        for (int i=0; i < NO_OF_AIRPLANES; i++) {
            double angle = i*ANGLE_BETWEEN_AIRPLANES;
            OrientedPoint start = new OrientedPoint(CENTER.x + Math.cos(angle)*CIRCLE_RADIUS, CENTER.y + Math.sin(angle)*CIRCLE_RADIUS,0,0,1,0);
            OrientedPoint end = new OrientedPoint(CENTER.x + Math.cos(angle+Math.PI)*CIRCLE_RADIUS, CENTER.y + Math.sin(angle+Math.PI)*CIRCLE_RADIUS,0,0,1,0);
            agents.add(new BlockingAsynchronousPrioritizedPlanningAgent("A"
                    + new DecimalFormat("00").format(i), getManeuvers(), 0.0,
                    start, end, getParams().SEPARATION,
                    getParams().MAX_T,
                    getParams().APPROXIMATION_SAMPLING_INTERVAL,
                    getParams().REPLANNING_STRATEGY));
        }

        return agents;
    }

    String getRandomNum(int max) {
        return Integer.toString(new Random().nextInt(max));
    }

    @Override
    protected Graph<Waypoint, SpatialManeuver> generateGraph<Waypoint, SpatialManeuver>() {
        //return new EightWayPieWaypointGraph(CENTER, CIRCLE_RADIUS, NO_OF_AIRPLANES, LEVELS);
        //return new FourWayConstantSpeedGridGraph(getParams().MAX_X, getParams().MAX_Y, getParams().GRID_X, getParams().GRID_Y, getParams().SPEED);
        return new EightWayConstantSpeedGridGraph(getParams().MAX_X, getParams().MAX_Y, getParams().GRID_X, getParams().GRID_Y, getParams().SPEED);
    }

}

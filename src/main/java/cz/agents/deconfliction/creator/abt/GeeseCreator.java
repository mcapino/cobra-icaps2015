package cz.agents.deconfliction.creator.abt;

import java.text.DecimalFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.Random;

import cz.agents.deconfliction.agent.Agent;
import cz.agents.deconfliction.agent.CommunicatingAgent;
import cz.agents.deconfliction.agent.DCSPAgent;
import cz.agents.deconfliction.configuration.Parameters;
import cz.agents.deconfliction.creator.DCSPCreator;
import cz.agents.deconfliction.trajectory.TrajectoryGenerator;
import cz.agents.deconfliction.util.KShortestPathsGenerator;
import cz.agents.deconfliction.util.KShortestPathsTrajectoryGenerator;
import cz.agents.alite.trajectorytools.util.OrientedPoint;
import cz.agents.alite.trajectorytools.util.Waypoint;

import org.apache.log4j.Level;
import org.apache.log4j.Logger;
import org.jgrapht.alg.KShortestPaths;
import org.jgrapht.graph.DefaultWeightedEdge;


public class GeeseCreator extends DCSPCreator {

    //static final double NMI = 1852; // One nautical mile is 1852 meters
    // To avoid problems with alite... making nmi smaller
    static final double NMI = 10;
    static final double KN = NMI/3600.0; // One knot is 0.514 m/s

    @Override
    protected Parameters configureParameters(Parameters params) {
        Logger.getRootLogger().setLevel(Level.INFO);
        params.GRID_X = 4;
        params.GRID_Y = 4;
        params.SEPARATION = 5.0 * NMI;
        params.MAX_X = 100 * (int) NMI;
        params.MAX_Y = 100 * (int) NMI;
        params.SPEED = 500 * KN;
        params.APPROXIMATION_SAMPLING_INTERVAL = 5;
        params.SIMULATION_SPEED_RATIO = 0.02;
        params.VISUALISE_SEARCH = false;
        params.VISUALIZE_NEW_ASSIGNMENT = false;
        params.CANDIDATES = 3000;
        return params;
    }

    @Override
    protected List<Agent> createAgents() {
        List<Agent> agents = new LinkedList<Agent>();

        final int AIRPLANES_IN_FLOW = 20;
        final double TIMEGAP_BETWEEN_AIRPLANES = 40.0;
        final int ADD_GAP_EACH = 100;

        final OrientedPoint vFlowStart = new OrientedPoint(getParams().MAX_X/2,0,0,0,1,0);
        final OrientedPoint vFlowEnd = new OrientedPoint(getParams().MAX_X/2,getParams().MAX_Y,0,0,1,0);
        final Waypoint vFlowStartWP = getManeuvers().getNearestWaypoint(vFlowStart);
        final Waypoint vFlowEndWP = getManeuvers().getNearestWaypoint(vFlowEnd);

        final OrientedPoint hFlowStart = new OrientedPoint(0, getParams().MAX_Y/2,0,0,1,0);
        final OrientedPoint hFlowEnd = new OrientedPoint(getParams().MAX_X, getParams().MAX_Y/2,0,0,1,0);
        final Waypoint hFlowStartWP = getManeuvers().getNearestWaypoint(hFlowStart);
        final Waypoint hFlowEndWP = getManeuvers().getNearestWaypoint(hFlowEnd);


        KShortestPathsGenerator vPathGenerator = new KShortestPathsGenerator(getManeuvers(), vFlowStartWP, vFlowEndWP, getParams().CANDIDATES);

        // Vertical flow
        for (int i=1; i <= AIRPLANES_IN_FLOW; i++) {
            if (i % ADD_GAP_EACH == 0) continue;
            double startTime = i*TIMEGAP_BETWEEN_AIRPLANES;
            TrajectoryGenerator trajectoryGenerator = new KShortestPathsTrajectoryGenerator(vPathGenerator, startTime, getParams().APPROXIMATION_SAMPLING_INTERVAL);
            agents.add(new DCSPAgent(new DecimalFormat("00").format(i) + "A", getManeuvers(), vFlowStart , vFlowEnd, startTime, trajectoryGenerator, getParams()));
        }

        KShortestPathsGenerator hPathGenerator = new KShortestPathsGenerator(getManeuvers(), hFlowStartWP, hFlowEndWP, getParams().CANDIDATES);

        // Horizontal flow
        for (int i=1; i <= AIRPLANES_IN_FLOW; i++) {
            if (i % ADD_GAP_EACH == 0) continue;
            double startTime = i*TIMEGAP_BETWEEN_AIRPLANES;
            TrajectoryGenerator trajectoryGenerator = new KShortestPathsTrajectoryGenerator(hPathGenerator, startTime, getParams().APPROXIMATION_SAMPLING_INTERVAL);
            agents.add(new DCSPAgent(new DecimalFormat("00").format(i) + "B", getManeuvers(), hFlowStart , hFlowEnd, startTime, trajectoryGenerator, getParams()));        }

        return agents;
    }

    String getRandomNum(int max) {
        return Integer.toString(new Random().nextInt(max));
    }
}

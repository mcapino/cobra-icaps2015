package cz.agents.map4rt.agent;

import java.util.LinkedList;
import java.util.Random;

import org.jgrapht.DirectedGraph;
import org.jgrapht.util.HeuristicToGoal;
import org.jgrapht.util.heuristics.PerfectHeuristic;

import tt.euclid2i.EvaluatedTrajectory;
import tt.euclid2i.Line;
import tt.euclid2i.Point;
import tt.euclid2i.probleminstance.Environment;
import tt.euclidtime3i.Region;
import tt.euclidtime3i.region.MovingCircle;
import cz.agents.map4rt.CommonTime;

public class COBRAAgent extends PlanningAgent {

	private int samplingInterval;

	public COBRAAgent(String name, Point start, int nTasks,
			Environment env, DirectedGraph<Point, Line> planningGraph, 
			int agentBodyRadius, final float maxSpeed, int maxTime, int timeStep, Random random) {
		super(name, start, nTasks, env, planningGraph, agentBodyRadius, maxSpeed, maxTime, timeStep, random);
		
		assert timeStep % 2 == 0;
		samplingInterval = timeStep/2;
		
		final HeuristicToGoal<tt.euclid2i.Point> spatialHeuristic = new PerfectHeuristic<tt.euclid2i.Point, Line>(planningGraph, start);
		final HeuristicToGoal<tt.euclidtime3i.Point> spaceTimeHeuristic = new HeuristicToGoal<tt.euclidtime3i.Point>() {
			@Override
			public double getCostToGoalEstimate(tt.euclidtime3i.Point current) {
				return spatialHeuristic.getCostToGoalEstimate(current.getPosition()) / (double) maxSpeed;
			}
		};
			
		EvaluatedTrajectory traj = BestResponse.computeBestResponse(start, 0, 0, start,
				maxSpeed, getPlanningGraph(), spaceTimeHeuristic,
				new LinkedList<Region>(), maxTime, timeStep, T_PLANNING);
		
		int samplingInterval = timeStep/2;
		Token.register(getName(), new MovingCircle(traj, agentBodyRadius, samplingInterval));
		currentTrajectory = traj;
	}

	@Override
	protected void handleNewTask(Point task) {
		
		long planningStartedAt = CommonTime.currentTimeMs();
		
		// nearest timestep in past
		int minTime = ((int) Math.floor( (double) CommonTime.currentTimeMs() / (double) timeStep) ) * timeStep;
		// start at a multiple of timestep
		int depTime = ((int) Math.ceil( (double) (CommonTime.currentTimeMs() + T_PLANNING) / (double) timeStep) ) * timeStep;
		
		EvaluatedTrajectory traj = getBestResponseTrajectory(
				getCurrentPos(), minTime, depTime, task,
				Token.getReservedRegions(getName()), maxTime);

		Token.register(getName(), new MovingCircle(traj, agentBodyRadius, samplingInterval));
		currentTrajectory = traj;
		lastTaskTravelStartedAt = depTime;
		currentTaskTouchedGoal = false;
		
		long planningDuration = (CommonTime.currentTimeMs() - planningStartedAt);
		planSum += planningDuration;
		planSumSq += planningDuration * planningDuration;
		
		long pWindowDuration = depTime - planningStartedAt;
		pWindowSum += pWindowDuration;
		pWindowSumSq += pWindowDuration * pWindowDuration;
	}

	@Override
	public void start() {}

}

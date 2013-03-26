package cz.agents.deconfliction.creator;

import java.awt.Color;
import java.io.File;
import java.io.FileInputStream;
import java.text.DecimalFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.Properties;

import javax.vecmath.Point2d;

import org.apache.log4j.Logger;
import org.apache.log4j.PropertyConfigurator;
import org.jgrapht.DirectedGraph;

import cz.agents.alite.creator.Creator;
import cz.agents.alite.simulation.ConcurrentProcessSimulation;
import cz.agents.alite.trajectorytools.graph.spatial.SpatialGridFactory;
import cz.agents.alite.trajectorytools.graph.spatial.maneuvers.SpatialManeuver;
import cz.agents.alite.trajectorytools.simulation.SimulatedAgentEnvironment;
import cz.agents.alite.trajectorytools.trajectory.Trajectory;
import cz.agents.alite.trajectorytools.util.TimePoint;
import cz.agents.alite.trajectorytools.util.Waypoint;
import cz.agents.alite.trajectorytools.vis.ConflictsLayer;
import cz.agents.alite.trajectorytools.vis.ConflictsLayer.ConflictsProvider;
import cz.agents.alite.trajectorytools.vis.GraphLayer;
import cz.agents.alite.trajectorytools.vis.GraphLayer.GraphProvider;
import cz.agents.alite.trajectorytools.vis.NodeIdLayer;
import cz.agents.alite.trajectorytools.vis.SimulatedCylindricAgentLayer;
import cz.agents.alite.trajectorytools.vis.SimulatedCylindricAgentLayer.TimeProvider;
import cz.agents.alite.trajectorytools.vis.SimulationControlLayer;
import cz.agents.alite.trajectorytools.vis.TrajectoriesLayer;
import cz.agents.alite.trajectorytools.vis.TrajectoriesLayer.TrajectoriesProvider;
import cz.agents.alite.trajectorytools.vis.projection.ProjectionTo2d;
import cz.agents.alite.vis.VisManager;
import cz.agents.alite.vis.VisManager.SceneParams;
import cz.agents.alite.vis.layer.common.ColorLayer;
import cz.agents.alite.vis.layer.common.VisInfoLayer;
import cz.agents.deconfliction.agent.Agent;
import cz.agents.deconfliction.configuration.Parameters;
import cz.agents.deconfliction.problem.DeconflictionProblem;
import cz.agents.deconfliction.vis.AgentMissionLayer;

public abstract class DeconflictionCreator implements Creator {

        protected static final Logger LOGGER = Logger.getLogger(DeconflictionCreator.class);
        private Parameters params = new Parameters();

        private DeconflictionProblem problem;
        private ConcurrentProcessSimulation concurrentSimulation;
        private SimulatedAgentEnvironment environment;

        protected double nominalGlobalCost;
        private DirectedGraph<Waypoint, SpatialManeuver> maneuvers;

        @Override
        public void init(String[] args) {
            // Set-up the logger
            String overrideLogFileName = null;
            if (args.length>1) overrideLogFileName  = args[1];

            Properties prop = new Properties();
            try {
                prop.load(new FileInputStream("resources" + File.separator + "log4j" + File.separator + "log4j.properties"));
            } catch (Exception ex){
                ex.printStackTrace();
            }
            if (overrideLogFileName != null) {
                prop.setProperty("log4j.appender.R.File", overrideLogFileName);
            }
            PropertyConfigurator.configure(prop);
        }

        protected Parameters configureParameters(Parameters params){ return params; }

        protected List<Agent> createAgents() {
            List<Agent> agents = new LinkedList<Agent>();
            return agents;
        }

        @Override
        public void create() {

            params = configureParameters(params);

            LOGGER.info(">>> CONFLICT RESOLUTION PROBLEM CREATED");
            problem = new DeconflictionProblem(params.SEPARATION);

            maneuvers = generateGraph();

            LOGGER.info(">>> PARAMETERS: SPEED: " + params.SPEED + "m/s SPACE:" + params.MAX_X + "mx" + params.MAX_Y + "m SEPARATION:" + params.SEPARATION + "m");
            LOGGER.info(">>>             Δt: " + params.APPROXIMATION_SAMPLING_INTERVAL + "s MAX_T:" + params.MAX_T + "s");

            createSimulation();
            LOGGER.info(">>> SIMULATION CREATED");

            setupAgents();
            LOGGER.info(">>> AGENTS CREATED. ");

            nominalGlobalCost = problem.getSumOfTrajectories();
            LOGGER.info("Nominal cost: " + new DecimalFormat("#.##").format(nominalGlobalCost));

            createEnvironment();
            LOGGER.info(">>> ENVIRONMENT CREATED. ");

            if (params.ENABLE_VISUALISATION) {
                createVisualization();
                LOGGER.info(">>> VISUALIZATION CREATED. ");
            }

            LOGGER.info(">>> STARTING RESOLUTION. ");
            startResolution();

            environment.updateTrajectories(problem.getAgentsTrajectories());

            LOGGER.info("Global cost:" + new DecimalFormat("#.##").format(problem.getSumOfTrajectories()) + " Quality:" + new DecimalFormat("#.##").format(getSolutionQuality(nominalGlobalCost)));

            //show3dView();
        }

        protected void setupAgents() {}

        protected DirectedGraph<Waypoint, SpatialManeuver> generateGraph() {
            return SpatialGridFactory.create4WayGridAsDirectedGraph(params.MAX_X, params.MAX_Y, params.GRID_X, params.GRID_Y, params.SPEED);
        }

        private double getSolutionQuality(double originalCost) {
            return (originalCost/problem.getSumOfTrajectories())*100.0;
        }

        private void createEnvironment() {
            environment = new SimulatedAgentEnvironment();
            environment.updateTrajectories(problem.getAgentsTrajectories());
        }

        private void createVisualization() {
            VisManager.setInitParam("Deconfliction", 1024, 768);
            VisManager.setSceneParam(new SceneParams(){

                @Override
                public Point2d getDefaultLookAt() {
                    return new Point2d(5, 5);
                }

                @Override
                public double getDefaultZoomFactor() {
                    return 40;
                }});
            VisManager.init();

            // background
            VisManager.registerLayer(ColorLayer.create(Color.WHITE));

            // static
            VisManager.registerLayer(GraphLayer.create(new GraphProvider<Waypoint, SpatialManeuver>() {

                @Override
                public DirectedGraph<Waypoint, SpatialManeuver> getGraph() {
                    return getManeuvers();
                }}, new Color(220, 220, 220), new Color(240, 240, 240), 1, 4));

            // dynamic
            //VisManager.registerLayer(AgentTrajectoryLayer.create(problem, params.SPEED/10, "t"));
            VisManager.registerLayer(TrajectoriesLayer.create(new TrajectoriesProvider() {

                @Override
                public Trajectory[] getTrajectories() {
                    return problem.getTrajectories();
                }
            }, new ProjectionTo2d<TimePoint>() {

                @Override
                public Point2d project(TimePoint point) {
                    return new Point2d(point.x, point.y);
                }

            }, params.SPEED/10, getParams().MAX_T, 't'));


            // dynamic
            VisManager.registerLayer(AgentMissionLayer.create(problem, "m"));

            // dynamic
            //VisManager.registerLayer(AgentSeparationLayer.create(problem));

            // dynamic
            VisManager.registerLayer(ConflictsLayer.create(new ConflictsProvider() {

                @Override
                public List<TimePoint> getConflicts() {
                    return problem.getConflicts();
                }
            }, getParams().SEPARATION));

            VisManager.registerLayer(NodeIdLayer.create(getManeuvers(), Color.GRAY, 1, "n"));

            //VisManager.registerLayer(CoordinateAxesLayer.create(Color.BLACK, 2, "c"));

            VisManager.registerLayer(SimulatedCylindricAgentLayer.create(environment.getAgentStorage(), new ProjectionTo2d<TimePoint>() {

                @Override
                public Point2d project(TimePoint point) {
                    return new Point2d(point.x, point.y);
                }

            }, new TimeProvider() {

                @Override
                public double getTime() {
                    return environment.getTime();
                }
            }, getParams().SEPARATION, getParams().SEPARATION/2));

            //VisManager.registerLayer(BubbleLayer.create());
            // Overlay
            VisManager.registerLayer(VisInfoLayer.create());

            VisManager.registerLayer(SimulationControlLayer.create(environment));

        }

        private void createSimulation() {
            concurrentSimulation = new ConcurrentProcessSimulation();
            concurrentSimulation.setPrintouts(1000);
        }

        protected abstract void startResolution();

        protected DirectedGraph<Waypoint, SpatialManeuver> getManeuvers() {
            return maneuvers;
        }

        protected Parameters getParams() {
            return params;
        }

        public DeconflictionProblem getProblem() {
            return problem;
        }

        public ConcurrentProcessSimulation getConcurrentSimulation() {
            return concurrentSimulation;
        }

        /*
        protected void show3dView() {
            // create your PlotPanel (you can use it as a JPanel)
            Plot3DPanel plot = new Plot3DPanel();

            // define the legend position
            plot.addLegend("SOUTH");

            int agentNo = 0;
            for(Agent agent : problem.getAgents()) {

                TrajectoryApproximation trajectoryApprox = agent.getCurrentTrajectory().getApproximation();
                double[][] data = new double[trajectoryApprox.length()][3];

                int i = 0;
                for (OrientedPoint pos : trajectoryApprox) {
                    if (pos != null) {
                        data[i][1] = pos.x;
                        data[i][0] = pos.y;
                        data[i][2] = (trajectoryApprox.getFirstSampleNo() + i) * trajectoryApprox.getSamplingInterval();
                        i++;
                    }
                }
                plot.addScatterPlot(agent.getName(), AgentTrajectoryLayer.getColorForAgent(agentNo), data);
                agentNo++;
            }

            // add a line plot to the PlotPanel


            // put the PlotPanel in a JFrame like a JPanel
            JFrame frame = new JFrame("Spatio-temporal view of trajectories");
            frame.setSize(600, 600);
            frame.setContentPane(plot);
            frame.setVisible(true);
        }*/



}

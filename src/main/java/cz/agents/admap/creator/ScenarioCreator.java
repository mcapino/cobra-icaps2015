package cz.agents.admap.creator;
import java.awt.Color;
import java.io.File;
import java.io.FileInputStream;
import java.util.Collection;
import java.util.LinkedList;
import java.util.Properties;

import javax.vecmath.Point2d;

import org.apache.log4j.Logger;
import org.apache.log4j.PropertyConfigurator;

import tt.euclid2i.Point;
import tt.euclid2i.region.Rectangle;
import tt.euclid2i.region.Region;
import tt.euclid2i.vis.RegionsLayer;
import tt.euclid2i.vis.RegionsLayer.RegionsProvider;
import tt.jointeuclidean2ni.probleminstance.ShortestPathProblem;
import cz.agents.alite.creator.Creator;
import cz.agents.alite.trajectorytools.vis.LabeledPointLayer;
import cz.agents.alite.trajectorytools.vis.LabeledPointLayer.LabeledPoint;
import cz.agents.alite.trajectorytools.vis.LabeledPointLayer.LabeledPointsProvider;
import cz.agents.alite.vis.VisManager;
import cz.agents.alite.vis.VisManager.SceneParams;
import cz.agents.alite.vis.layer.common.ColorLayer;
import cz.agents.alite.vis.layer.common.VisInfoLayer;

public class ScenarioCreator implements Creator {

    ////////////////////////////////////////////////////////////////////////

    public static void main(String[] args) {
        ScenarioCreator creator = new ScenarioCreator();
        creator.init(args);
        creator.create("default", Method.ADMAP, 5, 961, 500, true);
    }

    ///////////////////////////////////////////////////////////////////////


    @Override
    public void create() {
        if (args.length > 1) {
            createFromArgs();
        }
    }




    static Logger LOGGER = Logger.getLogger(ScenarioCreator.class);

    private static final int ITERATION_DELAY = 50;
    private static final int AGENT_BODY_RADIUS = 10;


    enum Method { ADPP, ADMAP}

    private String[] args;
    private ShortestPathProblem problem;

    @SuppressWarnings("unused")
    @Override
    public void init(String[] args) {

        this.args = args;

        // Set-up the logger
        String overrideLogFileName = null;
        //if (args.length>1) overrideLogFileName  = args[1];

        Properties prop = new Properties();

        String propFile = null;
        if ((new File("log4j.properties")).isFile()) {
            propFile = "log4j.properties";
        } else {
            if ((new File("resources" + File.separator + "log4j" + File.separator + "log4j.properties")).isFile()) {
                propFile = "resources" + File.separator + "log4j" + File.separator + "log4j.properties";
            }

        }
        if (propFile != null) {
            try {
                prop.load(new FileInputStream(propFile));
            } catch (Exception ex){
                ex.printStackTrace();
            }
            if (overrideLogFileName != null) {
                prop.setProperty("log4j.appender.R.File", overrideLogFileName);
            }
            PropertyConfigurator.configure(prop);

        }

    }

    public void createFromArgs() {
        //for (String arg : args) { System.out.println(arg); }
        String experimentId = args[1];
        String algorithm = args[2];
        int nAgents = Integer.parseInt(args[3]);
        int seed = Integer.parseInt(args[4]);
        int expandedStatesLimit = Integer.parseInt(args[5]);
        boolean showVis = Boolean.parseBoolean(args[6]);

        create(experimentId, Method.valueOf(algorithm), nAgents, seed, expandedStatesLimit, showVis);
    }

    public void create(String experimentId, Method method, int nAgents, int seed, long expandedStatesLimit,  boolean showVis) {
        this.problem = new ShortestPathProblem(nAgents, AGENT_BODY_RADIUS, seed);

        if (showVis) {
            createVisualization();
        }

        if (method == Method.ADMAP) {
            solveADMAP(problem, expandedStatesLimit, showVis);
        }
    }

    private void solveADMAP(final ShortestPathProblem problem, final long expandedStatesLimit, boolean showVis) {

        Collection<Rectangle> polygons = new LinkedList<Rectangle>();
        for (Region obstacle : problem.getObstacles()) {
            assert obstacle instanceof Rectangle;
            polygons.add((Rectangle) obstacle);
        }
        
        


        /*
        reportResult(Method.ADMAP.toString(), problem, path != null,
                path != null ? path.getWeight() : Double.POSITIVE_INFINITY,
                astar.getExpandedStatesCount(),
                expandedStatesLimit);*/
    }

    /*
    private void reportResult(String alg, ShortestPathProblem problem, boolean foundSolution,
            double solutionQuality, long iterationsToFirstSolution, long iterationLimit) {
        System.out.println(alg + ";" + problem.getSeed() + ";" + Analyzer.getProblemInstanceCharacteristics(problem)
                + ";" + foundSolution + ";"
                + solutionQuality + ";" + iterationsToFirstSolution + ";" + iterationLimit);
    } */


    private void createVisualization() {
        VisManager.setInitParam("Trajectory Tools Vis", 1024, 768);
        VisManager.setSceneParam(new SceneParams() {

            @Override
            public Point2d getDefaultLookAt() {
                return new Point2d(500, 500);
            }

            @Override
            public double getDefaultZoomFactor() {
                return 0.5;
            }

        });
        VisManager.init();

        // background
        VisManager.registerLayer(ColorLayer.create(Color.WHITE));

        // KeyToggleLayer graphToggle = KeyToggleLayer.create("g");
        // graphToggle.addSubLayer(GraphLayer.create(new GraphProvider<Waypoint,
        // SpatialManeuver>() {
        //
        // @Override
        // public Graph<Waypoint, SpatialManeuver> getGraph() {
        // return graph;
        // }}, Color.GRAY, Color.GRAY, 1, 4));
        // graphToggle.setEnabled(true);
        //
        // VisManager.registerLayer(graphToggle);

        // VisManager.registerLayer(NodeIdLayer.create(graph, Color.GRAY, 1,
        // "n"));

        VisManager.registerLayer(RegionsLayer.create(
                new RegionsProvider() {

                    @Override
                    public Collection<Region> getRegions() {
                        LinkedList<Region> list = new LinkedList<Region>();
                        list.add(problem.getBounds());
                        return list;
                    }

                }, Color.BLACK, Color.WHITE));

        VisManager.registerLayer(RegionsLayer.create(
                new RegionsProvider() {

                    @Override
                    public Collection<Region> getRegions() {
                        return problem.getObstacles();
                    }

                }, Color.GRAY, Color.GRAY));

        VisManager.registerLayer(LabeledPointLayer.create(new LabeledPointsProvider<Point>() {

            @Override
            public Collection<LabeledPoint<Point>> getLabeledPoints() {
                LinkedList<LabeledPoint<Point>> list = new LinkedList<LabeledPoint<Point>>();

                for (int i=0; i < problem.getStarts().length; i++) {
                    list.add(new LabeledPoint<Point>(problem.getStarts()[i], "s"+i));
                }
                return list;
            }

        }, new tt.euclid2i.vis.PointProjectionTo2d(), Color.BLUE));

        VisManager.registerLayer(RegionsLayer.create(
                new RegionsProvider() {

                    @Override
                    public Collection<Region> getRegions() {
                        LinkedList<Region> list = new LinkedList<Region>();

                        for (int i=0; i < problem.getTargetRegions().length; i++) {
                            list.add(problem.getTargetRegions()[i]);
                        }

                        return list;
                    }

                }, Color.PINK, Color.PINK));

        VisManager.registerLayer(LabeledPointLayer.create(new LabeledPointsProvider<Point>() {

            @Override
            public Collection<LabeledPoint<Point>> getLabeledPoints() {
                LinkedList<LabeledPoint<Point>> list = new LinkedList<LabeledPoint<Point>>();

                for (int i=0; i < problem.getTargetRegions().length; i++) {
                    list.add(new LabeledPoint<Point>(problem.getTargetPoint(i), "g"+i));
                }
                return list;
            }

        }, new tt.euclid2i.vis.PointProjectionTo2d(), Color.RED));

        // Overlay
        VisManager.registerLayer(VisInfoLayer.create());
    }
}

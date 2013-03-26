package cz.agents.deconfliction.vis;

import java.awt.Color;
import java.util.ArrayList;

import javax.vecmath.Point3d;

import cz.agents.alite.trajectorytools.util.Vector;
import cz.agents.alite.vis.element.Circle;
import cz.agents.alite.vis.element.StyledLine;
import cz.agents.alite.vis.element.StyledPoint;
import cz.agents.alite.vis.element.aggregation.CircleElements;
import cz.agents.alite.vis.element.aggregation.StyledLineElements;
import cz.agents.alite.vis.element.aggregation.StyledPointElements;
import cz.agents.alite.vis.element.implemetation.CircleImpl;
import cz.agents.alite.vis.element.implemetation.StyledLineImpl;
import cz.agents.alite.vis.element.implemetation.StyledPointImpl;
import cz.agents.alite.vis.layer.GroupLayer;
import cz.agents.alite.vis.layer.VisLayer;
import cz.agents.alite.vis.layer.common.CommonLayer;
import cz.agents.alite.vis.layer.terminal.CircleLayer;
import cz.agents.alite.vis.layer.terminal.StyledLineLayer;
import cz.agents.alite.vis.layer.terminal.StyledPointLayer;
import cz.agents.deconfliction.environment.embodiment.Airplane;
import cz.agents.deconfliction.environment.storage.AirplaneStorage;

public class AirplaneLayer extends CommonLayer {

    public static VisLayer create(final AirplaneStorage airplaneStorage, final double separation, final double tipLength) {
        GroupLayer group = GroupLayer.create();

        group.addSubLayer(StyledPointLayer.create(new StyledPointElements() {

            @Override
            public Iterable<? extends StyledPoint> getPoints() {
                ArrayList<StyledPoint> points = new ArrayList<StyledPoint>();

                for (Airplane airplaneInfo : airplaneStorage.getAirplanes().values()) {
                    Color color = Color.RED;
                    points.add(new StyledPointImpl(airplaneInfo.currentPositionDirection, color, 12));
                }

                return points;
            }

        }));

        group.addSubLayer(StyledLineLayer.create(new StyledLineElements() {

            @Override
            public Iterable<? extends StyledLine> getLines() {
                ArrayList<StyledLine> lines = new ArrayList<StyledLine>();

                for (Airplane airplaneInfo : airplaneStorage.getAirplanes().values()) {
                    Color color = Color.RED;

                    Vector arrowTip = new Vector(airplaneInfo.currentPositionDirection.orientation);
                    arrowTip.scale(tipLength);
                    arrowTip.add(airplaneInfo.currentPositionDirection);
                    lines.add(new StyledLineImpl(airplaneInfo.currentPositionDirection, new Point3d(arrowTip), color, 2));
                }

                return lines;
            }

        }));

        group.addSubLayer(CircleLayer.create(new CircleElements() {

            @Override
            public Iterable<? extends Circle> getCircles() {
                ArrayList<Circle> circles = new ArrayList<Circle>();

                for (Airplane airplaneInfo : airplaneStorage.getAirplanes().values()) {
                    circles.add(new CircleImpl(airplaneInfo.currentPositionDirection, separation/2.0));
                }

                return circles;
            }

            @Override
            public Color getColor() {
                return Color.DARK_GRAY;
            }

            @Override
            public int getStrokeWidth() {
                return 1;
            }

        }));

        group.addSubLayer(AgentIdLayer.create(airplaneStorage, Color.BLACK, 1, "a"));
        return group;
    }
}

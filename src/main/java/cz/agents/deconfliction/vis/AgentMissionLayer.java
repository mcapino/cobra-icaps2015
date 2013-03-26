package cz.agents.deconfliction.vis;

import java.awt.Color;
import java.util.ArrayList;

import cz.agents.alite.vis.element.Line;
import cz.agents.alite.vis.element.StyledPoint;
import cz.agents.alite.vis.element.aggregation.LineElements;
import cz.agents.alite.vis.element.aggregation.StyledPointElements;
import cz.agents.alite.vis.element.implemetation.StyledLineImpl;
import cz.agents.alite.vis.element.implemetation.StyledPointImpl;
import cz.agents.alite.vis.layer.GroupLayer;
import cz.agents.alite.vis.layer.VisLayer;
import cz.agents.alite.vis.layer.common.CommonLayer;
import cz.agents.alite.vis.layer.terminal.ArrowLayer;
import cz.agents.alite.vis.layer.terminal.StyledPointLayer;
import cz.agents.alite.vis.layer.toggle.KeyToggleLayer;
import cz.agents.deconfliction.agent.Agent;
import cz.agents.deconfliction.problem.DeconflictionProblem;

public class AgentMissionLayer extends CommonLayer {


    public static VisLayer create(final DeconflictionProblem problem, String toggleKey) {
        GroupLayer group = GroupLayer.create();

        group.addSubLayer(StyledPointLayer.create(new StyledPointElements() {

            @Override
            public Iterable<? extends StyledPoint> getPoints() {
                ArrayList<StyledPoint> points = new ArrayList<StyledPoint>();

                int i = 0;
                for (Agent agent : problem.getAgents()) {
                    points.add(new StyledPointImpl(agent.getStart(), AgentColors.getColorForAgent(i), 8));
                    //points.add(new StyledPointImpl(agent.getDestination(), AgentColors.getColorForAgent(i), 8));
                    i++;
                }

                return points;
            }

        }));

        group.addSubLayer(ArrowLayer.create(new LineElements() {

            @Override
            public Color getColor() {
                return Color.BLACK;
            }

            @Override
            public int getStrokeWidth() {
                return 1;
            }

            @Override
            public Iterable<? extends Line> getLines() {
                ArrayList<Line> lines = new ArrayList<Line>();


                int i = 0;
                for (Agent agent : problem.getAgents()) {
                    lines.add(new StyledLineImpl(agent.getStart(), agent.getDestination(), AgentColors.getColorForAgent(i), 2));
                    i++;
                }
                return lines;

            }

        }));
        /*
        group.addSubLayer(StyledLineLayer.create(new StyledLineElements() {

            @Override
            public Iterable<? extends StyledLine> getLines() {
                ArrayList<StyledLine> lines = new ArrayList<StyledLine>();

                int i = 0;
                for (Agent agent : problem.getAgents()) {
                    lines.add(new StyledLineImpl(agent.getStart(), agent.getDestination(), AgentColors.getColorForAgent(i), 2));
                    i++;
                }
                return lines;
            }

        }));*/


        KeyToggleLayer toggle = KeyToggleLayer.create(toggleKey);
        toggle.addSubLayer(group);
        toggle.setEnabled(true);

        return toggle;

    }
}

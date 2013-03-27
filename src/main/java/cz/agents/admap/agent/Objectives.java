package cz.agents.admap.agent;

import tt.euclid2i.Point;
import tt.euclid2i.region.Region;

public class Objectives {
    Point start;
    Region goal;

    public Objectives(Point start, Region goal) {
        super();
        this.start = start;
        this.goal = goal;
    }
}

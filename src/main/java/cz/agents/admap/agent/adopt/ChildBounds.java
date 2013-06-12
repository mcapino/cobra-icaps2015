package cz.agents.admap.agent.adopt;

public class ChildBounds {
    double lowerBound;
    double upperBound;
    double threshold;
    Context context;

    public ChildBounds() {
        this(Double.POSITIVE_INFINITY, 0, 0, new Context());
    }

    public ChildBounds(double lowerBound, double upperBound, double threshold,
            Context context) {
        super();
        this.lowerBound = lowerBound;
        this.upperBound = upperBound;
        this.threshold = threshold;
        this.context = context;
    }
}

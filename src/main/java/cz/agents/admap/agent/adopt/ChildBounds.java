package cz.agents.admap.agent.adopt;

public class ChildBounds {
    double lowerBound;
    double upperBound;
    double threshold;
    Context context;

    public ChildBounds() {
        this(0, Double.POSITIVE_INFINITY, 0, new Context());
    }

    public ChildBounds(double lowerBound, double upperBound, double threshold,
            Context context) {
        super();
        this.lowerBound = lowerBound;
        this.upperBound = upperBound;
        this.threshold = threshold;
        this.context = context;
    }

    @Override
    public String toString() {
        return String.format("(lb:%.2f, ub:%.2f, t:%.2f, context:%s)", lowerBound, upperBound, threshold, context.toString());
    }
}

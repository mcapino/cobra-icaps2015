package tt.euclidtime3i.discretization;

import org.jgrapht.DirectedGraph;
import org.jgrapht.Graph;
import org.jgrapht.graph.GraphDelegator;

import tt.euclidtime3i.Point;

@SuppressWarnings("serial")
public class ControlEffortWrapper extends GraphDelegator<Point, Straight> implements DirectedGraph<Point, Straight> {


    private double movingPenaltyPerSec;

	public ControlEffortWrapper(Graph<Point, Straight> g, double movePenaltyPerSec) {
        super(g);
        this.movingPenaltyPerSec = movePenaltyPerSec;
    }

	@Override
    public double getEdgeWeight(Straight e) {
        if (e.getStart().getPosition().distance(e.getEnd().getPosition()) < 0.01) {
            return super.getEdgeWeight(e);
        } else {
            return super.getEdgeWeight(e) + movingPenaltyPerSec * (e.duration());
        }
    }
}
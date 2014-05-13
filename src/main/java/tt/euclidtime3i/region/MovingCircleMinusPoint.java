package tt.euclidtime3i.region;

import tt.euclidtime3i.Point;
import tt.euclidtime3i.Region;

public class MovingCircleMinusPoint implements Region {

	private MovingCircle movingCircle;
	private tt.euclid2i.Point pointToSubtract;

    public MovingCircleMinusPoint(MovingCircle movingCircle, tt.euclid2i.Point pointToSubtract) {
        super(); 
        this.movingCircle = movingCircle;
        this.pointToSubtract = pointToSubtract;
    }

	@Override
	public boolean intersectsLine(Point p1, Point p2) {
		if (p1.getPosition().equals(pointToSubtract) && p2.getPosition().equals(pointToSubtract)) {
			return false;
		}
		return movingCircle.intersectsLine(p1, p2);
	}

    @Override
    public HyperRectangle getBoundingBox() {
    	return movingCircle.getBoundingBox();
    }

    @Override
    public String toString() {
        return (movingCircle.toString() + "\\" + pointToSubtract.toString());
    }

	@Override
	public boolean isInside(Point p) {
		if (p.getPosition().equals(pointToSubtract)) {
			return false;
		} else {
			return movingCircle.isInside(p);
		}
	}

}

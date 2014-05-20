package tt.euclidtime3i.util;

import java.util.Collection;

import tt.euclid2i.Point;
import tt.euclid2i.util.SeparationDetector;
import tt.euclidtime3i.Region;
import tt.euclidtime3i.region.MovingCircle;
import tt.util.NotImplementedException;

public class IntersectionCheckerWithProtectedPoint {

    public static boolean intersect(Region thisRegion, Collection<Region> obstacleCollection, Point protectedPoint) {
        assert(thisRegion != null);
        assert(!obstacleCollection.contains(null));

        Region[] obstacles = obstacleCollection.toArray(new Region[1]);

        for (int j = 0; j < obstacles.length; j++) {
            if (obstacles[j] != null) {
            	if (/*thisRegion.getBoundingBox().intersects(obstacles[j].getBoundingBox())*/ true) {
            		if (intersectIgnoreProtectedPoint(thisRegion, obstacles[j], protectedPoint)) {
            			return true;
            		}
            	}
            }
        }

        return false;
    }

	public static boolean intersectIgnoreProtectedPoint(Region thisRegion, Region otherRegion, Point protectedPoint) {
		if (thisRegion instanceof MovingCircle && otherRegion instanceof MovingCircle) {
			MovingCircle thisMc = (MovingCircle) thisRegion;
			MovingCircle otherMc = (MovingCircle) otherRegion;

			return SeparationDetector.hasConflictIgnoreProtectedPoint(
					thisMc.getTrajectory(),
					otherMc.getTrajectory(),
					protectedPoint,
					thisMc.getRadius() + otherMc.getRadius(),
					(int) Math.floor(Math.min(thisMc.getRadius(), otherMc.getRadius())/4.0));
		}

		throw new NotImplementedException("The conflict checking of " + thisRegion.getClass() + " vs. " + otherRegion.getClass() + " not implemented yet");
	}
}

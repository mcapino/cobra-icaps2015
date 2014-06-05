package tt.euclidtime3i.region;

import tt.euclid2i.Trajectory;

public class CircleMovingToTarget extends MovingCircle {

	private int targetReachedTime;

	public CircleMovingToTarget(Trajectory trajectory, int radius, int targetReachedTime) {
		super(trajectory, radius);
		this.targetReachedTime = targetReachedTime;
	}
	
	public int getTargetReachedTime() {
		return targetReachedTime;
	}	
	
    @Override
    public String toString() {
        return "MC(" + Integer.toHexString(trajectory.hashCode()) + ", r=" + radius + ", tdest="+ targetReachedTime + ")";
    }
}

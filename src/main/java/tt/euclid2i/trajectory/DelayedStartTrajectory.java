package tt.euclid2i.trajectory;

import tt.euclid2i.EvaluatedTrajectory;
import tt.euclid2i.Point;
import tt.euclid2i.Trajectory;

public class DelayedStartTrajectory implements EvaluatedTrajectory {
	
	EvaluatedTrajectory baseTraj;
	int delay;

	public DelayedStartTrajectory(EvaluatedTrajectory baseTraj, int delay) {
		super();
		this.baseTraj = baseTraj;
		this.delay = delay;
	}

	@Override
	public int getMinTime() {
		return baseTraj.getMinTime();
	}

	@Override
	public int getMaxTime() {
		return baseTraj.getMaxTime() + delay;
	}

	@Override
	public Point get(int t) {
		if (t <= baseTraj.getMinTime() + delay) {
			return baseTraj.get(0);
		} else {
			return baseTraj.get(t-delay);
		}
	}

	@Override
	public double getCost() {
		return baseTraj.getCost();
	}

}

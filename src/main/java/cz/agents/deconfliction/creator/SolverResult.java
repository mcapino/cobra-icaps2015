package cz.agents.deconfliction.creator;

public class SolverResult {
	final double solutionCost;
	final double wallclockRuntime;
	final double cummulativeRuntime;
	final double broadcastedMessageCount;
	final double broadcastedMessageSize;
	
	public SolverResult(double solutionCost, double wallclockRuntime,
			double cummulativeRuntime, double broadcastedMessageCount,
			double broadcastedMessageSize) {
		this.solutionCost = solutionCost;
		this.wallclockRuntime = wallclockRuntime;
		this.cummulativeRuntime = cummulativeRuntime;
		this.broadcastedMessageCount = broadcastedMessageCount;
		this.broadcastedMessageSize = broadcastedMessageSize;
	}

	public double getSolutionCost() {
		return solutionCost;
	}

	public double getWallclockRuntime() {
		return wallclockRuntime;
	}

	public double getCummulativeRuntime() {
		return cummulativeRuntime;
	}

	public double getBroadcastedMessageCount() {
		return broadcastedMessageCount;
	}

	public double getBroadcastedMessageSize() {
		return broadcastedMessageSize;
	}
	
	public String asCSV() {
		return "";
	}
	
}

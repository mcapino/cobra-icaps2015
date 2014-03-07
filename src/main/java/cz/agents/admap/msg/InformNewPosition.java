package cz.agents.admap.msg;

import javax.vecmath.Tuple2i;

import tt.euclid2d.Point;
import tt.euclid2d.Vector;
import cz.agents.alite.communication.content.Content;

public class InformNewPosition extends Content {

	final String agentName;
    final int agentId;
	final Point position;
    final Vector velocity;
	final double radius;

	public InformNewPosition(String agentName, int agentId, Point position, Vector velocity, double radius) {
		super(null);
		this.agentName = agentName;
		this.agentId = agentId;
		this.position = position;
		this.velocity = velocity;
		this.radius = radius;
	}


	@Override
	public String toString() {
		return "InformNewPosition [agentName=" + agentName + ", agentId="
				+ agentId + ", position=" + position + ", velocity=" + velocity
				+ ", radius=" + radius + "]";
	}

	public String getAgentName() {
		return agentName;
	}

	public int getAgentId() {
		return agentId;
	}

	public Point getPosition() {
		return position;
	}

	public Vector getVelocity() {
		return velocity;
	}

	public double getRadius() {
		return radius;
	}


}

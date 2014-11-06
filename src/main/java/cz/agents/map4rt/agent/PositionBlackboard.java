package cz.agents.map4rt.agent;

import java.util.HashMap;
import java.util.Map;

import tt.euclid2d.Point;
import tt.euclid2d.Vector;

public class PositionBlackboard {
	
	public static class Record {
		public Record(int agentId, Point position, Vector velocity, double radius) {
			this.agentId = agentId;
			this.position = position;
			this.velocity = velocity;
			this.radius = radius;
		}
	    final int agentId;
		final Point position;
	    final Vector velocity;
		final double radius;
	}
	
	static Map<String, Record> records = new HashMap<>();
	
	public static  synchronized void recordNewPosition(String name, int agentId, Point position, Vector velocity, double radius) {
		records.put(name, new Record(agentId, position, velocity, radius));
	}
	
	public static synchronized Map<String, Record> getRecords() {
		return new HashMap<String, Record>(records);
	}
}

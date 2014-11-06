package cz.agents.map4rt.agent;

import java.util.HashMap;
import java.util.Map;

import tt.euclid2i.Point;

public class CurrentTasks {
	static Map<String, Point> tasks = new HashMap<String, Point>();
	
	synchronized public static void registerTask(String name, Point dest) {
		tasks.put(name, dest);
	}
	
	synchronized public static boolean isFreeFromOtherTasks(String agent, Point point) {
		HashMap<String, Point> tasksWithoutAgent = new HashMap<String, Point>(tasks);
		tasksWithoutAgent.remove(agent);
		return !tasksWithoutAgent.values().contains(point);
	}
	
	synchronized public static Map<String, Point> getTasks() {
		return new HashMap<String, Point>(tasks);
	}
	
}

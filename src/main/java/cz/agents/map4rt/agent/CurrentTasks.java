package cz.agents.map4rt.agent;

import java.util.HashMap;
import java.util.Map;

import tt.euclid2i.Point;

public class CurrentTasks {
	static Map<String, Point> tasks = new HashMap<String, Point>();
	
	synchronized public static boolean tryToRegisterTask(String name, Point dest) {
		HashMap<String, Point> tasksWithoutAgent = new HashMap<String, Point>(tasks);
		tasksWithoutAgent.remove(name);
		
		boolean free = !tasksWithoutAgent.values().contains(dest);
		
		if (free) {
			tasks.put(name, dest);
			return true;
		} else {
			return false;
		}
	}
	
	synchronized public static Map<String, Point> getTasks() {
		return new HashMap<String, Point>(tasks);
	}
	
}

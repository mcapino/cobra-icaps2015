package cz.agents.map4rt.agent;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Random;

import tt.euclid2i.Point;
import tt.jointeuclid2ni.probleminstance.RelocationTask;

public class CurrentTasks {
	public static List<Point> docks = null;
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

	synchronized public static Point assignRandomDestination(String name, Random rnd) {
		List<Point> unassignedDestinations = new LinkedList<Point>(docks);
		unassignedDestinations.removeAll(tasks.values());
		Point[] dests = unassignedDestinations.toArray(new Point[0]);
		Point destination = dests[rnd.nextInt(dests.length)];
		tasks.put(name, destination);
		return destination;
	}
	
}

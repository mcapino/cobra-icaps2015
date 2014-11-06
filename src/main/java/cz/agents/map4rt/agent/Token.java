package cz.agents.map4rt.agent;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import tt.euclidtime3i.Region;
import tt.euclidtime3i.region.MovingCircle;

public class Token {
	
	static boolean locked;
	static Map<String, Region> regions = new HashMap<String, Region>();
	
	static List<Region> getReservedRegions(String askingAgent) {
		List<Region> reservedRegions =  new LinkedList<Region>();
		for (String agentName : regions.keySet()) {
			if (!agentName.equals(askingAgent))
				reservedRegions.add(regions.get(agentName));
		}
		return reservedRegions;
	}

	public static void register(String name, MovingCircle movingCircle) {
		regions.put(name, movingCircle);
	}
	
	synchronized static boolean tryLock() {
		if (!locked) {
			locked = true;
			return true;
		} else {
			return false;
		}
	}
	
	synchronized static void unlock() {
			locked = false;
	}
	
}

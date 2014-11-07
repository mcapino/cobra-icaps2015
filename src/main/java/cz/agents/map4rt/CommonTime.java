package cz.agents.map4rt;

public class CommonTime {
	static long startedAt = System.currentTimeMillis(); 
	
	public static int currentTimeMs() {
		return (int) (System.currentTimeMillis() - startedAt);
	}
}

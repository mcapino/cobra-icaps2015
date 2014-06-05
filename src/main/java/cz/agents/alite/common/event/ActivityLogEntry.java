package cz.agents.alite.common.event;

public class ActivityLogEntry {
	public static enum Type {EVENT_HANDLED, IDLE, EVENT_RECIEVED};
	
	public String processName;
	public Type type;
	public long startTime;
	public long duration;
	public long expandedStates;
	
	public ActivityLogEntry(String processName, Type type, long startTime,
			long duration, long expandedStates) {
		super();
		this.processName = processName;
		this.type = type;
		this.startTime = startTime;
		this.duration = duration;
		this.expandedStates = expandedStates;
	}
}

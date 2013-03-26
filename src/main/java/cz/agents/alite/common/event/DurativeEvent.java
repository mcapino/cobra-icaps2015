package cz.agents.alite.common.event;


public final class DurativeEvent implements Comparable<DurativeEvent> {

    private final long id;
    private final long time;
    private final String process;
    private final DurativeEventHandler handler;

    
    
    public DurativeEvent(long id, long time, String process, DurativeEventHandler handler) {
		super();
		this.id = id;
		this.time = time;
		this.process = process;
		this.handler = handler;
	}
    

	public long getTime() {
		return time;
	}

	public String getProcess() {
		return process;
	}

	public DurativeEventHandler getHandler() {
		return handler;
	}

	@Override
    public int compareTo(DurativeEvent event) {
        if (time == event.time) {
            if (id == event.id) {
                throw new RuntimeException("Found same event ids in comparation!");
            }
            return (id < event.id ? -1 : 1);
        } else {
            return (time < event.time ? -1 : (time == event.time ? 0 : 1));
        }
    }

    @Override
    public String toString() {
        return time + "@" + process + ": " + handler;
    }

}
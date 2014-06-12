package cz.agents.alite.common.event;

import java.util.Collection;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.SortedSet;
import java.util.TreeSet;

import org.apache.log4j.Level;
import org.apache.log4j.Logger;

import tt.util.Counters;
import cz.agents.alite.common.event.ActivityLogEntry.Type;

public class DurativeEventProcessor {

    private volatile boolean running = true;
    private volatile boolean finished = false;

    private long eventIdCounter = 0;
    private Thread thread = Thread.currentThread();

    private final Queue<DurativeEvent> eventQueue = new PriorityQueue<DurativeEvent>();

    private final Map<String,Long> processLastActivityFinishedTimes = new HashMap<String, Long>();
    private final Map<String,Long> processIdleCounter = new HashMap<String, Long>();
    private final Map<String,Long> processActiveCounter = new HashMap<String, Long>();
    
    private long lastEventStartedAtNanos = 0;
    private long currentEventTime = 0;
    
    private boolean keepActivityLog = false;
    private Collection<ActivityLogEntry> activityLog = new LinkedList<ActivityLogEntry>();

    public void run() {
        DurativeEvent event = eventQueue.poll();

        while (event != null) {
            beforeProcessingEvent(event);

            long time = event.getTime();
            String processName = event.getProcess();
            long lastActivityFinishes = getProcessLastActivityFinishedTime(processName);
            
            if (keepActivityLog)
            	activityLog.add(new ActivityLogEntry(processName, Type.EVENT_RECIEVED, time, 0, 0));

            if (time >= lastActivityFinishes) {

                // process is idle
                long idleTime = time - lastActivityFinishes;
                processIdleCounter.put(processName, getProcessIdleCounter(processName) + idleTime);
                
                if (keepActivityLog && idleTime > 0) {
                	 activityLog.add(new ActivityLogEntry(processName, Type.IDLE, lastActivityFinishes, idleTime, 0));
                }

                while (!running) {
                    synchronized (thread) {
                        try {
                            if (!running) {
                                thread.wait();
                            }
                        } catch (InterruptedException ex) {
                            Logger.getLogger(EventProcessor.class.getName()).log(Level.ERROR, null, ex);
                        }
                    }
                }
                
                int expandedStatesBefore = Counters.expandedStatesCounter;
                
                long duration = fireEvent(event);
                
                int expandedStatesAfter = Counters.expandedStatesCounter;
                
                processActiveCounter.put(processName, getProcessActiveCounter(processName) + duration);
                processLastActivityFinishedTimes.put(processName, time + duration);
                if (keepActivityLog)
                	activityLog.add(new ActivityLogEntry(processName, Type.EVENT_HANDLED, time, duration, expandedStatesAfter - expandedStatesBefore));
            } else {
                // move to future
                addEvent(lastActivityFinishes, processName, event.getHandler());
            }

            event = eventQueue.poll();
        }
        finished = true;
        running = false;
    }

    /**
     * Ends the the event processor by clearing the event queue.
     *
     * This method has to be called from the same thread as the run() method was called!
     */
    public void clearQueue() {
        eventQueue.clear();
    }

    public void addEvent(long time, String process, DurativeEventHandler eventHandler) {
        DurativeEvent event = new DurativeEvent(eventIdCounter++, time, process, eventHandler);
        eventQueue.add(event);
    }

    /**
     * Method pauses and un-pauses the processing of the events.
     *
     * The method can be called from other threads (than the run() method was called).
     *
     * @param running
     */
    public void setRunning(boolean running) {
        this.running = running;
        if (running) {
            synchronized (thread) {
                thread.notify();
            }
        }
    }

    /**
     * The method can be called from other threads (than the run() method was called).
     */
    public boolean isRunning() {
        return running;
    }

    /**
     * The method can be called from other threads (than the run() method was called).
     */
    public boolean isFinished() {
        return finished;
    }

    /**
     * The method can be called from other threads (than the run() method was called).
     */
    public long getProcessLastActivityFinishedTime(String process) {
        if (processLastActivityFinishedTimes.containsKey(process)) {
            return processLastActivityFinishedTimes.get(process);
        } else {
            return 0;
        }
    }

    protected SortedSet<String> getProcessNames(){
        return new TreeSet<String>(processLastActivityFinishedTimes.keySet());
    }

    /**
     * The method can be called from other threads (than the run() method was called).
     */
    public int getCurrentQueueLength() {
        return eventQueue.size();
    }

    protected void breforeRunningTest(DurativeEvent event) {
    }
    /**
     * @return duration of the event handling
     */
    private long fireEvent(DurativeEvent event) {
    	
        if (event.getHandler() != null) {
            currentEventTime = event.getTime();
            lastEventStartedAtNanos = System.nanoTime();

            long duration = event.getHandler().handleEvent(event);

            if (duration == DurativeEventHandler.COUNT_SYSTEM_NANOS) {
                duration = getCurrentEventHandlingDurationNanos();
            }

            return duration;

        }
        return 0;
    }

    public long getProcessIdleCounter(String processName) {
        if (processIdleCounter.containsKey(processName)) {
            return processIdleCounter.get(processName);
        } else {
            return 0;
        }
    }

    protected long getProcessActiveCounter(String processName) {
        if (processActiveCounter.containsKey(processName)) {
            return processActiveCounter.get(processName);
        } else {
            return 0;
        }
    }

    public long getCurrentEventHandlingDurationNanos() {
        if (lastEventStartedAtNanos != 0) {
            return System.nanoTime() - lastEventStartedAtNanos;
        } else {
            return 0;
        }
    }

    public long getCurrentEventTime() {
        return currentEventTime;
    }

    protected void beforeProcessingEvent(DurativeEvent event) {}

    public String getProcessTimesAsString() {
        StringBuilder sb = new StringBuilder();
        for (String processName : getProcessNames()) {
            sb.append(processName);
            sb.append("@");
            sb.append(processLastActivityFinishedTimes.get(processName));
            sb.append(" ");
        }
        return sb.toString();
    }
    
    public Collection<ActivityLogEntry> getActivityLog() {
		return activityLog;
	}
    
    public void setKeepActivityLog(boolean keepActivityLog) {
		this.keepActivityLog = keepActivityLog;
	}
}

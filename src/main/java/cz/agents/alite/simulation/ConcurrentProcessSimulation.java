package cz.agents.alite.simulation;

import org.apache.log4j.Logger;

import cz.agents.alite.common.event.DurativeEvent;
import cz.agents.alite.common.event.DurativeEventProcessor;

public class ConcurrentProcessSimulation extends DurativeEventProcessor {

    private Logger LOGGER = Logger.getLogger(ConcurrentProcessSimulation.class);

    private long eventCount = 0;
    private long runTime;
    private int printouts = 10000;
    
    public ConcurrentProcessSimulation() {
    }

    @Override
    public void run() {
        runTime = System.currentTimeMillis();
        LOGGER.info(">>> CONCURRENT PROCESS SIMULATION START");

        super.run();

        LOGGER.info(">>> SIMULATION FINISH");

        for (String processName : getProcessNames()) {
            LOGGER.info(String.format(">>> PROCESS TERMINATION TIME: %s %.4fs (Active: %.4fs Idle: %.4fs)", processName,getProcessLastActivityFinishedTime(processName)/1e9d, getProcessActiveCounter(processName)/1e9d, getProcessIdleCounter(processName)/1e9f));
        }

        LOGGER.info(String.format(">>> EVENT COUNT: %d", eventCount));
        LOGGER.info(String.format(">>> RUNTIME: %.2fs", (System.currentTimeMillis() - runTime) / 1000.0));
    }

    public long getEventCount() {
        return eventCount;
    }

    /** print every n-th event */
    public void setPrintouts(int n) {
        this.printouts = n;
    }

    @Override
    protected void beforeProcessingEvent(DurativeEvent event) {
        ++eventCount;

        if (eventCount % printouts == 0) {
            LOGGER.debug(String.format(
                            ">>> SIM. TIME: %s / RUNTIME: %.2fs / EVENTS: %d / QUEUE: %d",
                            getProcessTimesAsString(),
                            (System.currentTimeMillis() - runTime) / 1000.0,
                            eventCount, getCurrentQueueLength()));
        }
    }

    public long getCumulativeActiveRuntime() {
        long result = 0;
        for (String processName : getProcessNames() ) {
            result += getProcessLastActivityFinishedTime(processName) - getProcessIdleCounter(processName);
        }
        return result;
    }

    public long getCumulativeIdleRuntime() {
        long result = 0;
        long wallclock = getWallclockRuntime();
        for (String processName : getProcessNames() ) {
            result += getProcessIdleCounter(processName);
            result += wallclock - this.getProcessLastActivityFinishedTime(processName);
        }
        return result;
    }

    public long getWallclockRuntime() {
        long max = 0;
        for (String processName : getProcessNames() ) {
            long processLastActivityFinished = getProcessLastActivityFinishedTime(processName);

            if (processLastActivityFinished > max) {
                max = processLastActivityFinished;
            }
        }
        return max;
    }
}

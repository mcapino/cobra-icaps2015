package cz.agents.alite.common.event;


public interface DurativeEventHandler {

    final static long COUNT_SYSTEM_NANOS = -1;

    public DurativeEventProcessor getEventProcessor();

    /**
     * @return the duration of computation associated with handling the event,
     * 		   in simulation time.
     *         Returns {@link DurativeEventHandler#COUNT_SYSTEM_NANOS}
     *         if the duration of the event should be determined automatically
     *         using the walltime.
     */
    public long handleEvent(DurativeEvent event);

}

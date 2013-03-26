package cz.agents.alite.communication.eventbased;

import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import cz.agents.alite.common.event.DurativeEvent;
import cz.agents.alite.common.event.DurativeEventHandler;
import cz.agents.alite.common.event.DurativeEventProcessor;
import cz.agents.alite.communication.CommunicationReceiver;
import cz.agents.alite.communication.Message;
import cz.agents.alite.communication.channel.CommunicationChannelException;
import cz.agents.alite.communication.channel.DirectCommunicationChannel;

public class ConcurrentProcessCommunicationChannel extends DirectCommunicationChannel {

    private final DurativeEventProcessor eventProcessor;
    private final long messageDelay = 0;
    private final Map<String, List<Long>> inboxCounters;

    public ConcurrentProcessCommunicationChannel(CommunicationReceiver communicator, DurativeEventProcessor eventProcessor, ReceiverTable channelReceiverTable, Map<String, List<Long>> inboxCounters) throws CommunicationChannelException {
        super(communicator, channelReceiverTable);
        this.eventProcessor = eventProcessor;
        this.inboxCounters = inboxCounters;
    }

    @Override
    protected void callDirectReceive(final CommunicationReceiver receiver,
            final Message message) {
        long lastEventDuration = eventProcessor
                .getCurrentEventHandlingDurationNanos();

        /// This is probably wrong!!!!
        /*
        callDirectReceive(
                eventProcessor.getProcessLastActivityFinishedTime(this.getCommunicationReceiver()
                        .getAddress()) + lastEventDuration + messageDelay,
                receiver, message);
        */

        callDirectReceive(
                eventProcessor.getCurrentEventTime() + lastEventDuration + messageDelay,
                receiver, message);

    }

    protected void callDirectReceive(final long time, final CommunicationReceiver receiver, final Message message) {
        recordToInboxCounter(receiver.getAddress(), time);
        eventProcessor.addEvent(time, receiver.getAddress(), new DurativeEventHandler() {

            @Override
            public DurativeEventProcessor getEventProcessor() {
                return eventProcessor;
            }

            @Override
            public long handleEvent(DurativeEvent event) {
                removeFromInboxCounter(event.getProcess(), time);
                receiver.receiveMessage(message);
                return COUNT_SYSTEM_NANOS;
            }
        });
    }

    private void recordToInboxCounter(String processName, long time) {
        if (!inboxCounters.containsKey(processName)) {
            inboxCounters.put(processName, new LinkedList<Long>());
        }

        inboxCounters.get(processName).add(time);
    }

    private void removeFromInboxCounter(String processName, long time) {
        inboxCounters.get(processName).remove(time);
    }

    public int getInboxSize(String processName) {
        long currentTime = eventProcessor.getCurrentEventTime() + eventProcessor.getCurrentEventHandlingDurationNanos();
        if (inboxCounters.containsKey(processName)) {
            int counter = 0;

            for (long timeStamp : inboxCounters.get(processName)) {
                if (timeStamp <= currentTime) {
                    counter++;
                }
            }
            return counter;

        } else {
            return 0;
        }
    }




}
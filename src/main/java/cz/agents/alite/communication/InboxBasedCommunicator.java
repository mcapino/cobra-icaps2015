package cz.agents.alite.communication;

import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;

import cz.agents.alite.communication.channel.CommunicationChannelException;
import cz.agents.alite.communication.content.Content;
import cz.agents.alite.communication.content.error.ErrorContent;
import cz.agents.alite.communication.eventbased.ConcurrentProcessCommunicationChannel;

public class InboxBasedCommunicator implements Communicator {

    private final String address;
    private ConcurrentProcessCommunicationChannel channel = null;
    private final List<MessageHandler> messageHandlers = new CopyOnWriteArrayList<MessageHandler>();
    private int messagesSent = 0;

    private static long counter = System.currentTimeMillis();

    /**
     *
     * @param address
     */
    public InboxBasedCommunicator(String address) {
        this.address = address;
    }

    /**
     * Adds communication channel to the communicator.
     *
     * @param channel
     */
    public void setChannel(ConcurrentProcessCommunicationChannel channel) {
        this.channel = channel;
    }


    @Override
    public String getAddress() {
        return address;
    }


    @Override
    public Message createMessage(Content content) {
        return new Message(address, content, generateId());
    }


    @Override
    public Message createReply(Message message, Content content) {
        Message reply = new Message(address, content, generateId());
        reply.addReceiver(message.getSender());
        return reply;
    }


    @Override
    public void addMessageHandler(MessageHandler handler) {
        messageHandlers.add(handler);
    }


    @Override
    public void removeMessageHandler(MessageHandler handler) {
        messageHandlers.remove(handler);
    }


    @Override
    public void sendMessage(Message message) {
        try {
        	messagesSent++;
            channel.sendMessage(message);
        } catch (CommunicationChannelException e) {
            Message errorMessage = createMessage(new ErrorContent(e));
            errorMessage.addReceiver(getAddress());
            receiveMessage(errorMessage);
        }
    };


    @Override
    public synchronized void receiveMessage(Message message) {
        for (MessageHandler messageHandler : messageHandlers) {
            messageHandler.notify(message);
        }
    }

    private long generateId() {
        return address.hashCode() + counter;
    }

    public int getInboxSize() {
        return channel.getInboxSize(address);
    }

    public int getMessagesSent() {
		return messagesSent;
	}
}

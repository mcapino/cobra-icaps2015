package cz.agents.alite.communication.protocol.asynchronousbacktracking;

public class TimeMarkedMessage {
    protected long sentRelTimeNs;

    public TimeMarkedMessage(long sentRelTimeNs) {
        super();
        this.sentRelTimeNs = sentRelTimeNs;
    }

    public long getSentRelTimeNs() {
        return sentRelTimeNs;
    }
}

package cz.agents.alite.communication.protocol.asynchronousbacktracking;

public class RequestNeigborMessage extends TimeMarkedMessage{
    String newNeighborName;

    public RequestNeigborMessage(String newNeighborName, long sentRelTimeNs) {
        super(sentRelTimeNs);
        this.newNeighborName = newNeighborName;
    }

    public String getNewNeighborName() {
        return newNeighborName;
    }
}

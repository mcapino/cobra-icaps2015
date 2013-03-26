package cz.agents.alite.communication.protocol.asynchronousbacktracking;

import java.util.Collection;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

import org.apache.log4j.Logger;

import cz.agents.alite.communication.Communicator;
import cz.agents.alite.communication.Message;
import cz.agents.alite.communication.MessageHandler;
import cz.agents.alite.communication.protocol.DefaultProtocol;
import cz.agents.alite.communication.protocol.Performative;
import cz.agents.alite.communication.protocol.ProtocolContent;
import cz.agents.alite.communication.protocol.ProtocolMessageHandler;
/**
 * An implementation of the asynchronous backtracking protocol.
 * See Chapter 1 "Distributed Constraint Satisfaction" in
 * Multiagent Systems by Shoham et al. for description of the protocol.
 *
 * http://www.masfoundations.org/download.html
 *
 * @author cap
 *
 */
public abstract class AsynchronousBacktrackingProtocol extends DefaultProtocol {

    private static final Logger LOGGER = Logger.getLogger(AsynchronousBacktrackingProtocol.class);

    static final String ABT_PROTOCOL_NAME = "ABT";
    private final MessageHandler messagehandler;
    private List<String> agents;
    private Set<String> outgoingLinks;
    private String agentName;
    private boolean stopped = false;

    public AsynchronousBacktrackingProtocol(Communicator communicator, String name, List<String> agents) {
        super(communicator, ABT_PROTOCOL_NAME);
        this.agents = agents;
        this.agentName = name;

        int myIndex = agents.lastIndexOf(name);
        outgoingLinks =  new HashSet<String>(agents.subList(myIndex+1, agents.size()));

        messagehandler = new ProtocolMessageHandler(this) {

            @Override
            public void handleMessage(Message message, ProtocolContent content) {
                processMessage(content.getData());
            }
        };
        communicator.addMessageHandler(messagehandler);

    }

    protected void processMessage(Object content) {

        if (stopped)
            return;

        if (content instanceof OkayMessage) {
            processOkayMessage((OkayMessage) content);
        }
        else if (content instanceof NogoodMessage) {
            processNogoodMessage((NogoodMessage) content);
        }
        else if (content instanceof StopMessage) {
            stopped = true;
            processStopMessage((StopMessage) content);
        }
        else if (content instanceof RequestNeigborMessage) {
            String newNeighbor = ((RequestNeigborMessage) content).getNewNeighborName();
            LOGGER.trace("ABT " + agentName + ": Requested to add " + newNeighbor + "as a neighbor.");
            if (!outgoingLinks.contains(newNeighbor)) {
                LOGGER.trace("ABT " + agentName + ": Adding " + newNeighbor + "as a neighbor.");
                outgoingLinks.add(newNeighbor);
                processNewNeighbor(newNeighbor);
            }

        }

    }

    protected abstract void processOkayMessage(OkayMessage content);

    protected abstract void processNogoodMessage(NogoodMessage content);

    protected abstract void processStopMessage(StopMessage content);

    protected abstract void processNewNeighbor(String agentName);

    public void sendOkay(Collection<String> receivers, OkayMessage content) {
        ProtocolContent pC = new ProtocolContent(this, Performative.INFORM, content, "");
        Message message = communicator.createMessage(pC);
        message.addReceivers(receivers);
        LOGGER.trace("ABT " + agentName + ": Sent okay? ( " + content.trajectory + " ) message to: " + receivers);
        communicator.sendMessage(message);
    }

    public void sendOkay(OkayMessage content) {
        sendOkay(outgoingLinks, content);
    }

    public void sendOkay(String receiver, OkayMessage content) {
        LinkedList<String> receivers = new LinkedList<String>();
        receivers.add(receiver);
        sendOkay(receivers, content);
    }


    public void sendNogood(String receiver, NogoodMessage content) {
        ProtocolContent pC = new ProtocolContent(this, Performative.REFUSE, content, "");
        Message message = communicator.createMessage(pC);
        message.addReceiver(receiver);
        communicator.sendMessage(message);
    }

    public void sendStop(StopMessage content) {
        ProtocolContent pC = new ProtocolContent(this, Performative.INFORM, content, "");
        Message message = communicator.createMessage(pC);
        message.addReceivers(agents);
        communicator.sendMessage(message);
    }

    public void sendRequestNeighbor(String receiver, RequestNeigborMessage content) {
        ProtocolContent pC = new ProtocolContent(this, Performative.REQUEST, content, "");
        Message message = communicator.createMessage(pC);
        message.addReceiver(receiver);
        communicator.sendMessage(message);
    }

    public void requestAddMeAsNeighbor(String requestedAgentName) {
        /// ATTENTION, ADDING NEIGHBORS DISABLED
        //sendRequestNeighbor(requestedAgentName, new RequestNeigborMessage(agentName));
    }

    public boolean isConnectedByOutgoingLink(String agent) {
        return outgoingLinks.contains(agent);
    }
}

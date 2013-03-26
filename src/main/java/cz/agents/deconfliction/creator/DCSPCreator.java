package cz.agents.deconfliction.creator;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import cz.agents.alite.common.entity.Entity;
import cz.agents.alite.common.event.DurativeEvent;
import cz.agents.alite.common.event.DurativeEventHandler;
import cz.agents.alite.common.event.DurativeEventProcessor;
import cz.agents.alite.communication.Communicator;
import cz.agents.alite.communication.DefaultCommunicator;
import cz.agents.alite.communication.InboxBasedCommunicator;
import cz.agents.alite.communication.channel.CommunicationChannelException;
import cz.agents.alite.communication.channel.DirectCommunicationChannel;
import cz.agents.alite.communication.channel.DirectCommunicationChannel.ReceiverTable;
import cz.agents.alite.communication.eventbased.ConcurrentProcessCommunicationChannel;
import cz.agents.alite.communication.eventbased.EventBasedCommunicationChannel;
import cz.agents.alite.creator.Creator;
import cz.agents.deconfliction.agent.Agent;
import cz.agents.deconfliction.agent.CommunicatingAgent;

public class DCSPCreator extends DeconflictionCreator implements Creator{

    private ReceiverTable receiverTable = new DirectCommunicationChannel.DefaultReceiverTable();
    private Map<String, List<Long>> inboxCounters = new HashMap<String, List<Long>>();
    protected void startResolution() {

        for (final Agent agent : getProblem().getAgents()) {

           getConcurrentSimulation().addEvent(0, agent.getName(), new DurativeEventHandler() {
               @Override
               public long handleEvent(DurativeEvent event) {
                   if (getParams().RESOLVE_CONFLICTS) {
                       agent.start();
                   }
                   return COUNT_SYSTEM_NANOS;
               }

               @Override
               public DurativeEventProcessor getEventProcessor() {
                   return getConcurrentSimulation();
               }
           });
       }

       getConcurrentSimulation().run();
    }

    protected void setupAgents() {
        // Create

        List<Agent> agents = createAgents();
        Collections.sort(agents);

        List<String> agentNames =  new ArrayList<String>(agents.size());
        for (Agent agent : agents) {
            agentNames.add(agent.getName());
        }

        for (Agent agent : agents) {
            ((CommunicatingAgent) agent).setCommunicator(createCommunicator(agent), agentNames);
        }

        getProblem().setAgents(agents);

    }

    private Communicator createCommunicator(Entity entity) {
        InboxBasedCommunicator communicator = new InboxBasedCommunicator(entity.getName());

        try {
            communicator.setChannel(new ConcurrentProcessCommunicationChannel(communicator, getConcurrentSimulation(), receiverTable, inboxCounters));
        } catch (CommunicationChannelException e) {
            e.printStackTrace();
        }

        return communicator;
    }
}

package cz.agents.alite.communication.protocol.asynchronousbacktracking;

import cz.agents.deconfliction.trajectory.CandidateSolution;


public class NogoodMessage {
    CandidateSolution nogood;
    String sender;

    public NogoodMessage(String sender, CandidateSolution nogood) {
        this.nogood = nogood;
        this.sender = sender;
    }

    public CandidateSolution getNogood() {
        return nogood;
    }

    public String getSender() {
        return sender;
    }
}

package cz.agents.admap.msg;

import cz.agents.alite.communication.content.Content;

public class InformNewRound extends Content {
    final int roundNo;

    public InformNewRound(int roundNo) {
        super(null);
        this.roundNo = roundNo;
    }

    public int getRoundNo() {
		return roundNo;
	}
    
	@Override
	public String toString() {
		return "InformNewRound [roundNo=" + roundNo + "]";
	}
    
}

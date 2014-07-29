package cz.agents.admap.msg;

import cz.agents.alite.communication.content.Content;

public class InformSuccessfulConvergence extends Content {
	
	public InformSuccessfulConvergence() {
		super(null);
	}

	@Override
	public String toString() {
		return "InformGloballyConverged []";
	}
	
}

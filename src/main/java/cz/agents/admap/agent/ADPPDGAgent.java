package cz.agents.admap.agent;

import java.util.Collection;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;

import tt.euclid2i.EvaluatedTrajectory;
import tt.euclid2i.Point;
import tt.euclid2i.Trajectory;
import tt.euclid2i.probleminstance.Environment;
import tt.euclidtime3i.region.MovingCircle;
import cz.agents.admap.msg.InformNewTrajectory;
import cz.agents.alite.communication.Message;

public class ADPPDGAgent extends Agent {

    Map<String, Objectives> group = new HashMap<String, Objectives>();
    Map<String, Trajectory> trajectories =  new HashMap<String, Trajectory>();
    Map<String, MovingCircle> avoids =  new HashMap<String, MovingCircle>();

    public ADPPDGAgent(String name, Point start, Point goal, Environment environment, int agentBodyRadius) {
        super(name, start, goal, environment, agentBodyRadius);
        this.group.put(name, new Objectives(start, goal));
    }

    @Override
    public EvaluatedTrajectory getCurrentTrajectory() {
        return trajectory;
    }

    @Override
    public void start() {
        replan();
    }

    private void replan() {

        // compute best response

        Collection<tt.euclidtime3i.Region> avoidRegions = new LinkedList<tt.euclidtime3i.Region>();
        for (MovingCircle movingCircle : avoids.values()) {
            avoidRegions.add(new MovingCircle(movingCircle.getTrajectory(), movingCircle.getRadius() + agentBodyRadius));
        }

        trajectory = Util.computeBestResponse(start, goal, inflatedObstacles, environment.getBounds(), avoidRegions);

        // broadcast to the others

        broadcast(new InformNewTrajectory(getName(), new MovingCircle(getCurrentTrajectory(), agentBodyRadius)));
    }



    @Override
    protected void notify(Message message) {
        super.notify(message);
        if (message.getContent() instanceof InformNewTrajectory) {
            InformNewTrajectory newTrajectoryMessage = (InformNewTrajectory) (message.getContent());
            String agentName = newTrajectoryMessage.getAgentName();
            MovingCircle occupiedRegion = (MovingCircle) newTrajectoryMessage.getRegion();

            if (agentName.compareTo(getName()) < 0) {
                avoids.put(agentName, occupiedRegion);
                replan();
            }
        }
    }


}

package cz.agents.admap.agent;

import tt.euclid2i.EvaluatedTrajectory;
import tt.euclid2i.Point;
import tt.euclid2i.probleminstance.Environment;
import tt.euclidtime3i.region.MovingCircle;
import cz.agents.admap.agent.adopt.Bounds;
import cz.agents.admap.agent.adopt.Constraint;
import cz.agents.admap.agent.adopt.Context;
import cz.agents.admap.agent.adopt.LocalCost;
import cz.agents.admap.msg.InformNewTrajectory;
import cz.agents.alite.communication.Message;

public class ADOPTAgent extends Agent {

    Context context;
    Bounds valueBounds;
    double threshold;
    Constraint constraint;

    LocalCost localCost = new LocalCost() {

        @Override
        public double getCost(EvaluatedTrajectory value) {
            return computeLocalCost(value);
        }
    };

    public ADOPTAgent(String name, Point start, Point goal, Environment environment, int agentBodyRadius, Constraint constraint) {
        super(name, start, goal, environment, agentBodyRadius);
        context = new Context();
        valueBounds = new Bounds();
        threshold = 0;
        this.constraint = constraint;
    }

    double computeLocalCost(EvaluatedTrajectory value) {
        double cost = value.getCost();

        for (String agent : context.vars()) {
            cost += constraint.getCost(getOccupiedRegion(), context.get(agent));
        }

        return cost;
    }

    private EvaluatedTrajectory findMinimumLowerBoundTrajectory() {
        return Util.computeBestResponse(start, goal, inflatedObstacles, environment.getBounds(), context.getOccupiedRegions(agentBodyRadius));
    }

    private void setValue(EvaluatedTrajectory traj) {
        context.put(getName(), new MovingCircle(traj, agentBodyRadius));
    }

    private EvaluatedTrajectory getValue() {
        if (context.contains(getName())) {
            return (EvaluatedTrajectory)((MovingCircle) context.get(getName())).getTrajectory();
        } else {
            return null;
        }
    }

    @Override
    public EvaluatedTrajectory getCurrentTrajectory() {
        return getValue();
    }

    @Override
    public void start() {
        setValue(findMinimumLowerBoundTrajectory());
        backtrack();
    }

    private void backtrack() {

        if (threshold == valueBounds.getUpperBound(localCost)) {
            // we have found the optimum
        } else if (valueBounds.getLowerBoundOf(localCost, getValue()) > threshold) {
            // switch to a different value
        }

        // broadcast to lower-priority
        broadcast(new InformNewTrajectory(getName(), new MovingCircle(getValue(), agentBodyRadius)));
    }

    @Override
    protected void notify(Message message) {
        super.notify(message);
        if (message.getContent() instanceof InformNewTrajectory) {
          InformNewTrajectory newTrajectoryMessage = (InformNewTrajectory) (message.getContent());
          String agentName = newTrajectoryMessage.getAgentName();
          MovingCircle occupiedRegion = (MovingCircle) newTrajectoryMessage.getRegion();

          if (agentName.compareTo(getName()) < 0) {
              // it is a message from a pseudo-parent

              context.put(agentName, occupiedRegion);

              // we're in a new context -- reset bounds
              valueBounds.resetAfterContextChange(context);

              maintainThresholdInvariant();

              backtrack();
          }
        }
    }

    private void maintainThresholdInvariant() {
        // TODO
    }


}

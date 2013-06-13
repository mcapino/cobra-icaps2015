package cz.agents.admap.agent;

import tt.euclid2i.EvaluatedTrajectory;
import tt.euclid2i.Point;
import tt.euclid2i.probleminstance.Environment;
import tt.euclidtime3i.region.MovingCircle;
import cz.agents.admap.agent.adopt.Constraint;
import cz.agents.admap.agent.adopt.Context;
import cz.agents.admap.agent.adopt.LocalCost;
import cz.agents.admap.agent.adopt.ValueBounds;
import cz.agents.admap.msg.CostMsg;
import cz.agents.admap.msg.ValueMsg;
import cz.agents.alite.communication.Message;

public class ADOPTAgent extends Agent {

    Context context;
    ValueBounds valueBounds;
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
        valueBounds = new ValueBounds();
        threshold = 0;
        this.constraint = constraint;
    }

    double computeLocalCost(EvaluatedTrajectory value) {
        double cost = value.getCost();

        for (String agent : context.vars()) {
            if (!agent.equals(getName())) {
                cost += constraint.getCost(getOccupiedRegion(), context.get(agent));
            }
        }

        return cost;
    }

    private EvaluatedTrajectory findMinimumLowerBoundTrajectory() {
        return Util.computeBestResponse(start, goal, inflatedObstacles, environment.getBounds(), context.getOccupiedRegions(agentBodyRadius));
    }

    private void setValue(EvaluatedTrajectory traj) {
        context.put(getName(), new MovingCircle(traj, agentBodyRadius));
        valueBounds.introduceNewValue(traj);
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

        if (threshold == ub()) {
            // we have found the optimum
            // TODO
        } else if (lb(getValue()) > threshold) {
            // switch to a different value
            // TODO
            System.out.println(getName() + " BACKTRACKING...");
        }

        // broadcast to lower-priority
        broadcast(new ValueMsg(getName(), new MovingCircle(getValue(), agentBodyRadius)));

        // TODO maintain allocation invariant
        maintainAllocationInvariant();


        if (threshold == ub()) {
            //terminated
        }

        // broadcast cost
        broadcast(new CostMsg(getName(), new MovingCircle(getValue(), agentBodyRadius), context, lb(), ub()));

    }

    private void maintainAllocationInvariant() {
        // TODO still to be implemented...
    }

    private double ub(EvaluatedTrajectory value) {
        return valueBounds.getUpperBoundOf(localCost, value);
    }

    private double lb(EvaluatedTrajectory value) {
        return valueBounds.getLowerBoundOf(localCost, value);
    }

    private double ub() {
        return valueBounds.getUpperBound(localCost);
    }

    private double lb() {
        return valueBounds.getLowerBound(localCost);
    }

    @Override
    protected void notify(Message message) {
        super.notify(message);

        if (message.getContent() instanceof ValueMsg) {
            handleInformMessage((ValueMsg) (message.getContent()));
        }

        if (message.getContent() instanceof CostMsg) {
            handleCostMessage((CostMsg) message.getContent());
        }
    }

    private void handleInformMessage(ValueMsg valueMessage) {
        String agentName = valueMessage.getAgentName();
          MovingCircle occupiedRegion = (MovingCircle) valueMessage.getRegion();

          // Ignore messages from ancestors
          if (agentName.compareTo(getName()) < 0) {

              System.out.println(getName()+": processing " + valueMessage);

              context.put(agentName, occupiedRegion);

              // we're in a new context -- reset inconsistent bounds
              valueBounds.resetAfterContextChange(context);

              maintainThresholdInvariant();

              backtrack();
          }
    }

    private void handleCostMessage(CostMsg costMessage) {
        String agentName = costMessage.getAgentName();
        Context msgcontext = costMessage.getContext();
        msgcontext.removeVar(getName());

        // ignore costs of lower priority agents
        if (agentName.compareTo(getName()) > 0) {

            System.out.println(getName()+": processing " + costMessage);

            // add variables not in my context
            for (String varName : msgcontext.vars()) {
                // TODO if not my neighbor ...
                context.put(varName, msgcontext.get(varName));
            }
            valueBounds.resetAfterContextChange(context);

            if (context.compatibleWith(msgcontext)) {
                valueBounds.set(getValue(),
                        costMessage.getAgentName(),
                        costMessage.getLb(),
                        costMessage.getUb(),
                        0 /*threshold*/,
                        costMessage.getContext());
            }

            maintainChildThresholdInvariant();
            maintainThresholdInvariant();
        }
    }

    private void maintainChildThresholdInvariant() {
        // TODO Still to be implemented...
    }

    private void maintainThresholdInvariant() {
        if (threshold > ub()) {
            threshold = ub();
        }

        if (threshold < lb()) {
            threshold = lb();
        }
    }

    @Override
    public String toString() {
        return String.format("ADOPTAgent %s lb: %.2f ub: %.2f t: %.2f values explored: %d",
                getName(),
                lb(),
                ub(),
                threshold,
                valueBounds.size());
    }

    @Override
    public String getStatus() {
        String s = String.format("%s c: %.2f \n lb: %.2f ub: %.2f \n t: %.2f val. expl.: %d",
                getName(),
                getCurrentTrajectory() != null ? getCurrentTrajectory().getCost() : Double.POSITIVE_INFINITY,
                lb(),
                ub(),
                threshold,
                valueBounds.size());

        s += "\n" + valueBounds.toString();
        return s;
    }



}

package cz.agents.admap.agent;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.Random;
import java.util.Set;

import tt.euclid2i.EvaluatedTrajectory;
import tt.euclid2i.Point;
import tt.euclid2i.probleminstance.Environment;
import tt.euclidtime3i.region.MovingCircle;
import cz.agents.admap.agent.adopt.Constraint;
import cz.agents.admap.agent.adopt.Context;
import cz.agents.admap.agent.adopt.FeasibleTrajectoriesDomain;
import cz.agents.admap.agent.adopt.LocalCost;
import cz.agents.admap.agent.adopt.ValueBounds;
import cz.agents.admap.msg.CostMsg;
import cz.agents.admap.msg.ValueMsg;
import cz.agents.alite.communication.Message;

public class ADOPTAgent extends Agent {

    Context context;
    ValueBounds valueBounds;
    double backtrackThreshold;
    double newValueThreshold;
    Constraint constraint;
    
    static final int MAXTIME = 10000;
    static final int WAITMOVEDURATION = 10;

    LocalCost localCost = new LocalCost() {
        @Override
        public double getCost(EvaluatedTrajectory value) {
            return computeLocalCost(value);
        }
    };

    private FeasibleTrajectoriesDomain domain;
    private Random random = new Random(1);
	EvaluatedTrajectory trajectory;

    public ADOPTAgent(String name, Point start, Point goal,
            Environment environment, int agentBodyRadius, Constraint constraint) {
        super(name, start, goal, environment, agentBodyRadius);
        context = new Context();
        valueBounds = new ValueBounds();
        backtrackThreshold = 0;
        this.constraint = constraint;
        domain = new FeasibleTrajectoriesDomain(start, goal, inflatedObstacles, new LinkedList<tt.euclidtime3i.Region>(), environment.getBoundary().getBoundingBox(), MAXTIME, WAITMOVEDURATION, random);
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

    private void setValue(EvaluatedTrajectory traj) {
        context.put(getName(), new MovingCircle(traj, agentBodyRadius));
        valueBounds.introduceNewValue(traj);
    }

    private EvaluatedTrajectory getValue() {
        if (context.contains(getName())) {
            return (EvaluatedTrajectory) ((MovingCircle) context.get(getName())).getTrajectory();
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
        valueBounds.updateChildren(getChildren());
        setValue(domain.getNewTrajectory(Double.POSITIVE_INFINITY));
        backtrack();
    }

    private void backtrack() {
//        if (valueBounds.isEmpty()) {
//            // the domain is empty, take best response
//            System.out.println(getName() + " CHOSING BEST-RESPONSE VALUE...");
//            setValue(findMinimumLowerBoundTrajectory());
//            threshold = lb();
//            System.out.println(getStatus());
//        } else
        if (backtrackThreshold == ub()) {
            // we have found the optimum
            // TODO
        }

        if (lb() > newValueThreshold) {
            // introduce a new value
            EvaluatedTrajectory traj = domain.getNewTrajectory(newValueThreshold);
            System.out.println(getName() + " Introduced new value: " + Integer.toHexString(traj.hashCode()));
            valueBounds.introduceNewValue(traj);
        }

        if (lb(getValue()) > backtrackThreshold) {

            System.out.println(getName() + " Backtrack not implemented :-(");

            // Chose the best (w.r.t. lower-bound) value from the existing ones

            //System.out.println(getName() + " Switching to a new value: " + Integer.toHexString(traj.hashCode()));

            //EvaluatedTrajectory traj = valueBounds.findLowestBoundValue();

            //setValue(traj);
            //System.out.println(getStatus());

            //broadcast(new ValueMsg(getName(), new MovingCircle(getValue(), agentBodyRadius)));

            //try { Thread.sleep(5000); } catch (InterruptedException e) {}
        }

        broadcast(new CostMsg(getName(), new MovingCircle(getValue(), agentBodyRadius), new Context(context), lb(), ub()));

        /*
        // broadcast to lower-priority
        broadcast(new ValueMsg(getName(), new MovingCircle(getValue(), agentBodyRadius)));

        // TODO maintain allocation invariant
        maintainAllocationInvariant();

        if (threshold == ub()) {
            // terminated
        }

        // broadcast cost
        broadcast(new CostMsg(getName(), new MovingCircle(getValue(), agentBodyRadius), new Context(context), lb(), ub()));
        */

    }

    private double ub(EvaluatedTrajectory value) {
        return valueBounds.getUpperBoundOf(localCost, value);
    }

    private double lb(EvaluatedTrajectory value) {
        return valueBounds.getLowerBoundOf(localCost, value);
    }

    private double ub() {
        if (domain.isEmpty()) {
            // there are no consistent trajectories
            return Double.POSITIVE_INFINITY;
        }
        else if (valueBounds.isEmpty()) {
            // there are no children
            return domain.getShortestTrajCost();
        } else {
            return valueBounds.getUpperBound(localCost);
        }
    }

    private double lb() {
        if (domain.isEmpty()) {
            // there are no consistent trajectories
            return Double.POSITIVE_INFINITY;
        }
        else if (valueBounds.isEmpty()) {
            // there are no children
            return domain.getShortestTrajCost();
        } else {
            return valueBounds.getLowerBound(localCost);
        }
    }

    @Override
    protected void notify(Message message) {
        super.notify(message);

        // update the collection of known children

        if (message.getContent() instanceof ValueMsg) {
            handleValueMessage((ValueMsg) (message.getContent()));
        }

        if (message.getContent() instanceof CostMsg) {
            handleCostMessage((CostMsg) message.getContent());
        }
    }

    private void handleValueMessage(ValueMsg valueMessage) {
        String agentName = valueMessage.getAgentName();
        MovingCircle occupiedRegion = (MovingCircle) valueMessage.getRegion();

        // Ignore messages from ancestors
        if (agentName.compareTo(getName()) < 0) {

            System.out.println(getName() + ": processing " + valueMessage);

            context.put(agentName, occupiedRegion);

            // we're in a new context -- reset inconsistent bounds
            // TOO SOPHISTICATED, DISABLING FOR A WHILE
            //valueBounds.resetAfterContextChange(context);

            valueBounds = new ValueBounds();
            valueBounds.updateChildren(getChildren());

            domain = new FeasibleTrajectoriesDomain(start, goal, inflatedObstacles, 
            		context.getOccupiedRegions(getName(), agentBodyRadius+2), 
            		environment.getBoundary().getBoundingBox(), 
            		MAXTIME, WAITMOVEDURATION, random);

            maintainThresholdInvariant();

            backtrack();
        }
    }

    private void handleCostMessage(CostMsg costMessage) {
        String agentName = costMessage.getAgentName();
        Context msgcontext = costMessage.getContext();
        msgcontext.removeVar(getName());

        // ignore costs from other agents than children
        if (getChildren().contains(agentName)) {

            System.out.println(getName() + ": processing " + costMessage);

            // add variables not in my context
            for (String varName : msgcontext.vars()) {
                // TODO if not my neighbor ... parent and children are neighbors
                if (!getNeighbors().contains(varName)) {
                    context.put(varName, msgcontext.get(varName));
                }
            }

            // TODO disabled
            valueBounds.resetAfterContextChange(context);

            if (context.compatibleWith(msgcontext)) {
                valueBounds.set(getValue(), costMessage.getAgentName(),
                        costMessage.getLb(), costMessage.getUb(),
                        0 /* threshold */, costMessage.getContext());
            }

            maintainChildThresholdInvariant();
            maintainThresholdInvariant();
            backtrack();
        }
    }

    private void maintainChildThresholdInvariant() {
        // TODO Still to be implemented...
    }

    private void maintainThresholdInvariant() {
          backtrackThreshold = 99;
//        if (threshold < lb()) {
//            threshold = lb();
//        }
//
//        if (threshold > ub()) {
//            threshold = ub();
//        }
    }

    private void maintainAllocationInvariant() {
        // TODO still to be implemented...
    }

    @Override
    public String toString() {
        return String.format(
                "ADOPTAgent %s lb: %.2f ub: %.2f t: %.2f values explored: %d",
                getName(), lb(), ub(), backtrackThreshold, valueBounds.size());
    }

    @Override
    public String getStatus() {
        String s = String
                .format("%s traj: %s c: %.2f \n lb: %.2f ub: %.2f \n t: %.2f \n context: %s \n children: %s",
                        getName(),
                        getCurrentTrajectory() != null ? Integer.toHexString(getCurrentTrajectory().hashCode()) : null,
                        getCurrentTrajectory() != null ? getCurrentTrajectory().getCost() : Double.POSITIVE_INFINITY,
                        lb(),
                        ub(),
                        backtrackThreshold,
                        context,
                        getChildren().toString());

        s += "\nvalues:\n" + valueBounds.toString(localCost);
        return s;
    }

    public Set<String> getChildren() {
        String[] agentsArray = agents.toArray(new String[agents.size()]);
        for (int i = 0; i < agentsArray.length; i++) {
            if (agentsArray[i].compareTo(getName()) > 0) {
                Set<String> children = new HashSet<String>();
                children.add(agentsArray[i]);
                return children;
            }
        }
        return new HashSet<String>();
    }

    public String getParent() {
        String[] agentsArray = agents.toArray(new String[agents.size()]);
        for (int i = agentsArray.length-1; i >= 0; i--) {
            if (agentsArray[i].compareTo(getName()) < 0) {
                return agentsArray[i];
            }
        }
        return null;
    }

    public Set<String> getNeighbors() {
        Set<String> neighbors =  new HashSet<String>(getChildren());
        neighbors.add(getParent());
        return neighbors;
    }

	@Override
	public boolean isGlobalTerminationDetected() {
		return false;
	}
	
	@Override
	public boolean hasSucceeded() {
		return true;
	}
}

package cz.agents.alite.common.event;

import org.junit.Test;

import cz.agents.alite.simulation.ConcurrentProcessSimulation;

public class ConcurrentProcessSimulationTest {
    int rounds = 10;
    ConcurrentProcessSimulation simulation = new ConcurrentProcessSimulation();

    class Handler1 implements DurativeEventHandler {

        @Override
        public DurativeEventProcessor getEventProcessor() {
            return simulation;
        }

        @Override
        public long handleEvent(DurativeEvent event) {

            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            if (rounds > 0) {
                simulation.addEvent(event.getTime() + simulation.getCurrentEventHandlingDurationNanos(), "p2", new Handler2());
                rounds--;
            }

            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }


            return COUNT_SYSTEM_NANOS;
        }

    }

    class Handler2 implements DurativeEventHandler {

        @Override
        public DurativeEventProcessor getEventProcessor() {
            return simulation;
        }

        @Override
        public long handleEvent(DurativeEvent event) {

            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            simulation.addEvent(event.getTime() + simulation.getCurrentEventHandlingDurationNanos(), "p1", new Handler1());
            return COUNT_SYSTEM_NANOS;
        }

    }


    @Test
    public void test() {
        simulation.addEvent(0, "p1", new Handler1());
        simulation.setPrintouts(1);
        simulation.run();

    }

}

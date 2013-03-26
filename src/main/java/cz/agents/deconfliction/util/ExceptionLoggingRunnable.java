package cz.agents.deconfliction.util;

import org.apache.log4j.Level;
import org.apache.log4j.Logger;

class ExceptionLoggingRunnable implements Runnable {

    private static final Logger LOGGER = Logger.getLogger(ExceptionLoggingRunnable.class);

    Runnable runnable;

    public ExceptionLoggingRunnable(Runnable runnable) {
        this.runnable = runnable;
    }

    @Override
    public void run() {
        try {
            runnable.run();
        } catch (Throwable t) {
            LOGGER.log(Level.FATAL, "ExceptionLoggingRunnable", t);
        }
    }

}

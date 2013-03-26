package cz.agents.deconfliction.util;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

public class SerialCallQueue {

    final ExecutorService executorService;

    public SerialCallQueue() {
        this.executorService = Executors.newFixedThreadPool(1);
    }

    public Future<?> addCall(Runnable runnable) {
        return executorService.submit(new ExceptionLoggingRunnable(runnable));
    }

}

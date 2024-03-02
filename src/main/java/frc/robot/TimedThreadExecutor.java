package frc.robot;

import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

public class TimedThreadExecutor {
    private final Runnable task;
    private final long timeout;
    private final TimeUnit timeUnit;


    public TimedThreadExecutor(Runnable task, long timeout, TimeUnit timeUnit) {
        this.task = task;
        this.timeout = timeout;
        this.timeUnit = timeUnit;
    }

    public void run() {
        ExecutorService executor = Executors.newSingleThreadExecutor();

        Future<?> future = executor.submit(task);

        try {
            future.get(timeout, timeUnit);
        } catch (TimeoutException e) {
            future.cancel(true); // Interrupts the thread if it's still running
        } catch (InterruptedException | ExecutionException e) { } finally {
            executor.shutdownNow();
        }
    }

    public void runImmediate() {
        task.run();
    }
}

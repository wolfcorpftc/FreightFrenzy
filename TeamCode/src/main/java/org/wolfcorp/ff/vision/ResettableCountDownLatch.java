package org.wolfcorp.ff.vision;

import java.util.concurrent.CountDownLatch;
import java.util.concurrent.atomic.AtomicReference;

// https://stackoverflow.com/a/40288284
public class ResettableCountDownLatch {

    private final int initialCount;
    private final AtomicReference<CountDownLatch> latchHolder = new AtomicReference<>();

    public ResettableCountDownLatch(int count) {
        initialCount = count;
        latchHolder.set(new CountDownLatch(count));
    }

    public void reset() {
        // obtaining a local reference for modifying the required latch
        final CountDownLatch oldLatch = latchHolder.getAndSet(new CountDownLatch(initialCount));
        if (oldLatch != null) {
            // checking the count each time to prevent unnecessary countdowns due to parallel countdowns
            while (oldLatch.getCount() > 0) {
                oldLatch.countDown();
            }
        }
    }

    public void countDown() {
        latchHolder.get().countDown();
    }

    public void await() throws InterruptedException {
        latchHolder.get().await();
    }
}
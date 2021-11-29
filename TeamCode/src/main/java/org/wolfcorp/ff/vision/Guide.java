package org.wolfcorp.ff.vision;

public interface Guide {
    Freight getTarget();
    void setTarget(Freight f);
    public PolarPoint navigate() throws InterruptedException;
    void start();
    void stop();
}

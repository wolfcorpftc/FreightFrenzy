package org.wolfcorp.ff.vision;

public interface Guide {
    Freight getTarget();
    void setTarget(Freight f);
    PolarPoint navigate() throws InterruptedException;
    PolarPoint getLastNavigation();
    void start();
    void stop();
}

package org.wolfcorp.ff.vision;

public class PolarPoint {
    public final double r;
    public final double theta;

    public PolarPoint(double r, double theta) {
        this.r = r;
        this.theta = theta;
    }

    public PolarPoint() {
        this(0, 0);
    }

    public double getRadius() {
        return r;
    }

    public double getAngle() {
        return theta;
    }
}

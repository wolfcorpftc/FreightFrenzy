package org.wolfcorp.ff.opmode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class TimedController {
    private final ElapsedTime timer = new ElapsedTime();
    private final double cap;
    private final double initialValue;
    private final double rate;

    public TimedController(double rate, double initialValue, double cap) {
        this.rate = rate;
        this.initialValue = initialValue;
        this.cap = cap;
    }
    
    public void start() {
        timer.reset();
    }
    
    public double update() {
        return Math.min(timer.seconds() * rate + initialValue, cap);
    }
}

package org.wolfcorp.ff.opmode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class TimedController {
    private final ElapsedTime timer = new ElapsedTime();
    private final double goal;
    private final double initialValue;
    private final double rate;

    public TimedController(double rate, double initialValue, double goal) {
        this.rate = rate;
        this.initialValue = initialValue;
        this.goal = goal;
    }
    
    public void reset() {
        timer.reset();
    }
    
    public double update() {
        double vel = timer.seconds() * rate + initialValue;
        if (rate > 0) {
            return Math.min(vel, goal);
        } else {
            return Math.max(vel, goal);
        }
    }
}

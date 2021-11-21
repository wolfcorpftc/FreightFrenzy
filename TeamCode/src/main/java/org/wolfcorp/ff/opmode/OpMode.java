package org.wolfcorp.ff.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class OpMode extends LinearOpMode {
    static private OpMode instance = null;

    public OpMode() {
        instance = this;

        Match.isRed = this.getClass().getSimpleName().contains("Red");

        // Faster telemetry
        telemetry.setMsTransmissionInterval(50);
    }

    static public void log(String s) {
        if (instance == null) {
            return;
        }
        instance.telemetry.addLine(s);
        instance.telemetry.update();
    }

    protected void resetInstance() {
        instance = null;
    }
}

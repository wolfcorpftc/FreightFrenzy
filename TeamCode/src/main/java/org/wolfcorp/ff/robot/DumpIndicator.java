package org.wolfcorp.ff.robot;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.wolfcorp.ff.opmode.OpMode;

import java.util.Arrays;

public class DumpIndicator {
    // TODO: {a,b,c,d}{Green,Red}
    private final LED aGreen;
    private final LED aRed;
    private final LED bGreen;
    private final LED bRed;
    private final LED[] reds;
    private final LED[] greens;
    private final Object lock = new Object();

    public enum Status {
        EMPTY,
        FULL,
        OVERFLOW
    }

    public DumpIndicator(HardwareMap hardwareMap) {
        aGreen = hardwareMap.get(LED.class, "aGreen");
        aRed = hardwareMap.get(LED.class, "aRed");
        bGreen = hardwareMap.get(LED.class, "bGreen");
        bRed = hardwareMap.get(LED.class, "bRed");

        reds = new LED[]{aRed, bRed};
        greens = new LED[]{aGreen, bGreen};

        empty();
    }

    public void enable(boolean green, boolean red) {
        synchronized (lock) {
            for (LED led : greens) {
                led.enable(green);
            }
            for (LED led : reds) {
                led.enable(red);
            }
        }
    }

    /**
     * Sets the LEDs to orange because the dump is empty.
     */
    public void empty() {
        enable(true, true);
    }

    /**
     * Sets the LEDs to red because the dump is overflowing (2 elements).
     */
    public void overflow() {
        enable(true, false);
    }

    /**
     * Sets the LEDs to green because the dump is full (ready to deliver).
     */
    public void full() {
        enable(false, true);
    }

    /**
     * Updates the dump indicator based on distance sensor readings.
     *
     * @return the status of the dump
     */
    public Status update() {
        boolean overflow = OpMode.upperDumpDistance.getDistance(DistanceUnit.INCH) < Outtake.DUMP_OVERFLOW_DIST;
        boolean full = OpMode.lowerDumpDistance.getDistance(DistanceUnit.INCH) < Outtake.DUMP_FULL_DIST;

        if (full && overflow) {
            overflow();
            return Status.OVERFLOW;
        } else if (full) {
            full();
            return Status.FULL;
        } else {
            empty();
            return Status.EMPTY;
        }
    }
}

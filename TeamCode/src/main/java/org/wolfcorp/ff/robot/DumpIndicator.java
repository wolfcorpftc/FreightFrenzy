package org.wolfcorp.ff.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.wolfcorp.ff.opmode.OpMode;

public class DumpIndicator {
    private final LED leftGreen;
    private final LED rightGreen;
    private final LED leftRed;
    private final LED rightRed;
    private final Object lock = new Object();

    public enum Status {
        EMPTY,
        AFLOAT,
        FULL,
        OVERFLOW,
    }

    public DumpIndicator(HardwareMap hardwareMap) {
        leftGreen = hardwareMap.get(LED.class, "leftGreen");
        rightGreen = hardwareMap.get(LED.class, "rightGreen");
        leftRed = hardwareMap.get(LED.class, "leftRed");
        rightRed = hardwareMap.get(LED.class, "rightRed");
        empty();
    }

    /**
     * Sets the LEDs to orange because the dump is empty.
     */
    public void empty() {
        synchronized (lock) {
            leftGreen.enable(true);
            rightGreen.enable(true);
            leftRed.enable(true);
            rightRed.enable(true);
        }
    }

    /**
     * Sets the LEDs to red because the dump is overflowing (2 elements).
     */
    public void overflow() {
        synchronized (lock) {
            leftGreen.enable(true);
            rightGreen.enable(true);
            leftRed.enable(false);
            rightRed.enable(false);
        }
    }

    /**
     * Sets the LEDs to green because the dump is full (ready to deliver).
     */
    public void full() {
        synchronized (lock) {
            leftGreen.enable(false);
            rightGreen.enable(false);
            leftRed.enable(true);
            rightRed.enable(true);
        }
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
        } else if (overflow) {
            overflow();
            return Status.AFLOAT;
        } else if (full) {
            full();
            return Status.FULL;
        } else {
            empty();
            return Status.EMPTY;
        }
    }
}

package org.wolfcorp.ff.robot;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.wolfcorp.ff.opmode.OpMode;

public class DumpIndicator {
    private final LED leftGreen;
    private final LED rightGreen;
    private final LED leftRed;
    private final LED rightRed;

    public enum Status {
        EMPTY,
        FULL,
        OVERFLOW
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
        leftGreen.enable(true);
        rightGreen.enable(true);
        leftRed.enable(true);
        rightRed.enable(true);
    }

    /**
     * Sets the LEDs to red because the dump is overflowing (2 elements).
     */
    public void overflow() {
        leftGreen.enable(true);
        rightGreen.enable(true);
        leftRed.enable(false);
        rightRed.enable(false);
    }

    /**
     * Sets the LEDs to green because the dump is full (ready to deliver).
     */
    public void full() {
        leftGreen.enable(false);
        rightGreen.enable(false);
        leftRed.enable(true);
        rightRed.enable(true);
    }

    /**
     * Updates the dump indicator based on distance sensor readings.
     *
     * @return the status of the dump
     */
    public Status update() {
        if (hasFreight(OpMode.upperDumpDistance)) {
            overflow();
            return Status.OVERFLOW;
        } else if (hasFreight(OpMode.lowerDumpDistance)) {
            full();
            return Status.FULL;
        } else {
            empty();
            return Status.EMPTY;
        }
    }

    public boolean hasFreight(RevColorSensorV3 sensor) {
        if (sensor == OpMode.upperDumpDistance &&
            OpMode.upperDumpDistance.getDistance(DistanceUnit.INCH) > Outtake.DUMP_OVERFLOW_DIST) {
            return false;
        } else if (sensor == OpMode.lowerDumpDistance &&
            OpMode.lowerDumpDistance.getDistance(DistanceUnit.INCH) > Outtake.DUMP_OVERFLOW_DIST) {
            return false;
        }

        /*if (sensor.red() > 2000 && sensor.blue() > 2000 && sensor.green() > 2000) {
            return false;
        }*/

        return true;
    }
}

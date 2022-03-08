package org.wolfcorp.ff.robot.util;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class InchSensor {
    private final DistanceSensor sensor;

    public InchSensor(DistanceSensor s) {
        sensor = s;
    }
    public double get() {
        return sensor.getDistance(DistanceUnit.INCH);
    }

    public double getDistance(DistanceUnit unit) {
        return sensor.getDistance(unit);
    }
}

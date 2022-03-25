package org.wolfcorp.ff.robot.util;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class InchSensor {
    private DistanceSensor sensor;
    private ModernRoboticsI2cRangeSensor ultrasonic;


    public InchSensor(DistanceSensor s) {
        sensor = s;
    }
    public InchSensor(ModernRoboticsI2cRangeSensor s) { ultrasonic = s; }
    public double get(DistanceUnit unit) {
        if (sensor != null)
            return sensor.getDistance(unit);
        else if (ultrasonic != null)
            return ultrasonic.getDistance(unit);
        else
            return 300; // HACK
    }

    public double get() {
        return get(DistanceUnit.INCH);
    }

    public double getDistance(DistanceUnit unit) {
        return get(unit);
    }
}

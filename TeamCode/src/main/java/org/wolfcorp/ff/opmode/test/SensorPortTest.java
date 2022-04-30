package org.wolfcorp.ff.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Sensor Port Test", group = "!test")
public class SensorPortTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DistanceSensor sensor = hardwareMap.get(DistanceSensor.class, "s");
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Distance", sensor.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }

}

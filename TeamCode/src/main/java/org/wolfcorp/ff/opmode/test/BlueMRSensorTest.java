package org.wolfcorp.ff.opmode.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.wolfcorp.ff.opmode.AutonomousMode;
import org.wolfcorp.ff.opmode.util.Match;

@TeleOp(name = "Blue MR Sensor Test", group = "!test")
public class BlueMRSensorTest extends AutonomousMode {
    double frontOffset = -0.25;
    double sideOffset = 1.75;
    @Override
    public void runOpMode() {
        Match.setupTelemetry();
        telemetry.setAutoClear(true);
        initHardware();
        Match.log("Waiting for start");
        waitForStart();

        /*
         * Old (REV sensor)
         * Control Hub (E3)
         * 0 - range sensor
         * 1 - intake dist
         * 2 - left range sensor
         * 3 - right range sensor
         *
         * New (MR sensor)
         * Control Hub (E3)
         * 0 - upper dist
         * 1 - range sensor (remove 0, add 1)
         * 2 - lower dist
         * 3 - left range sensor (remove 2, rename 3, place 3 elsewhere?)
         * Expansion Hub (E2)
         * 0 - range sensor (old)
         * 1 - intake dist
         * 2 - not plugged in
         * 3 - right range sensor (old REV sensor) NEEDS UPDATE
         */
        // Sensor snaps to nearest .155 cm
        while (opModeIsActive()) {
            telemetry.addData("Front", rangeSensor.get());
            telemetry.addData("Side", leftRangeSensor.get());
            telemetry.update();
        }
    }

    public double getCorrectedFront(){
        return(1.07863*rangeSensor.get()-0.318066+frontOffset);
    }
    public double getCorrectedSide(){
        return(1.11113*rangeSensor.get()-0.241264+sideOffset);
    }
//    public Pose2d getPos(){
//
//    }
}

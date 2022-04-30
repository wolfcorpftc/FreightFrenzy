package org.wolfcorp.ff.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.wolfcorp.ff.opmode.AutonomousMode;
import org.wolfcorp.ff.opmode.util.Match;

@Autonomous(name = "Distance Sensor Test", group = "!test")
public class DistanceSensorTest extends AutonomousMode {
    @Override
    public void runOpMode() {
        Match.setupTelemetry();
        initHardware();
        Match.log("Ready");
        waitForStart();


        telemetry.setAutoClear(true);
        while (opModeIsActive()) {
            telemetry.addData("Front", frontDist.get());
            telemetry.update();
        }
    }
}


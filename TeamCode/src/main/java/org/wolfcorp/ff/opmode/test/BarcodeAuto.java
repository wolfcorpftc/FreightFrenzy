package org.wolfcorp.ff.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.wolfcorp.ff.opmode.AutonomousMode;
import org.wolfcorp.ff.opmode.util.Match;

@Autonomous(name = "Barcode Vision Test", group = "test")
public class BarcodeAuto extends AutonomousMode {
    @Override
    public void runOpMode() {
        Match.setupTelemetry();
        initVisionWebcam();
        scanner.start();
        waitForStart();
        while (opModeIsActive()) {
            // Scanner already does the logging
            idle();
            if (Thread.interrupted()) {
                scanner.stop();
                break;
            }
        }
    }
}

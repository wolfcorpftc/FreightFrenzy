package org.wolfcorp.ff.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.wolfcorp.ff.opmode.AutonomousMode;

@Autonomous(name = "Barcode Demo", group = "test")
public class BarcodeDemo extends AutonomousMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initVision();
        scanner.start();
        while (opModeIsActive()) {
            // Scanner already does the logging
            idle();
            if (Thread.interrupted()) {
                scanner.stop();
                throw new InterruptedException();
            }
        }
    }
}

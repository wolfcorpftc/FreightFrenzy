package org.wolfcorp.ff.opmode.test;

import org.wolfcorp.ff.opmode.AutonomousMode;

public class BarcodeDemo extends AutonomousMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initVision();
        scanner.start();
        while (opModeIsActive()) {
            // Scanner already does the logging
            idle();
        }
    }
}

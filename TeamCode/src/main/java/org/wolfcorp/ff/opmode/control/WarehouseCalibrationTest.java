package org.wolfcorp.ff.opmode.control;

import org.wolfcorp.ff.opmode.AutonomousMode;
import org.wolfcorp.ff.opmode.util.Match;

public class WarehouseCalibrationTest extends AutonomousMode {
    @Override
    public void runOpMode() {
        Match.setupTelemetry();
        initHardware();
        drive.setPoseEstimate(pos(-60, 24));
        Match.status("Waiting for start");
        waitForStart();

        boolean maskCalibrate = false;
        while (opModeIsActive()) {
            drive.update();

            if (gamepad1.y && !maskCalibrate) {
                warehouseLocalization();
                maskCalibrate = true;
            }
            if (!gamepad1.y) {
                maskCalibrate = false;
            }
        }
    }
}

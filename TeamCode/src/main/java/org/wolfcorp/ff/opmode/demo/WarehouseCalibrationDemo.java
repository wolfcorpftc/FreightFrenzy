package org.wolfcorp.ff.opmode.demo;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.wolfcorp.ff.opmode.AutonomousMode;
import org.wolfcorp.ff.opmode.util.Match;

@TeleOp(name = "Warehouse Calibration Demo", group = "!demo")
public class WarehouseCalibrationDemo extends AutonomousMode {
    @Override
    public void runOpMode() {
        Match.setupTelemetry();
        initHardware();
        drive.setPoseEstimate(pos(-60, 24));
        Match.status("Waiting for start");
        waitForStart();

        boolean maskCalibrate = false;
        while (opModeIsActive()) {
            drive.updatePoseEstimate();

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

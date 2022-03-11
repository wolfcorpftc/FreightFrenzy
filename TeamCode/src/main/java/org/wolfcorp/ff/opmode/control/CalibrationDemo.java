package org.wolfcorp.ff.opmode.control;

import org.wolfcorp.ff.opmode.AutonomousMode;
import org.wolfcorp.ff.opmode.util.Match;

public class CalibrationDemo extends AutonomousMode {
    @Override
    public void runOpMode() {
        Match.setupTelemetry();
        initHardware();
        drive.setPoseEstimate(pos(0, 0));
        waitForStart();
        telemetry.setAutoClear(true);
        while (opModeIsActive()) {
            Match.status("Looping");
            localizeWarehouse();
            telemetry.addData("Pose", drive.getPoseEstimate());
            telemetry.update();
            drive.update();
        }
    }
}

package org.wolfcorp.ff.opmode.control;

import static org.wolfcorp.ff.opmode.util.Match.BLUE;
import static org.wolfcorp.ff.opmode.util.Match.RED;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.wolfcorp.ff.opmode.AutonomousMode;
import org.wolfcorp.ff.opmode.util.Match;
import org.wolfcorp.ff.robot.util.InchSensor;

@TeleOp(name = "Calibration Demo", group = "!demo")
public class CalibrationDemo extends AutonomousMode {
    @Override
    public void runOpMode() {
        waitForStart();
        while (opModeIsActive()) {
            drive.update();
            if (gamepad1.x) {
                RED = false;
                BLUE = true;
            }
            if (gamepad1.b) {
                RED = true;
                BLUE = false;
            }
            if (gamepad1.a) {
                break;
            }
        }
        while (opModeIsActive()) {
            warehouseLocalization();
            drive.update();
            if (rangeSensor.getDistance(DistanceUnit.INCH) + 6.5 > 29){
                drive.setMotorPowers(0);
                break;
            }
        }

    }
}

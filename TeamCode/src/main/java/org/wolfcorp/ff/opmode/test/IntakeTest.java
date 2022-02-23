package org.wolfcorp.ff.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.wolfcorp.ff.robot.Intake;

@TeleOp(name = "Intake Test", group = "^testing")
public class IntakeTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Intake intake = new Intake(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.y) {
                intake.in();
            } else if (gamepad1.a) {
                intake.out();
            } else if (gamepad1.x) {
                intake.off();
            }
        }
    }
}

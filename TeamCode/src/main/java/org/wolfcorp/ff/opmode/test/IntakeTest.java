package org.wolfcorp.ff.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.wolfcorp.ff.robot.Intake;
import org.wolfcorp.ff.robot.Outtake;
import org.wolfcorp.ff.vision.Barcode;

@TeleOp(name = "Intake Test", group = "^testing")
public class IntakeTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Intake intake = new Intake(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap);

        boolean mask = true;

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.y) {
                intake.in();
            } else if (gamepad1.a) {
                intake.out();
            } else if (gamepad1.x) {
                intake.off();
            } else if (gamepad1.b && mask) {
                mask = false;
                outtake.getDumpServo().setPosition(0.7);
                sleep(100);
                outtake.slideToAsync(Barcode.TOP);
                sleep(100);
                outtake.dumpOut();
                outtake.waitForSlide();
                sleep(500);
                outtake.dumpDrop();
                sleep(1000);
//            outtake.dumpOut();
                sleep(500);
                outtake.dumpIn();
                outtake.slideToAsync(Barcode.ZERO);
                outtake.waitForSlide();
                sleep(2000);
            } else if (!gamepad1.b) {
                mask = true;
            }
        }
    }
}

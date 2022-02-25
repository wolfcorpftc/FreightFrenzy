package org.wolfcorp.ff.opmode.test;

import static org.wolfcorp.ff.robot.Outtake.DUMP_OUT_TOP_POSITION;
import static org.wolfcorp.ff.robot.Outtake.PIVOT_OUT_TOP_POSITION;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.wolfcorp.ff.robot.Intake;
import org.wolfcorp.ff.robot.Outtake;
import org.wolfcorp.ff.vision.Barcode;

@TeleOp(name = "Intake Test", group = "!!testing")
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
                outtake.getDump().setPosition(0.7);
                sleep(100);
                outtake.slideToAsync(Barcode.TOP);
                sleep(100);
                outtake.getPivot().setPosition(PIVOT_OUT_TOP_POSITION);
                outtake.getDump().setPosition(DUMP_OUT_TOP_POSITION);
                outtake.waitForSlide();
                sleep(500);
                outtake.drop();
                sleep(1000);
//            outtake.dumpOut();
                sleep(500);
                // FIXME
//                outtake.dumpIn();
                outtake.slideToAsync(Barcode.ZERO);
                outtake.waitForSlide();
                sleep(2000);
            } else if (!gamepad1.b) {
                mask = true;
            }
        }
    }
}

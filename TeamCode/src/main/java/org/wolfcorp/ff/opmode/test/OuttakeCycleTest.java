package org.wolfcorp.ff.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.wolfcorp.ff.opmode.OpMode;
import org.wolfcorp.ff.opmode.util.Match;
import org.wolfcorp.ff.robot.Outtake;
import org.wolfcorp.ff.vision.Barcode;

@TeleOp(name = "Outtake Cycle Test", group = "^testing")
public class OuttakeCycleTest extends OpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Match.setupTelemetry();
        Outtake outtake = new Outtake(hardwareMap);
        waitForStart();
//        outtake.cycleAsync(Barcode.TOP).join();

        // OUT
//        outtake.slideToAsync(TOP);
//        outtake.dumpOut();
//        // FIXME: tune
//        OpMode.waitFor(100);
//        outtake.dumpIn();
//        OpMode.waitFor(400);
////        outtake.waitForSlide();
//
//        // DROP
//        outtake.dumpDrop();
//
//        // IN
//        outtake.slideToAsync(ZERO);
//        // FIXME: tune
//        OpMode.waitFor(2000);
//        outtake.dumpOut();
//        OpMode.waitFor(100);
//        outtake.dumpIn();

        while (opModeIsActive()) {
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
        }
    }
}

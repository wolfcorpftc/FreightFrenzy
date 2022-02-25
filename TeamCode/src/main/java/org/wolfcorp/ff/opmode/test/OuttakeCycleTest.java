package org.wolfcorp.ff.opmode.test;

import static org.wolfcorp.ff.robot.Outtake.DUMP_IN_POSITION;
import static org.wolfcorp.ff.robot.Outtake.DUMP_OUT_TOP_POSITION;
import static org.wolfcorp.ff.robot.Outtake.PIVOT_IN_POSITION;
import static org.wolfcorp.ff.robot.Outtake.PIVOT_OUT_TOP_POSITION;
import static org.wolfcorp.ff.vision.Barcode.TOP;
import static org.wolfcorp.ff.vision.Barcode.ZERO;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.wolfcorp.ff.opmode.OpMode;
import org.wolfcorp.ff.opmode.util.Match;
import org.wolfcorp.ff.robot.Outtake;

@TeleOp(name = "Outtake Cycle Test", group = "!!testing")
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
            ElapsedTime timer = new ElapsedTime();
            double time;
//            outtake.cycle(TOP);
            {
                outtake.out(TOP);
                OpMode.waitFor(600);
                outtake.drop();//1.636
                OpMode.waitFor(800);
                time = timer.seconds();
                outtake.in();
            }
            telemetry.addData("Cycling Time", time);
            telemetry.update();
            sleep(4000);
        }
    }
}

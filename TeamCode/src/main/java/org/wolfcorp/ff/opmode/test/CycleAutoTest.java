package org.wolfcorp.ff.opmode.test;

import static org.wolfcorp.ff.opmode.util.Match.RED;
import static org.wolfcorp.ff.robot.DumpIndicator.Status.EMPTY;
import static org.wolfcorp.ff.robot.DumpIndicator.Status.FULL;
import static org.wolfcorp.ff.robot.Outtake.DUMP_IN_POSITION;
import static org.wolfcorp.ff.robot.Outtake.DUMP_OUT_TOP_POSITION;
import static org.wolfcorp.ff.robot.Outtake.PIVOT_IN_POSITION;
import static org.wolfcorp.ff.robot.Outtake.PIVOT_OUT_TOP_POSITION;
import static org.wolfcorp.ff.vision.Barcode.EXCESS;
import static org.wolfcorp.ff.vision.Barcode.TOP;
import static org.wolfcorp.ff.vision.Barcode.ZERO;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.wolfcorp.ff.opmode.AutonomousMode;
import org.wolfcorp.ff.opmode.OpMode;
import org.wolfcorp.ff.opmode.util.Match;

@Autonomous(name = "Cycle Auto Test", group = "!testing")
public class CycleAutoTest extends AutonomousMode {
    ElapsedTime whToHubTimer = new ElapsedTime();
    public void runOpMode() {
        Match.setupTelemetry();
        initialPose = trueWhPose;
        initHardware();
        drive.setPoseEstimate(initialPose);
        for (int i = 1; i <= SCORING_CYCLES; i++) {
            queue(() -> {
                ElapsedTime timer = new ElapsedTime();
                intake.in();
                drive.setMotorPowers(0.2);
                while (dumpIndicator.update() == EMPTY) {
                    drive.updatePoseEstimate();
                }
                drive.setMotorPowers(0);
                intake.directedOut();
            });
            queue(whToHubTimer::reset);
            queueWarehouseSensorCalibration(trueWhPose);
            queue(fromHere()
                    .addTemporalMarker(1.0, -1.1, this::outtakeAsync)
                    .lineToConstantHeading(hubPose.vec()));
            queue(intake::off);
            queue(() -> Match.log(whToHubTimer.seconds() + " seconds"));
            queue(() -> drive.setPoseEstimate(trueHubPose));
            queue(() -> waitFor(4000));
            queue(fromHere().lineToConstantHeading(whPose.vec()));
            queueWarehouseSensorCalibration(trueWhPose);
        }
//        queue(() -> sleep(300));
        waitForStart();
        runTasks();
    }
    public void outtakeAsync() {
        Thread t = new Thread(this::outtakeCycle);
        t.start();
    }
    public void outtakeCycle() {
        outtake.getDump().setPosition(0.7);
        OpMode.waitFor(100);
        outtake.slideToAsync(TOP);
        OpMode.waitFor(100);
        outtake.getPivot().setPosition(PIVOT_OUT_TOP_POSITION);
        OpMode.waitFor(400);
        outtake.getDump().setPosition(DUMP_OUT_TOP_POSITION + 0.25);

        OpMode.waitFor(600);

        // DROP
        outtake.drop();

        OpMode.waitFor(1200);
        // IN
        outtake.getDump().setPosition(DUMP_IN_POSITION);
        OpMode.waitFor(75);
        outtake.resetSlideMode();
        outtake.slideToPositionAsync(-50);
        OpMode.waitFor(50);
        outtake.getPivot().setPosition(PIVOT_IN_POSITION);
        outtake.waitForSlide();
        outtake.resetSlideEncoder();
    }
    public void runOpModeOrig() {
        Match.setupTelemetry();
//        initialPose = pos(-72 + WIDTH / 2, 72 - 22 - LENGTH / 2);
        initialPose = trueWhPose;
        drive.setPoseEstimate(initialPose);
        initHardware();
        outtake.cycleAsync(TOP);
        waitFor(1600);

        /* cycle */ {
            for (int i = 1; i <= SCORING_CYCLES; i++) {
                Match.status("Initializing: cycle " + i);

                // *** To warehouse ***
                queue(() -> intake.in(0.8, -0.8));
                queue(from(trueHubPose).lineToConstantHeading(whPose.vec()));
                queueWarehouseSensorCalibration(trueWhPose);

                // *** Intake ***
                intake(i);
                queueWarehouseSensorCalibration(trueWhPose);

                // *** To hub ***
                queue(from(trueWhPose)
                        .addTemporalMarker(1.15, () -> {
                            // last-minute check & fix for intake
                            if (dumpIndicator.update() == FULL) {
                                intake.out();
                            } else if (dumpIndicator.update() == EMPTY) {
                                intake.in();
                            } else {
                                intake.out();
                                // FIXME: stop rid excess later in the program, or make it automatic?
                                outtake.ridExcess();
                            }
                        })
                        .addTemporalMarker(1.0, -1.6, () -> outtake.cycleAsync(TOP))
                        .lineToConstantHeading(cycleHubPose.vec()));
                queueXCalibration(trueHubPose);

                queue(intake::off);
            }
        }
        waitForStart();
    }
}

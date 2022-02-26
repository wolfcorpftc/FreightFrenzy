package org.wolfcorp.ff.opmode.test;

import static org.wolfcorp.ff.opmode.util.Match.RED;
import static org.wolfcorp.ff.robot.DumpIndicator.Status.EMPTY;
import static org.wolfcorp.ff.robot.DumpIndicator.Status.FULL;
import static org.wolfcorp.ff.vision.Barcode.EXCESS;
import static org.wolfcorp.ff.vision.Barcode.TOP;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.wolfcorp.ff.opmode.AutonomousMode;
import org.wolfcorp.ff.opmode.util.Match;

@Autonomous(name = "Cycle Auto Test", group = "!testing")
public class CycleAutoTest extends AutonomousMode {
    ElapsedTime whToHubTimer = new ElapsedTime();
    public void runOpMode() {
        Match.setupTelemetry();
        initialPose = trueWhPose;
        initHardware();
        drive.setPoseEstimate(initialPose);
        queue(() -> {
            ElapsedTime timer = new ElapsedTime();
            intake.in();
            drive.setMotorPowers(0.1);
            while (timer.seconds() < 2) {
                drive.updatePoseEstimate();
            }
            drive.setMotorPowers(0);
            intake.directedOut();
        });
        queue(whToHubTimer::reset);
        queueWarehouseSensorCalibration(trueWhPose);
        queue(fromHere()
                .addTemporalMarker(1.0, -1.1, () -> outtake.cycleAsync(TOP))
                .lineToConstantHeading(hubPose.vec()));
        queue(intake::off);
        queue(() -> Match.log(whToHubTimer.seconds() + " seconds"));
        queue(() -> drive.setPoseEstimate(trueHubPose));
        queue(() -> waitFor(4000));
//        queue(() -> sleep(300));
        waitForStart();
        runTasks();
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

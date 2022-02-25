package org.wolfcorp.ff.opmode.test;

import static org.wolfcorp.ff.opmode.util.Match.RED;
import static org.wolfcorp.ff.robot.DumpIndicator.Status.EMPTY;
import static org.wolfcorp.ff.robot.DumpIndicator.Status.FULL;
import static org.wolfcorp.ff.vision.Barcode.EXCESS;
import static org.wolfcorp.ff.vision.Barcode.TOP;

import org.wolfcorp.ff.opmode.AutonomousMode;
import org.wolfcorp.ff.opmode.util.Match;

public class CycleAutoTest extends AutonomousMode {
    @Override
    public void runOpMode() {
        Match.setupTelemetry();
//        initialPose = pos(-72 + WIDTH / 2, 72 - 22 - LENGTH / 2);
        initialPose = trueWhPose;
        drive.setPoseEstimate(initialPose);
        initHardware();

        /* cycle */ {
            for (int i = 1; i <= SCORING_CYCLES; i++) {
                Match.status("Initializing: cycle " + i);

                // *** To warehouse ***
                queue(() -> intake.in(0.8, -0.8));
//                Pose2d moddedWhPose = whPose.plus(pos(0, i == 1 ? 0 : 2 + i * 1.8));
                queue(from(trueHubPose).splineToSplineHeading(whPose));
                queueWarehouseSensorCalibration(trueWhPose);

                // *** Intake ***
                intake(i);
                queueWarehouseSensorCalibration(trueWhPose);

                // *** To hub ***
                double angleOffset = RED ? -5 : 5;
                queue(from(trueWhPose.plus(pos(0, 0, angleOffset)))
                        .addTemporalMarker(1.15, () -> {
                            // last-minute check & fix for intake
                            if (dumpIndicator.update() == FULL) {
                                intake.out();
                                outtake.outAsync(TOP);
                            } else if (dumpIndicator.update() == EMPTY) {
                                intake.in();
                            } else {
                                intake.out();
                                outtake.slideToAsync(EXCESS);
                                outtake.ridExcess();
                            }
                        })
                        .addTemporalMarker(1.0, -1.6, () -> outtake.cycleAsync(TOP))
                        .splineToSplineHeading(cycleHubPose));
                queueHubSensorCalibration(trueHubPose);

                queue(intake::off);
            }
        }
        waitForStart();
    }
}

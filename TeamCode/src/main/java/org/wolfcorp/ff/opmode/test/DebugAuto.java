package org.wolfcorp.ff.opmode.test;

import static org.wolfcorp.ff.robot.DriveConstants.WIDTH;
import static org.wolfcorp.ff.robot.DumpIndicator.Status.EMPTY;
import static org.wolfcorp.ff.robot.DumpIndicator.Status.FULL;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.wolfcorp.ff.opmode.AutonomousMode;
import org.wolfcorp.ff.opmode.util.Match;
import org.wolfcorp.ff.opmode.util.TimedController;
import org.wolfcorp.ff.robot.DriveConstants;
import org.wolfcorp.ff.robot.Drivetrain;
import org.wolfcorp.ff.robot.DumpIndicator;
import org.wolfcorp.ff.robot.Intake;
import org.wolfcorp.ff.vision.Barcode;

@Autonomous(name = "Debug", group = "test")
public class DebugAuto extends AutonomousMode {
    @Override
    public void runOpMode() {
        Match.setupTelemetry();
        initHardware();
        initialPose = pos(-72 + WIDTH / 2, 24);
        drive.setPoseEstimate(initialPose);
        waitForStart();
        cycle();
        runTasks();
    }

    public void cycle() {
        // *** Hub to warehouse ***
        queue(() -> intake.in());
        queue(fromHere().lineTo(calibrateWhPose.vec()));
        // *** Intake ***
        queue(() -> {
            TimedController intakeController = new TimedController(-25, -475, -800);
            TimedController driveController = new TimedController(0.020, 0.15, 0.35);
            while (dumpIndicator.update() == EMPTY && rangeSensor.getDistance(DistanceUnit.INCH) > 18) {
                System.out.println(rangeSensor.getDistance(DistanceUnit.INCH) + " inches");
                drive.updatePoseEstimate();
                drive.setMotorPowers(driveController.update());
                intake.setVelocityRPM(intakeController.update());
            }
            drive.updatePoseEstimate();
            drive.setMotorPowers(0);
            if (dumpIndicator.update() == EMPTY) {
                System.out.println("asfdasfd");
                drive.turn(-deg(10));
                sleep(250);
                drive.turn(-deg(10));
                sleep(250);
                drive.turn(deg(20));
                if (dumpIndicator.update() == EMPTY) {
                    sleep(10000);
                }
            }
            intake.off();
            // TODO: make sure it flows smoothly into next section(s)
        });
        // TEST!
//            queue(() -> drive.sidestepRight(0.75, 5 * (RED ? 1 : -1)));
        queue(() -> {
            sleep(250);
            if (dumpIndicator.update() != EMPTY) {
                intake.out();
            } else {
                intake.getMotor().setVelocity(1.75 * Intake.IN_SPEED);
            }
        });
        queueWarehouseSensorCalibration(pos(-72 + WIDTH / 2, 46, 0));
        queue(() -> Match.status(drive.getPoseEstimate() + "; sensor = " + rangeSensor.getDistance(DistanceUnit.INCH)));


        // *** Warehouse to hub ***
        queue(fromHere()
                // get out
                .lineTo(preWhPose.minus(pos(4, 10)).vec())
                .now(() -> {
                    switch (dumpIndicator.update()) {
                        case EMPTY:
                            intake.in();
                            break;
                        case FULL:
                            intake.out();
                        case OVERFLOW:
//                                outtake.slideToAsync(Barcode.EXCESS);
                            // outtake.dumpExcess();
                            intake.out();
                            break;
                    }
                }));
        queue(fromHere().addTemporalMarker(0.5, () -> {
            if (dumpIndicator.update() == FULL) {
                outtake.slideToAsync(Barcode.TOP);
            }
        }).lineToSplineHeading(cycleHubPose));
        queueHubSensorCalibration(trueHubPose);


        // *** Score ***
        queue(() -> {
            intake.off();

//                outtake.slideTo(Barcode.TOP); // since it gets counted in TeleOp period scoring
            outtake.slideTo(Barcode.TOP);
            outtake.dumpOut();
            sleep(1200);
            outtake.dumpIn();
            outtake.slideToAsync(Barcode.ZERO);
        });
    }
}

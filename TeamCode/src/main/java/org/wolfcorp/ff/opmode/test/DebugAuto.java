package org.wolfcorp.ff.opmode.test;

import static org.wolfcorp.ff.robot.DriveConstants.WIDTH;
import static org.wolfcorp.ff.robot.DumpIndicator.Status.EMPTY;
import static org.wolfcorp.ff.robot.DumpIndicator.Status.FULL;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.wolfcorp.ff.opmode.AutonomousMode;
import org.wolfcorp.ff.opmode.OpMode;
import org.wolfcorp.ff.opmode.util.Match;
import org.wolfcorp.ff.opmode.util.TimedController;
import org.wolfcorp.ff.robot.DriveConstants;
import org.wolfcorp.ff.robot.Drivetrain;
import org.wolfcorp.ff.robot.DumpIndicator;
import org.wolfcorp.ff.robot.Intake;
import org.wolfcorp.ff.vision.Barcode;
import org.wolfcorp.ff.vision.PolarPoint;

import java.util.HashMap;
import java.util.Map;

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
            while (dumpIndicator.update() != EMPTY && rangeSensor.getDistance(DistanceUnit.INCH) > 18) {
                System.out.println(rangeSensor.getDistance(DistanceUnit.INCH) + " inches");
                drive.updatePoseEstimate();
                drive.setMotorPowers(driveController.update());
                intake.setVelocityRPM(intakeController.update());
            }
            drive.setMotorPowers(0);
            drive.updatePoseEstimate();
            if (dumpIndicator.update() == EMPTY) {
                Match.log("Turning...");
                Vector2d freightVec = turnAndSeekFreight(deg(-20));
                // TODO: slower intake to prevent over intaking, but faster intake to ensure intake
                // TODO: fix weird turning (deg to rad conv?)
                sleep(250);
                if (dumpIndicator.update() == EMPTY) {
                    Match.log("Angle 1 = " + (freightVec.angle() - drive.getPoseEstimate().getHeading())); // TEST
                    drive.turn(freightVec.angle() - drive.getPoseEstimate().getHeading());
                    intake.getMotor().setVelocity(0.5 * Intake.IN_SPEED);
                    double dist = approachFreight();
                    // TODO: double check if we've got freight
                    drive.follow(from(drive.getPoseEstimate()).back(dist).build());
                    Match.log("Angle 2 = " + (-drive.getPoseEstimate().getHeading())); // TEST
                    // FIXME: doesn't turn correctly
                    drive.turn(-drive.getPoseEstimate().getHeading() - 1e6);
                    Match.update();
                } else {
                    drive.turn(deg(20));
                }
            }
            intake.out();
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

    /**
     * Scans for game element using intake ramp sensor while turning (in the hope that the robot may
     * bump into a freight).
     * @param angle angle to turn
     * @return the angle to the closest freight
     */
    public Vector2d turnAndSeekFreight(double angle) {
        final double INCREMENT = 200; // millis; check distance every INCREMENT milliseconds

        HashMap<Double, Double> distances = new HashMap<>(); // angle -> distance
        drive.turnAsync(angle);
        ElapsedTime turnTimer = new ElapsedTime();
        while (drive.isBusy()) {
            drive.update();
            if (turnTimer.milliseconds() >= INCREMENT) {
                distances.put(drive.getPoseEstimate().getHeading(), intakeRampDistance.getDistance(DistanceUnit.INCH));
                turnTimer.reset();
            }
        }
        double minAngle = 0;
        double minDistance = Double.MAX_VALUE;
        for (Map.Entry<Double, Double> set : distances.entrySet()) {
            if (set.getValue() < minDistance) {
                minAngle = set.getKey();
                minDistance = set.getValue();
            }
        }
        return Vector2d.polar(minDistance, minAngle);
    }

    /**
     * Approaches freight with decreasing speed (motor power) and stops upon acquiring freight.
     * @return the distance robot travelled
     * @see TimedController
     */
    public double approachFreight() {
        TimedController driveController = new TimedController(-0.05, 0.25, 0.05);
        Pose2d initialPose = drive.getPoseEstimate();
        while (intakeRampDistance.getDistance(DistanceUnit.INCH) > 3.9
                && dumpIndicator.update() == EMPTY
                && OpMode.isActive()) {
            drive.updatePoseEstimate();
            drive.setMotorPowers(driveController.update());
        }
        drive.setMotorPowers(0);
        drive.updatePoseEstimate();
        Pose2d finalPose = drive.getPoseEstimate();
        return Math.hypot(finalPose.getX() - initialPose.getX(), finalPose.getY() - initialPose.getY());
    }
}

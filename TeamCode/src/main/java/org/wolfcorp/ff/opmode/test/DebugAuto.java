package org.wolfcorp.ff.opmode.test;

import static org.wolfcorp.ff.robot.DriveConstants.TRACK_WIDTH;
import static org.wolfcorp.ff.robot.DriveConstants.WIDTH;
import static org.wolfcorp.ff.robot.Drivetrain.getAccelerationConstraint;
import static org.wolfcorp.ff.robot.Drivetrain.getVelocityConstraint;
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
import org.wolfcorp.ff.robot.Drivetrain;
import org.wolfcorp.ff.robot.Intake;
import org.wolfcorp.ff.vision.Barcode;

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
        queue(intake::in);
        queue(fromHere().lineTo(calibrateWhPose.vec()));
        // *** Intake ***
        queue(() -> {
            TimedController intakeController = new TimedController(-25, -475, -800);
            TimedController driveController = new TimedController(0.020, 0.15, 0.35);
            Match.status("pre-advance");
            while (dumpIndicator.update() != EMPTY && rangeSensor.getDistance(DistanceUnit.INCH) > 18) {
                drive.updatePoseEstimate();
                drive.setMotorPowers(driveController.update());
                intake.setVelocityRPM(intakeController.update());
            }
            drive.setMotorPowers(0);
            drive.updatePoseEstimate();
            Match.status("pre-turn");
            if (dumpIndicator.update() == EMPTY) {
                Match.status("pre-turn-and-seek");
                drive.follow(from(drive.getPoseEstimate()).strafeRight(3).build());
                Vector2d freightVec = turnAndSeekFreight(deg(-35));
                sleep(250);
                if (dumpIndicator.update() == EMPTY) {
                    Match.log("Angle 1 = " + (freightVec.angle() - drive.getPoseEstimate().getHeading())); // TEST
                    drive.updatePoseEstimate();
                    drive.follow(from(drive.getPoseEstimate()).turn(freightVec.angle() - drive.getPoseEstimate().getHeading(), 3, 3).build());
                    // FIXME: tune intake speed
                    intake.getMotor().setVelocity(0.75 * Intake.IN_SPEED);
                    double dist = approachFreight();
                    // TODO: double check if we've got freight
                    if (dumpIndicator.update() == EMPTY) {
                        // WE'RE FUCKED
                    }
                    drive.follow(from(drive.getPoseEstimate()).back(dist).build());
                    drive.updatePoseEstimate();
                    // FIXME: doesn't turn correctly
                    drive.turn(-drive.getPoseEstimate().getHeading());
                    Match.update();
                } else {
                    drive.turn(deg(20));
                }
                drive.follow(from(drive.getPoseEstimate()).strafeLeft(3).build());
            }
            // TODO: ram against wall BEFORE warehouse calibration
            intake.out();
            // TODO: make sure it flows smoothly into next section(s)
        });
        queueWarehouseSensorCalibration(pos(-72 + WIDTH / 2, 46, 0));

        // *** Warehouse to hub ***
        queue(fromHere()
                .lineTo(preWhPose.minus(pos(4, 10)).vec())
                .addTemporalMarker(2, () -> { // TODO: tune time
                    if (dumpIndicator.update() == FULL) { // TODO: check if this causes delays and inaccuracies
                        outtake.slideToAsync(Barcode.TOP);
                    }
                    // TODO: what about OVERFLOW? maybe async bucket tipping?
                }).lineToSplineHeading(cycleHubPose)
        );
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

    public void cycleBruteforce() {
        // *** Hub to warehouse ***
        queue(intake::in);
        queue(fromHere().lineTo(calibrateWhPose.vec()));
        // *** Intake ***
        queue(() -> {
            TimedController intakeController = new TimedController(-25, -475, -800);
            TimedController driveController = new TimedController(0.020, 0.15, 0.35);
            Match.status("pre-advance");
            while (dumpIndicator.update() != EMPTY && rangeSensor.getDistance(DistanceUnit.INCH) > 18) {
                drive.updatePoseEstimate();
                drive.setMotorPowers(driveController.update());
                intake.setVelocityRPM(intakeController.update());
            }
            drive.setMotorPowers(0);
            drive.updatePoseEstimate();
            Match.status("pre-turn");
            if (dumpIndicator.update() == EMPTY) {
                Match.status("pre-turn-and-seek");
                Vector2d freightVec = turnAndSeekFreight(deg(-20));
                sleep(250);
                if (dumpIndicator.update() == EMPTY) {
                    Match.log("Angle 1 = " + (freightVec.angle() - drive.getPoseEstimate().getHeading())); // TEST
                    drive.updatePoseEstimate();
                    drive.follow(from(drive.getPoseEstimate()).turn(freightVec.angle() - drive.getPoseEstimate().getHeading(), 3, 3).build());
                    // FIXME: tune intake speed
                    intake.getMotor().setVelocity(0.75 * Intake.IN_SPEED);
                    double dist = approachFreight();
                    // TODO: double check if we've got freight
                    if (dumpIndicator.update() == EMPTY) {
                        // WE'RE FUCKED
                    }
                    drive.follow(from(drive.getPoseEstimate()).back(dist).build());
                    drive.updatePoseEstimate();
                    // FIXME: doesn't turn correctly
                    drive.turn(-drive.getPoseEstimate().getHeading());
                    Match.update();
                } else {
                    drive.turn(deg(20));
                }
            }
            // TODO: ram against wall BEFORE warehouse calibration
            intake.out();
            // TODO: make sure it flows smoothly into next section(s)
        });
        queueWarehouseSensorCalibration(pos(-72 + WIDTH / 2, 46, 0));

        // *** Warehouse to hub ***
        queue(fromHere()
                .lineTo(preWhPose.minus(pos(4, 10)).vec())
                .addTemporalMarker(2, () -> { // TODO: tune time
                    if (dumpIndicator.update() == FULL) { // TODO: check if this causes delays and inaccuracies
                        outtake.slideToAsync(Barcode.TOP);
                    }
                    // TODO: what about OVERFLOW? maybe async bucket tipping?
                }).lineToSplineHeading(cycleHubPose)
        );
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
     * bump into a freight along the way).
     *
     * @param angle angle to turn
     * @return the vector pointing to the closest freight, with its origin at the robot's center
     */
    public Vector2d turnAndSeekFreight(double angle) {
        final double INCREMENT = 50; // millis; check distance every INCREMENT milliseconds

        HashMap<Double, Double> distances = new HashMap<>(); // angle -> distance
        Match.status("pre-turnAsync");
        drive.turnAsync(angle);
        drive.updatePoseEstimate();
        drive.followAsync(from(drive.getPoseEstimate()).turn(angle, 1, 1).build());
        Match.status("post-turnAsync");
        ElapsedTime turnTimer = new ElapsedTime();
        Match.telemetry.setMsTransmissionInterval(16);
        Telemetry.Item distItem = Match.createLogItem("distance", 0);
        Telemetry.Item angleItem = Match.createLogItem("angle", 0);
        while (drive.isBusy()) {
            drive.update();
            if (turnTimer.milliseconds() >= INCREMENT) {
                distances.put(drive.getPoseEstimate().getHeading(), intakeRampDistance.getDistance(DistanceUnit.INCH));
                turnTimer.reset();
            }
            distItem.setValue(intakeRampDistance.getDistance(DistanceUnit.INCH));
            angleItem.setValue(Math.toDegrees(drive.getPoseEstimate().getHeading() - 2 * Math.PI));
            Match.update();
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
     *
     * @return the distance robot travelled
     * @see TimedController
     */
    public double approachFreight() {
        TimedController driveController = new TimedController(-0.05, 0.25, 0.02);
        Pose2d initialPose = drive.getPoseEstimate();
        // approach in decreasing drivetrain speed until close enough / freight acquired
        while (intakeRampDistance.getDistance(DistanceUnit.INCH) > 4
                && dumpIndicator.update() == EMPTY
                && opModeIsActive()) {
            drive.updatePoseEstimate();
            drive.setMotorPowers(driveController.update());
        }
        drive.setMotorPowers(0);
        // if we don't have freight, approach very slowly while increasing intake speed in case
        // freight gets stuck
        ElapsedTime dumpTimer = new ElapsedTime();
        TimedController intakeController = new TimedController(-30, intake.getMotor().getVelocity(), 1.25 * Intake.IN_SPEED);
        while (opModeIsActive()) {
            drive.updatePoseEstimate();
            if (dumpIndicator.update() == EMPTY) {
                dumpTimer.reset();
                intake.getMotor().setVelocity(intakeController.update());
            } else if (dumpTimer.milliseconds() > 200) { // if the bucket is non-empty for longer than ...
                break;
            }
        }
        drive.updatePoseEstimate();
        intake.out();
        Pose2d finalPose = drive.getPoseEstimate();
        return Math.hypot(finalPose.getX() - initialPose.getX(), finalPose.getY() - initialPose.getY());
    }
}

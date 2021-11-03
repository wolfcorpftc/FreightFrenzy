package org.wolfcorp.ff.robot.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.wolfcorp.ff.robot.Drivetrain;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain drive = new Drivetrain(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            Math.abs(gamepad1.right_stick_y) < 0.8 && Math.abs(gamepad1.right_stick_y) > 0.05 ? gamepad1.right_stick_y > 0 ? -0.5 : 0.5 : -gamepad1.right_stick_y,
                            Math.abs(gamepad1.right_stick_x) < 0.8 && Math.abs(gamepad1.right_stick_x) > 0.05 ? gamepad1.right_stick_x > 0 ? -0.5 : 0.5 : -gamepad1.right_stick_x,
                            Math.abs(gamepad1.left_stick_x) < 0.6  && Math.abs(gamepad1.left_stick_x)  > 0.05 ? gamepad1.left_stick_x > 0  ? -0.3 : 0.3 : -gamepad1.left_stick_x
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("LF encoder", drive.leftFront.getCurrentPosition());
            telemetry.addData("RF encoder", drive.rightFront.getCurrentPosition());
            telemetry.addData("LB encoder", drive.leftBack.getCurrentPosition());
            telemetry.addData("RB encoder", drive.rightBack.getCurrentPosition());

            telemetry.update();
        }
    }
}

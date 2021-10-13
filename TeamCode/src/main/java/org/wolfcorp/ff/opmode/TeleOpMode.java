package org.wolfcorp.ff.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.wolfcorp.ff.robot.Drivetrain;

@TeleOp(name = "TeleOp", group = "TeleOp")
public class TeleOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
            FtcDashboard dashboard = FtcDashboard.getInstance();
            Drivetrain drive = new Drivetrain(hardwareMap);
            ElapsedTime timer = new ElapsedTime();

            // Send telemetry message to signify robot waiting;
            telemetry.addData("Message", "Hello Driver");
            telemetry.update();
            // Wait for the game to start (driver presses PLAY)

            Pose2d initialPose = new Pose2d(12, 36, Math.toRadians(0));

            drive.setPoseEstimate(initialPose);

            waitForStart();

            timer.reset();
            while (opModeIsActive()) {
                // Drivetrain
                drive.drive(
                        Math.abs(gamepad1.right_stick_y) < 0.8 && Math.abs(gamepad1.right_stick_y) > 0.05 ? gamepad1.right_stick_y > 0 ? -0.5 : 0.5 : -gamepad1.right_stick_y,
                        Math.abs(gamepad1.right_stick_x) < 0.8 && Math.abs(gamepad1.right_stick_x) > 0.05 ? gamepad1.right_stick_x > 0 ?  0.5 : -0.5 : gamepad1.right_stick_x,
                        Math.abs(gamepad1.left_stick_x) < 0.6  && Math.abs(gamepad1.left_stick_x) > 0.05  ? gamepad1.left_stick_x > 0  ?  0.3 : -0.3 : gamepad1.left_stick_x,
                        0.4,
                        gamepad1.right_trigger > 0.8);
                drive.update(); // odometry update
            }

    }
}

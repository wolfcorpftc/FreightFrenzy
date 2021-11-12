package org.wolfcorp.ff.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.wolfcorp.ff.robot.Drivetrain;

@TeleOp(name = "TeleOp", group = "TeleOp")
public class TeleOpMode extends LinearOpMode {

    private boolean click = false;

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Drivetrain drive = new Drivetrain(hardwareMap);
        ElapsedTime timer = new ElapsedTime();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Message", "Hello Driver");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)

        //Pose2d initialPose = new Pose2d(12, 36, Math.toRadians(0));

        drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();

        timer.reset();
        while (opModeIsActive()) {
            // Drivetrain

            if(!drive.isBusy()) {
                drive.drive(
                        -gamepad1.right_stick_y,
                        gamepad1.right_stick_x,
                        gamepad1.left_stick_x,
                        1,
                        gamepad1.right_trigger > 0.8
                );
            }

            if (gamepad1.b && !click){
                click = true;
                System.out.println("b");
                Trajectory toHub = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(PoseStorage.hubPose)
                        .build();
                drive.followAsync(toHub);
            }

            if (gamepad1.y && !click){
                click = true;
                drive.setPoseEstimate(PoseStorage.hubPose);
            }

            if (Math.abs(gamepad1.right_stick_y) + Math.abs(gamepad1.right_stick_x) + Math.abs(gamepad1.left_stick_x) > 0.1){
                drive.abort();
            }

            if (!gamepad1.b && !gamepad1.y){
                click = false;
            }

            drive.update(); // odometry update
            telemetry.addData("LF Power", drive.leftFront.getPower());
            telemetry.addData("LF Current", drive.leftFront.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("LB Power", drive.leftBack.getPower());
            telemetry.addData("LB Current", drive.leftBack.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("RF Power", drive.rightFront.getPower());
            telemetry.addData("RF Current", drive.rightFront.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("RB Power", drive.rightBack.getPower());
            telemetry.addData("RB Current", drive.rightBack.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.update();
        }

    }
}

package org.wolfcorp.ff.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.wolfcorp.ff.opmode.util.Match;
import org.wolfcorp.ff.robot.Drivetrain;

@TeleOp(name = "Motor Test", group = "test")
public class MotorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Match.setupTelemetry();
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
//            drive.setMotorPowers(0, 0, 0, 1);
//            sleep(5000);
//            drive.setMotorPowers(0, 0, 0, -1);
//            sleep(5000);
            drive.setMotorPowers(0, 0, 1, 0);
            sleep(5000);
            drive.setMotorPowers(0, 0, -1, 0);
            sleep(5000);
            drive.setMotorPowers(0, 1, 0, 0);
            sleep(5000);
            drive.setMotorPowers(0, -1, 0, 0);
            sleep(5000);
            drive.setMotorPowers(1, 0, 0, 0);
            sleep(5000);
            drive.setMotorPowers(-1, 0, 0, 0);
            sleep(5000);
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

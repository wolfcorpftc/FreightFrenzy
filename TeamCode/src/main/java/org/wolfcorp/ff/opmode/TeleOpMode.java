package org.wolfcorp.ff.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.wolfcorp.ff.robot.CarouselSpinner;
import org.wolfcorp.ff.robot.Drivetrain;
import org.wolfcorp.ff.robot.Shovel;

@TeleOp(name = "TeleOp", group = "TeleOp")
public class TeleOpMode extends LinearOpMode {

    private boolean click = false;

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Drivetrain drive = new Drivetrain(hardwareMap);
        Shovel shovel = new Shovel(hardwareMap);
        CarouselSpinner spinner = new CarouselSpinner(hardwareMap, this::sleep);
        ElapsedTime timer = new ElapsedTime();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Message", "Hello Driver");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)

        //Pose2d initialPose = new Pose2d(12, 36, Math.toRadians(0));

        drive.setPoseEstimate(PoseStorage.currentPose);
        shovel.setTargetPos(shovel.getCurrentPos());


        waitForStart();

        timer.reset();
        while (opModeIsActive()) {
            // Drivetrain
            if(!drive.isBusy()) {
                Vector2d input = new Vector2d(
                        //-gamepad1.right_stick_y,
                        //-gamepad1.right_stick_x
                        (gamepad1.dpad_left?0.5:0) - (gamepad1.dpad_right?0.5:0),
                        (gamepad1.dpad_up?0.5:0) - (gamepad1.dpad_down?0.5:0)
                ).rotated(-drive.getPoseEstimate().getHeading()-90);

//                drive.drive(
//                        -input.getX(),
//                        input.getY(),
//                        gamepad1.left_stick_x,
//                        1,
//                        gamepad1.right_trigger > 0.8
//                );
//                drive.setWeightedDrivePower(
//                        new Pose2d(
//                                input.getY(),
//                                input.getX(),
//                                -gamepad1.left_stick_x
//                        )
//                );

                drive.drive(
                        -gamepad1.right_stick_y - input.getY(),
                        gamepad1.right_stick_x + input.getX(),
                        gamepad1.left_stick_x,
                        1,
                        gamepad1.right_trigger > 0.8
                );
            }

            if (gamepad1.left_bumper) {
                spinner.on();
            }

            if (gamepad1.right_bumper) {
                spinner.off();
            }

            if (gamepad1.b && !gamepad1.start && !click){
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

            if ((!gamepad1.b || (gamepad1.b && gamepad1.start)) && !gamepad1.y && !gamepad1.x){
                click = false;
            }

            if (gamepad2.y) {
                shovel.setTargetPos(shovel.getCurrentPos());
                if (gamepad2.left_bumper) {
                    shovel.setPower(0.4);
                } else {
                    shovel.setPower(0.1);
                }
            } else if (gamepad2.a && !gamepad2.start) {
                shovel.setPower(-0.05);
                shovel.setTargetPos(shovel.getCurrentPos());
            } else {
                shovel.eliminateDrift();
                shovel.setPower(0);
            }

            telemetry.addData("shovel", shovel.getCurrentPos());
            telemetry.addData("shovelTarget", shovel.getTargetPos());

            drive.update(); // odometry update
            telemetry.addData("LF Power", drive.leftFront.getPower());
            telemetry.addData("LF Current", drive.leftFront.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("LB Power", drive.leftBack.getPower());
            telemetry.addData("LB Current", drive.leftBack.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("RF Power", drive.rightFront.getPower());
            telemetry.addData("RF Current", drive.rightFront.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("RB Power", drive.rightBack.getPower());
            telemetry.addData("RB Current", drive.rightBack.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("Heading", drive.getPoseEstimate().getHeading());
            telemetry.update();
        }
    }
}

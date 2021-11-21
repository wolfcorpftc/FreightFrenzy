package org.wolfcorp.ff.opmode;

import static org.wolfcorp.ff.opmode.AutonomousMode.deg;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.wolfcorp.ff.robot.CarouselSpinner;
import org.wolfcorp.ff.robot.Drivetrain;
import org.wolfcorp.ff.robot.Shovel;

public abstract class TeleOpMode extends OpMode {
    private boolean blockCheckpoint = false;
    private boolean intakeCheckpoint = false;

    public TeleOpMode() {
        Match.isRed = this.getClass().getSimpleName().contains("Red");
        // Faster telemetry
        telemetry.setMsTransmissionInterval(50);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Drivetrain drive = new Drivetrain(hardwareMap);
        Shovel shovel = new Shovel(hardwareMap);
        CarouselSpinner spinner = new CarouselSpinner(hardwareMap, this::sleep);
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime spinnerDelay = new ElapsedTime();

        log("Initializing robot");
        drive.setPoseEstimate(Match.teleOpInitialPose);
        shovel.recordRestPos();

        log("Robot initialized, waiting for start");
        waitForStart();

        log("Start!");
        timer.reset();
        while (opModeIsActive()) {
            // *** Drivetrain ***
            telemetry.addLine("TeleOp: Pre-drive");
            if(!drive.isBusy()) {
                telemetry.addLine("TeleOp: Drive");
                Vector2d input = new Vector2d(
                        //-gamepad1.right_stick_y,
                        //-gamepad1.right_stick_x
                        (gamepad1.dpad_left?0.5:0) - (gamepad1.dpad_right?0.5:0),
                        (gamepad1.dpad_up?0.5:0) - (gamepad1.dpad_down?0.5:0)
                ).rotated(-drive.getPoseEstimate().getHeading() - deg(90));

//                drive.drive(
//                        -input.getX(),
//                        input.getY(),
//                        gamepad1.left_stick_x,
//                        1,
//                        gamepad1.right_trigger > 0.8
//                );
//                drive.setWeightedDrivePower(
//                        new Pose2d(
//                                input.getY() - gamepad1.right_stick_y,
//                                input.getX() + gamepad1.right_stick_x,
//                                -gamepad1.left_stick_x
//                        )
//                );

                drive.drive(
                        gamepad1.right_stick_y + input.getY(),
                        -gamepad1.right_stick_x - input.getX(),
                        gamepad1.left_stick_x,
                        1,
                        gamepad1.right_trigger > 0.8
                );
            }

            // *** Carousel Spinner ***
            if (gamepad2.left_bumper && spinnerDelay.milliseconds() > 200) {
                spinnerDelay.reset();
                if (spinner.isOff()) {
                    spinner.on();
                }
                else {
                    spinner.off();
                }
            }

            // *** Driver Assist: Checkpoint ***
            // Go to checkpoint / hub
            if (gamepad1.b && !gamepad1.start && !blockCheckpoint){
                blockCheckpoint = true;
                System.out.println("b");
                Trajectory toHub = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(Match.hubPose)
                        .build();
                drive.followAsync(toHub);
            }

            // Set current pose as checkpoint / hub
            if (gamepad1.y && !blockCheckpoint){
                blockCheckpoint = true;
                drive.setPoseEstimate(Match.hubPose);
            }

            // Cancel current trajectory to pose
            if (Math.abs(gamepad1.right_stick_y) + Math.abs(gamepad1.right_stick_x) + Math.abs(gamepad1.left_stick_x) > 0.1){
                drive.abort();
            }

            // Unblock checkpoint functionalities when no relevant inputs are pressed
            if ((!gamepad1.b || (gamepad1.b && gamepad1.start)) && !gamepad1.y && !gamepad1.x){
                blockCheckpoint = false;
            }

            // *** Shovel ***
            if (gamepad2.y && !intakeCheckpoint) {
                intakeCheckpoint = true;
                // Go up
                shovel.recordRestPos();
                //shovel.setFree();
                if (gamepad2.right_bumper) {
                    //shovel.setPower(-0.4);
                } else {
                    shovel.up();
                    //shovel.setPower(-0.025);
                }
            } else if (gamepad2.a && !gamepad2.start && !intakeCheckpoint) {
                intakeCheckpoint = true;
                // Go down
                shovel.recordRestPos();
                //shovel.setFree();
                shovel.down();
                //shovel.setPower(0.025);
            } else {
                shovel.setPower(0);
            }

            if (!gamepad2.a && !gamepad2.y) {
                intakeCheckpoint = false;
            }

            telemetry.addData("shovel", shovel.getCurrentPos());
            telemetry.addData("shovelTarget", shovel.getRestPos());

            drive.update(); // odometry update
            telemetry.addData("LF Power", drive.leftFront.getPower());
            telemetry.addData("LF Current", drive.leftFront.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("LF Position", drive.leftFront.getCurrentPosition());

            telemetry.addData("LB Power", drive.leftBack.getPower());
            telemetry.addData("LB Current", drive.leftBack.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("LB Position", drive.leftBack.getCurrentPosition());

            telemetry.addData("RF Power", drive.rightFront.getPower());
            telemetry.addData("RF Current", drive.rightFront.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("RF Position", drive.rightFront.getCurrentPosition());

            telemetry.addData("RB Power", drive.rightBack.getPower());
            telemetry.addData("RB Current", drive.rightBack.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("RB Position", drive.rightBack.getCurrentPosition());

            telemetry.addData("Heading", drive.getPoseEstimate().getHeading());
            telemetry.update();
        }
    }
}

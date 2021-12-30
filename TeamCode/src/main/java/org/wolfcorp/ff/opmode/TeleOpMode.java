package org.wolfcorp.ff.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.wolfcorp.ff.robot.CarouselSpinner;
import org.wolfcorp.ff.robot.Outtake;
import org.wolfcorp.ff.vision.Barcode;

public abstract class TeleOpMode extends OpMode {
    protected FtcDashboard dashboard = null;

    private boolean maskSlowMode = false;
    private boolean maskCheckpoint = false;
    private boolean maskIntake = false;
    private boolean maskSlide = false;
    private boolean maskDump = false;
    private boolean maskSpinner = false;
    private boolean maskManualSpinner = false;
    private boolean maskOuttakeReset = false;
    private boolean maskSpinnerOverride = false;

    private boolean slowMode = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Match.setupTelemetry();
        telemetry.setAutoClear(true);
        initHardware();

        dashboard = FtcDashboard.getInstance();

        Match.status("Setting pose");
        drive.setPoseEstimate(Match.teleOpInitialPose);

        Match.status("Robot initialized, waiting for start");
        outtake.getServo().setPosition(Outtake.DUMP_IN_POSITION);
        waitForStart();

        Match.status("Start!");
        while (opModeIsActive()) {
            // *** Drivetrain ***
            if (gamepad1.right_bumper && !maskSlowMode) {
                slowMode = !slowMode;
                maskSlowMode = true;
            }
            if (!gamepad1.right_bumper) {
                maskSlowMode = false;
            }
            if (!drive.isBusy()) {
                telemetry.addLine("TeleOp: Drive");
                /*
                Vector2d input = new Vector2d(
                        //-gamepad1.right_stick_y,
                        //-gamepad1.right_stick_x
                        (gamepad1.dpad_left ? 0.5 : 0) - (gamepad1.dpad_right ? 0.5 : 0),
                        (gamepad1.dpad_up ? 0.5 : 0) - (gamepad1.dpad_down ? 0.5 : 0)
                ).rotated(-drive.getPoseEstimate().getHeading() - deg(90));

//                drive.drive(
//                        -input.getX(),
//                        input.getY(),
//                        gamepad1.left_stick_x,
//                        1,
//                        gamepad1.right_trigger > 0.8
//                );
                if (input.getX() * input.getY() != 0) {
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    input.getY(),
                                    input.getX(),
                                    -gamepad1.left_stick_x
                            )
                    );
                } else {
                    drive.drive(
                            gamepad1.right_stick_y,
                            -gamepad1.right_stick_x,
                            gamepad1.left_stick_x,
                            0.4,
                            slowMode
                    );
                }
              */
                drive.drive(
                        gamepad1.right_stick_y,
                        -gamepad1.right_stick_x,
                        gamepad1.left_stick_x,
                        0.4,
                        slowMode
                );
            }

            // *** Automatic Carousel Spinner ***
            if (gamepad2.left_bumper && !maskSpinner) {
                maskSpinner = true;
                spinner.spinAsync(8, CarouselSpinner.SPIN_TIME, 300);
            }

            if (!gamepad2.left_bumper) {
                maskSpinner = false;
            }

            // *** Manual Carousel Spinner ***
            if (gamepad2.left_trigger > 0.8 && !maskManualSpinner) {
                maskManualSpinner = true;
                if (spinner.isOn()) {
                    spinner.off();
                } else {
                    spinner.on();
                }
            }

            if (gamepad2.left_trigger < 0.8) {
                maskManualSpinner = false;
            }

            // *** Override Carousel Spinner ***
            if (gamepad1.left_trigger > 0.8 && gamepad1.right_trigger > 0.8 && !maskSpinnerOverride) {
                maskSpinnerOverride = true;
                spinner.stopSpin();
            }

            if (!(gamepad1.left_trigger > 0.8 && gamepad1.right_trigger > 0.8)) {
                maskSpinnerOverride = false;
            }

            // *** Driver Assist: Checkpoint ***

            // Go to checkpoint / hub
            /*
            if (gamepad1.b && !gamepad1.start && !maskCheckpoint) {
                maskCheckpoint = true;
                Trajectory toHub = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(Match.hubPose)
                        .build();
                drive.followAsync(toHub);
            }

            // Set current pose as checkpoint / hub
            if (gamepad1.y && !maskCheckpoint) {
                maskCheckpoint = true;
                drive.setPoseEstimate(Match.hubPose);
            }

            // Cancel current trajectory to pose
            if (Math.abs(gamepad1.right_stick_y) + Math.abs(gamepad1.right_stick_x) + Math.abs(gamepad1.left_stick_x) > 0.1) {
                drive.abort();
            }

            // Unblock checkpoint functionalities when button presses are released
            if ((!gamepad1.b || (gamepad1.b && gamepad1.start)) && !gamepad1.y && !gamepad1.x) {
                maskCheckpoint = false;
            }*/

            // *** Intake ***
            if (gamepad2.b && !gamepad2.start && !maskIntake) {
                maskIntake = true;
                intake.toggleOut();
            } else if (gamepad2.x && !maskIntake) {
                maskIntake = true;
                // the dump must be at the bottom-most position when intake is on
                outtake.slideToAsync(Barcode.ZERO);
                intake.toggleIn();
            }

            if (!gamepad2.b && !gamepad2.x) {
                maskIntake = false;
            }

            // *** Outtake: slide - manual ***
            if (gamepad2.y && !maskSlide) {
                if (outtake.getMotor().getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
                    outtake.resetSlide();
                }
                outtake.extend(gamepad2.back);
            } else if (gamepad2.a && !maskSlide) {
                if (outtake.getMotor().getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
                    outtake.resetSlide();
                }
                outtake.retract(gamepad2.back);
            } else if (outtake.getMotor().getMode() != DcMotor.RunMode.RUN_TO_POSITION && !maskSlide) {
                outtake.getMotor().setVelocity(0);
            }

            // *** Outtake: slide - snap ***
            if (gamepad2.dpad_up && !maskSlide) {
                maskSlide = true;
                outtake.slideToAsync(Barcode.TOP);
            } else if (gamepad2.dpad_right && !maskSlide) {
                maskSlide = true;
                outtake.slideToAsync(Barcode.MID);
            } else if (gamepad2.dpad_down && !maskSlide) {
                maskSlide = true;
                outtake.slideToAsync(Barcode.BOT);
            } else if (outtake.getMotor().getMode() == DcMotor.RunMode.RUN_TO_POSITION
                    && outtake.reachedTargetPosition()) {
                outtake.resetSlide();
            }

            if (!(gamepad2.dpad_up || gamepad2.dpad_right || gamepad2.dpad_down || gamepad2.a || gamepad2.y)) {
                maskSlide = false;
            }

            // *** Outtake : reset ***
            if (gamepad2.left_trigger > 0.8 && gamepad2.right_trigger > 0.8 && !maskOuttakeReset) {
                outtake.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                outtake.getMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                maskOuttakeReset = true;
            }

            if (gamepad2.left_trigger < 0.8 || gamepad2.right_trigger < 0.8) {
                maskOuttakeReset = false;
            }

            // *** Outtake: dump ***
            if (gamepad2.right_bumper && !maskDump) {
                maskDump = true;
                outtake.toggleDump();
            }

            if (!gamepad2.right_bumper) {
                maskDump = false;
            }

            // *** Outtake: dump status ***
            dumpIndicator.update();

            // *** Odometry update ***
            drive.update();

            // *** Telemetry ***
            telemetry.addData("Spinner is On", spinner.isOn());
            telemetry.addData("Spinner Masked", maskSpinner);
            telemetry.addData("Spinner Power", spinner.getServo().getPower());
            telemetry.addData("Slow Mode", slowMode);
            telemetry.addData("Mask Slow Mode", maskSlowMode);

            telemetry.addData("Robot Pose", drive.getPoseEstimate().toString());

            telemetry.addData("Intake Current Speed", intake.getMotor().getVelocity());
            telemetry.addData("Intake Current Power", intake.getMotor().getPower());
            telemetry.addData("Intake Current Current", intake.getMotor().getCurrent(CurrentUnit.MILLIAMPS));

            telemetry.addData("Outtake Current Pos", outtake.getMotor().getCurrentPosition());
            telemetry.addData("Outtake Target Pos", outtake.getMotor().getTargetPosition());
            telemetry.addData("Outtake Dump Pos", outtake.getServo().getPosition());

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

//            telemetry.addData("Touch Sensor", !touchSensor.getState());
            telemetry.addData("Lower Dump Distance", lowerDumpDistance.getDistance(DistanceUnit.INCH));
            telemetry.addData("Upper Dump Distance", upperDumpDistance.getDistance(DistanceUnit.INCH));
            telemetry.addData("Dump R", lowerDumpDistance.red());
            telemetry.addData("Dump G", lowerDumpDistance.green());
            telemetry.addData("Dump B", lowerDumpDistance.blue());
            telemetry.addData("Dump Status", dumpIndicator.update().toString().toLowerCase());
            telemetry.addData("Intake Ramp Distance", intakeRampDistance.getDistance(DistanceUnit.INCH));

            telemetry.update();
        }
        resetHardware();
    }
}

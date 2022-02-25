package org.wolfcorp.ff.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.wolfcorp.ff.opmode.util.Match;
import org.wolfcorp.ff.robot.CarouselSpinner;
import org.wolfcorp.ff.robot.Outtake;
import org.wolfcorp.ff.robot.ShippingArm;
import org.wolfcorp.ff.vision.Barcode;

// NOTE ABOUT TELEOP: Direction of robot is SWAPPED
public abstract class TeleOpDebugging extends OpMode {
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
    private boolean maskSnapTurn = false;
    private boolean maskGyroReset = false;
    private boolean maskSlamForward = false;
    private boolean maskToggleClaw = false;

    private boolean slowMode = false;

    private ElapsedTime timer = new ElapsedTime();

    private double c1=0, c2=0, c3=0, c4=0, c5=0;
    private double ctimer = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        Match.setupTelemetry();
        telemetry.setAutoClear(true);
        initHardware();

        dashboard = FtcDashboard.getInstance();

        Match.status("Setting pose");
        drive.setPoseEstimate(Match.teleOpInitialPose);

        Match.status("Robot initialized, waiting for start");
        outtake.getDump().setPosition(Outtake.DUMP_IN_POSITION);

        if (Match.RED) {
            drive.setExternalHeadingDeg(180);
        }

        waitForStart();

        Match.status("Start!");
        while (opModeIsActive()) {

            // *** Drivetrain ***

            // *** Slow Mode ***
            if (gamepad1.right_bumper && !maskSlowMode) {
                slowMode = !slowMode;
                maskSlowMode = true;
            }
            if (!gamepad1.right_bumper) {
                maskSlowMode = false;
            }

            if (!drive.isBusy() && drive.leftFront.getMode() != DcMotor.RunMode.RUN_TO_POSITION
                    && !maskGyroReset && !maskSlamForward && !maskSnapTurn) {

                // *** Even Slower Mode
                if (gamepad1.right_trigger > 0.5) {
                    drive.drive(
                            gamepad1.right_stick_y,
                            -gamepad1.right_stick_x,
                            gamepad1.left_stick_x,
                            0.32,
                            true
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
            }

            // *** Driver Assist: Checkpoint ***

            // Go to checkpoint / hub
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
            }

            // *** Snap Robot *** //
            if (gamepad1.left_bumper) {
                maskSnapTurn = true;
                // TODO: talk with kevin
                drive.turnToDegAsync(Math.round(drive.getExternalHeadingDeg() / 90.0) * 90);
            } else {
                maskSnapTurn = false;
            }

            // *** Reset Gyro *** //
            if (gamepad1.dpad_left && !maskGyroReset) {
                maskGyroReset = true;
                // TODO: Talk with kevin about reset
                drive.setExternalHeadingDeg((Math.round(drive.getExternalHeadingDeg() / 90.0) * 90) % 360);
            } else if (!gamepad1.dpad_left) {
                maskGyroReset = false;
            }

            // *** Slam and Forward *** //
            if (gamepad1.dpad_up && !maskSlamForward) {
                maskSlamForward = true;
                drive.backward(1, 24);
            } else if (!gamepad1.dpad_up) {
                maskSlamForward = false;
            }

            // *** Automatic Carousel Spinner ***
            if (gamepad2.left_bumper && !maskSpinner) {
                maskSpinner = true;
                spinner.spinAsync(10, CarouselSpinner.SPIN_TIME, 200);
            }

            if (!gamepad2.left_bumper) {
                maskSpinner = false;
            }

            // *** Manual Carousel Spinner ***
            if (gamepad2.left_trigger > 0.8 && !maskManualSpinner) {
                maskManualSpinner = true;
                boolean isOn = spinner.isOn();
                spinner.stopSpin();
                if (!isOn) {
                    spinner.on();
                }
            }

            if (gamepad2.left_trigger < 0.8) {
                maskManualSpinner = false;
            }

            // *** Override Carousel Spinner ***
            if (gamepad1.left_trigger > 0.4 && gamepad1.right_trigger > 0.4 && !maskSpinnerOverride) {
                maskSpinnerOverride = true;
                spinner.stopSpin();
            }

            if (!(gamepad1.left_trigger > 0.4 && gamepad1.right_trigger > 0.4)) {
                maskSpinnerOverride = false;
            }

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
                if (outtake.getSlide().getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
                    outtake.resetSlideMode();
                }
                outtake.extend(gamepad2.back);
            } else if (gamepad2.a && !maskSlide) {
                if (outtake.getSlide().getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
                    outtake.resetSlideMode();
                }
                outtake.retract(gamepad2.back);
            } else if (outtake.getSlide().getMode() != DcMotor.RunMode.RUN_TO_POSITION && !maskSlide) {
                outtake.getSlide().setVelocity(0);
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
            } else if (outtake.getSlide().getMode() == DcMotor.RunMode.RUN_TO_POSITION
                    && outtake.isAtTargetPosition()) {
                outtake.resetSlideMode();
            }

            if (!(gamepad2.dpad_up || gamepad2.dpad_right || gamepad2.dpad_down || gamepad2.a || gamepad2.y)) {
                maskSlide = false;
            }

            // *** Outtake : reset ***
            if (gamepad1.dpad_right && !maskOuttakeReset) {
                outtake.getSlide().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                outtake.getSlide().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                maskOuttakeReset = true;
            }

            if (gamepad2.left_trigger < 0.8 || gamepad2.right_trigger < 0.8) {
                maskOuttakeReset = false;
            }

            // *** Outtake: dump ***
            if (!gamepad2.right_bumper) {
                maskDump = false;
            }

            // *** Shipping Element Arm: claw ***
            if (gamepad2.right_trigger > 0.8 && !maskToggleClaw) {
                maskToggleClaw = true;
                shippingArm.toggleClaw();
            }

            if (gamepad2.right_trigger < 0.8) {
                maskToggleClaw = false;
            }

            // *** Shipping Element Arm: claw ***
            double armMultiplier = Math.abs(gamepad2.left_stick_y)
                    * (gamepad2.right_stick_button ? 0.4 : 1);
            if (gamepad2.left_stick_y < -0.02) {
                shippingArm.setArmVelocity(armMultiplier * ShippingArm.ARM_OUT_SPEED);
            } else if (gamepad2.left_stick_y > 0.02) {
                shippingArm.setArmVelocity(armMultiplier * ShippingArm.ARM_IN_SPEED);
            } else {
                shippingArm.holdPosition();
            }

            // *** Outtake: dump status ***
            dumpIndicator.update();

            // *** Odometry update ***
            drive.update();

            // *** Telemetry ***

            telemetry.addData("Shipping Arm Position", shippingArm.getMotor().getCurrentPosition());

            telemetry.addData("Spinner is On", spinner.isOn());
            telemetry.addData("Spinner Masked", maskSpinner);
            telemetry.addData("Spinner Power", spinner.getServo().getPower());
            telemetry.addData("Slow Mode", slowMode);
            telemetry.addData("Mask Slow Mode", maskSlowMode);

            telemetry.addData("Robot Pose", drive.getPoseEstimate().toString());
            telemetry.addData("Snap To Angle", Math.round(drive.getExternalHeadingDeg() / 90.0) * 90);

            telemetry.addData("Intake Current Speed", intake.getFront().getVelocity());
            telemetry.addData("Intake Current Power", intake.getFront().getPower());
            telemetry.addData("Intake Current Current", intake.getFront().getCurrent(CurrentUnit.MILLIAMPS));

            telemetry.addData("Outtake Current Pos", outtake.getSlide().getCurrentPosition());
            telemetry.addData("Outtake Target Pos", outtake.getSlide().getTargetPosition());
            telemetry.addData("Outtake Dump Pos", outtake.getDump().getPosition());

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
            telemetry.addData("Pose Calibration Distance", rangeSensor.getDistance(DistanceUnit.INCH));

            telemetry.addData("Arm Position", shippingArm.getMotor().getCurrentPosition());

            telemetry.update();

        }
        resetHardware();
    }

    public double gettime(){
        double a = ctimer;
        ctimer = timer.milliseconds();
        return timer.milliseconds()-a;
    }
}

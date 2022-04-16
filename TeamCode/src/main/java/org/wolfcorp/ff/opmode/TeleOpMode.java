package org.wolfcorp.ff.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.wolfcorp.ff.opmode.util.Match;
import org.wolfcorp.ff.robot.CarouselSpinner;
import org.wolfcorp.ff.robot.DumpIndicator;
import org.wolfcorp.ff.robot.Outtake;
import org.wolfcorp.ff.robot.ShippingArm;
import org.wolfcorp.ff.robot.TapeMeasure;
import org.wolfcorp.ff.vision.Barcode;

// NOTE ABOUT TELEOP: Direction of robot is SWAPPED
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
    private boolean maskToggleClaw = false;

    private boolean slowMode = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Match.setupTelemetry();
        telemetry.setAutoClear(true);
        initHardware();

        dashboard = FtcDashboard.getInstance();
        ElapsedTime runtime = new ElapsedTime();

        Match.status("Setting pose");
        drive.setPoseEstimate(Match.teleOpInitialPose);

        Match.status("Robot initialized, waiting for start");
        if (Match.RED) {
            drive.setExternalHeadingDeg(180);
        }
        outtake.getServo().setPosition(Outtake.DUMP_IN_POSITION);
        shippingArm.toggleClaw();
        spinner.on();
        sleep(100);
        spinner.off();
        shippingArm.toggleClaw();

        waitForStart();
        runtime.reset();

        Match.status("Start!");
        while (opModeIsActive()) {
            // *** Outtake: dump ***
            if (!gamepad2.right_bumper) {
                maskDump = false;
            } else if (gamepad2.right_bumper && !maskDump) {
                maskDump = true;
                outtake.toggleDump();
            }

            // *** Drivetrain ***
            if (!drive.isBusy() && drive.leftFront.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {

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
//                      drive.drive(
//                            gamepad1.right_stick_y,
//                            -gamepad1.right_stick_x,
//                            gamepad1.left_stick_x,
//                            0.4,
//                            slowMode
//                      );
                        drive.drive(
                            gamepad1.right_stick_y,
                            -gamepad1.right_stick_x,
                            gamepad1.left_stick_x,
                            0.28,
                            slowMode && gamepad1.right_stick_y < -0.02
                    );
                }
            }

            // *** Slow Mode ***
            if (!gamepad1.left_bumper) {
                maskSlowMode = false;
            } else if (gamepad1.left_bumper && !maskSlowMode) {
                slowMode = !slowMode;
                maskSlowMode = true;
            }

            // *** Outtake: slide - manual ***
            if (gamepad2.y && !maskSlide) {
                if (outtake.getMotor().getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
                    outtake.resetSlide();
                }
                outtake.extend(gamepad2.back);
            } else if (gamepad2.a && !gamepad2.start && !gamepad1.start && !maskSlide) {
                if (outtake.getMotor().getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
                    outtake.resetSlide();
                }
                outtake.retract(gamepad2.back);
            } else if (outtake.getMotor().getMode() != DcMotor.RunMode.RUN_TO_POSITION && !maskSlide) {
                outtake.getMotor().setVelocity(0);
            }

            // *** Intake ***
            if (gamepad2.b && !gamepad2.start && !gamepad1.start && !maskIntake) {
                maskIntake = true;
                if (dumpIndicator.update() == DumpIndicator.Status.OVERFLOW && !intake.isOn()) {
                    outtake.dumpExcess();
                } else {
                    outtake.dumpIn();
                }
                intake.toggleOut();
            } else if (gamepad2.x && !maskIntake) {
                maskIntake = true;
                // the dump must be at the bottom-most position when intake is on
                outtake.slideToAsync(Barcode.ZERO);
                outtake.dumpIn();
                intake.toggleIn();
            }

            if (!gamepad2.b && !gamepad2.x) {
                maskIntake = false;
            }

            // *** Automatic Carousel Spinner ***
            if (!gamepad2.left_bumper) {
                maskSpinner = false;
            } else if (gamepad2.left_bumper && !maskSpinner) {
                maskSpinner = true;
                spinner.spinAsync(10, CarouselSpinner.SPIN_TIME, 300);
            }

            // *** Manual Carousel Spinner ***
            if (gamepad2.left_trigger < 0.8) {
                maskManualSpinner = false;
            } else if (gamepad2.left_trigger > 0.8 && !maskManualSpinner) {
                maskManualSpinner = true;
                boolean isOn = spinner.isOn();
                spinner.stopSpin();
                if (!isOn) {
                    spinner.on();
                }
            }

            // *** Override Carousel Spinner ***
            if (gamepad1.left_trigger > 0.4 && gamepad1.right_trigger > 0.4 && !maskSpinnerOverride) {
                maskSpinnerOverride = true;
                 spinner.stopSpin();
            }

            if (!(gamepad1.left_trigger > 0.4 && gamepad1.right_trigger > 0.4)) {
                maskSpinnerOverride = false;
            }

            if (gamepad2.right_trigger > 0.8 && !maskToggleClaw) {
                maskToggleClaw = true;
                shippingArm.toggleClaw();
            } else if (gamepad2.right_trigger <= 0.8) {
                maskToggleClaw = false;
            }

            // *** Shipping Element Arm: claw ***
            double armMultiplier = Math.abs(gamepad2.left_stick_y);
            if (gamepad2.left_stick_y < -0.02) {
                shippingArm.setArmVelocity(armMultiplier * ShippingArm.ARM_OUT_SPEED);
            } else if (gamepad2.left_stick_y > 0.02) {
                shippingArm.setArmVelocity(armMultiplier * ShippingArm.ARM_IN_SPEED);
            } else {
                shippingArm.holdPosition(runtime.seconds() > (120 - 45));
            }

            // *** Outtake : reset ***
            if (gamepad2.dpad_right && !maskOuttakeReset) {
                outtake.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                outtake.getMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                maskOuttakeReset = true;
            }

            if (gamepad2.dpad_up) {
                if (outtake.getMotor().getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
                    outtake.resetSlide();
                }
                outtake.getMotor().setVelocity(Outtake.SLIDE_UP_SPEED);
            } else if (gamepad2.dpad_down) {
                if (outtake.getMotor().getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
                    outtake.resetSlide();
                }
                outtake.getMotor().setVelocity(Outtake.SLIDE_DOWN_SPEED);
            }

            if (!gamepad2.dpad_right) {
                maskOuttakeReset = false;
            }

            // *** Tape measure
            if (runtime.seconds() > (120 - 45) || true) {
                if (gamepad1.dpad_left) {
                    tapeMeasure.rotateTapeIncrement(-1 * (slowMode ? 0.003 : 0.006));
                } else if (gamepad1.dpad_right) {
                    tapeMeasure.rotateTapeIncrement(1 * (slowMode ? 0.003 : 0.006));
                }
            }
            if (gamepad1.dpad_up) {
                tapeMeasure.pitchTape(-0.25 * (slowMode ? 0.3 : 1));
            } else if (gamepad1.dpad_down) {
                tapeMeasure.pitchTape(0.2);
            } else {
                tapeMeasure.pitchTape(0);
            }
            if ((gamepad1.a && !gamepad1.start && !gamepad2.start) || gamepad2.right_stick_y > 0.4) {
                tapeMeasure.spoolTape(1);
            } else if (gamepad1.y || gamepad2.right_stick_y < -0.4) {
                tapeMeasure.spoolTape(-1);
            } else {
                tapeMeasure.spoolTape(0);
            }

            // *** Outtake: dump status ***
            dumpIndicator.update();

//            telemetry.addData("voltage", drive.batteryVoltageSensor.getVoltage());
//            telemetry.addData("ramp sensor", intakeRampDistance.getDistance(DistanceUnit.INCH));
//            telemetry.addData("top distance", upperDumpDistance.getDistance(DistanceUnit.INCH));
//            telemetry.addData("bottom disance", lowerDumpDistance.getDistance(DistanceUnit.INCH));
//            telemetry.addData("distance sensor", infaredDistanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.addData("tapeRotate position", tapeMeasure.getRotate().getPosition());
            telemetry.addData("distance", altRangeSensor.getDistance(DistanceUnit.INCH));
            telemetry.addData("distance", rangeSensor.getDistance(DistanceUnit.INCH));


            // *** Odometry update ***
            // drive.update();

            telemetry.update();
        }
        resetHardware();
    }
}

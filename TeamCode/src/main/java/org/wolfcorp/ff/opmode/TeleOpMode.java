package org.wolfcorp.ff.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.wolfcorp.ff.opmode.util.Match;
import org.wolfcorp.ff.robot.CarouselSpinner;
import org.wolfcorp.ff.robot.Outtake;
import org.wolfcorp.ff.robot.ShippingArm;
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
    private boolean maskSnapTurn = false;
    private boolean maskGyroReset = false;
    private boolean maskSlamForward = false;
    private boolean maskToggleClaw = false;

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
        outtake.getDump().setPosition(Outtake.DUMP_IN_POSITION);

        if (Match.RED) {
            drive.setExternalHeadingDeg(180);
        }

        waitForStart();

        Match.status("Start!");
        while (opModeIsActive()) {
            // *** Outtake: dump ***
            if (gamepad2.right_bumper && !maskDump) {
                maskDump = true;
                outtake.toggleDump();
                // FIXME
//                outtake.toggleDump();
            }

            if (!gamepad2.right_bumper) {
                maskDump = false;
            }

            // *** Drivetrain ***
            if (!drive.isBusy() && drive.leftFront.getMode() != DcMotor.RunMode.RUN_TO_POSITION
                    && !maskGyroReset && !maskSlamForward && !maskSnapTurn) {

                // *** Even Slower Mode
                if (gamepad1.right_trigger > 0.5) {
                    drive.drive(
                            -gamepad1.right_stick_x,
                            -gamepad1.right_stick_y,
                            gamepad1.left_stick_x,
                            0.32,
                            true
                    );
                } else {
                    drive.drive(
                            -gamepad1.right_stick_x,
                            -gamepad1.right_stick_y,
                            gamepad1.left_stick_x,
                            0.4,
                            slowMode
                    );
                }
            }

            // *** Slow Mode ***
            if (gamepad1.right_bumper && !maskSlowMode) {
                slowMode = !slowMode;
                maskSlowMode = true;
            }
            if (!gamepad1.right_bumper) {
                maskSlowMode = false;
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

            // *** Automatic Carousel Spinner ***
            if (gamepad2.left_bumper && !maskSpinner) {
                maskSpinner = true;
                spinner.spinAsync(10, CarouselSpinner.SPIN_TIME, 50);
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

            if (!(gamepad2.dpad_up || gamepad2.dpad_right || gamepad2.dpad_down || gamepad2.a || gamepad2.y)) {
                maskSlide = false;
            }
/*

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


            // *** Outtake : reset ***
            if (gamepad1.dpad_right && !maskOuttakeReset) {
                outtake.getSlide().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                outtake.getSlide().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                maskOuttakeReset = true;
            }

            if (!gamepad1.dpad_right) {
                maskOuttakeReset = false;
            }

            // *** Outtake: dump status ***
            dumpIndicator.update();
*/
            // *** Odometry update ***
            drive.update();

            telemetry.addData("Slide", outtake.getSlide().getCurrentPosition());
            telemetry.addData("Dump", outtake.getDump().getPosition());
            telemetry.addData("Pivot", outtake.getPivot().getPosition());
            telemetry.update();
        }
        resetHardware();
    }
}

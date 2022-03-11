package org.wolfcorp.ff.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.wolfcorp.ff.opmode.util.Match;
import org.wolfcorp.ff.robot.CarouselSpinner;
import org.wolfcorp.ff.robot.Outtake;
import org.wolfcorp.ff.robot.ShippingArm;
import org.wolfcorp.ff.vision.Barcode;

@TeleOp(name = "Demo TeleOp", group = "!main")
public class DemoTeleOp extends OpMode {
    protected FtcDashboard dashboard = null;

    private boolean maskIntake = false;
    private boolean maskSlide = false;
    private boolean maskDump = false;
    private boolean maskSpinner = false;
    private boolean maskManualSpinner = false;
    private boolean maskOuttakeReset = false;
    private boolean maskSpinnerOverride = false;
    private boolean maskToggleClaw = false;
    private boolean maskLights = false;

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
            }

            if (!gamepad2.right_bumper) {
                maskDump = false;
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

            // *** Intake ***
            if (gamepad2.b && !gamepad2.start && !maskIntake) {
                maskIntake = true;
                intake.toggleOut();
            } else if (gamepad2.x && !maskIntake) {
                maskIntake = true;
                // the dump must be at the bottom-most position when intake is on
                outtake.slideToAsync(Barcode.ZERO);
                intake.toggleIn(0.75);
            }

            if (!gamepad2.b && !gamepad2.x) {
                maskIntake = false;
            }

            // *** Automatic Carousel Spinner ***
            if (gamepad2.left_bumper && !maskSpinner) {
                maskSpinner = true;
                spinner.spinAsync(10, CarouselSpinner.SPIN_TIME, 250);
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

            if (!(gamepad2.a || gamepad2.y)) {
                maskSlide = false;
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
                shippingArm.holdPosition(false);
            }

            // *** Outtake : reset ***
            if (gamepad1.dpad_right && !maskOuttakeReset) {
                outtake.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                outtake.getMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                maskOuttakeReset = true;
            }

            if (!gamepad1.dpad_right) {
                maskOuttakeReset = false;
            }

            // *** Light flash && Outtake: dump status ***
            if (gamepad2.dpad_up && !maskLights) {
                maskLights = true;
                System.out.println("hit");
                for (int i = 0; i < 15; i++) {
                    System.out.println(i + "pre");
                    dumpIndicator.full();
                    sleep(400);
                    System.out.println("post");
                    dumpIndicator.overflow();
                    sleep(400);
                }
            } else if (!gamepad2.dpad_up) {
                maskLights = false;
                dumpIndicator.update();
            }

            telemetry.update();
        }
        resetHardware();
    }
}

package org.wolfcorp.ff.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.wolfcorp.ff.opmode.Match;
import org.wolfcorp.ff.vision.Barcode;

public class Outtake {
    public static double TICKS_PER_REV = 1425.1;
    public static double OUTTAKE_MAX_SPEED = 117;
    public static double OUTTAKE_UP_SPEED = 100 / 60.0 * TICKS_PER_REV;
    public static double OUTTAKE_DOWN_SPEED = -100 / 60.0 * TICKS_PER_REV;

    public static int TOP_POSITION = 2000;
    public static int MID_POSITION = 1000;
    public static int BOT_POSITION = 400;

    public static int MIN_POSITION = -100;
    public static int MAX_POSITION = 2100;

    public static double DUMP_IN_POSITION = 0.88;
    public static double DUMP_OUT_POSITION = 0.45;

    private DcMotorEx motor;
    private Servo servo;

    private boolean isDumpOut = false;

    public Outtake(HardwareMap hwMap) {
        motor = hwMap.get(DcMotorEx.class, "outtake");
        servo = hwMap.get(Servo.class, "dump");

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Sets the slide to move. Intended for TeleOp.
     * @param extend whether to extend or retract the slide
     */
    public void slide(boolean extend, boolean ignoreOverextension) {
        boolean isOverextension = (extend && motor.getCurrentPosition() >= MAX_POSITION)
                || (!extend && motor.getCurrentPosition() <= MIN_POSITION);
        if (!isOverextension || ignoreOverextension) {
            motor.setVelocity(extend ? OUTTAKE_UP_SPEED : OUTTAKE_DOWN_SPEED);
        }
        else {
            Match.status("Overextension");
            motor.setPower(0);
        }
    }

    /**
     * Moves the slide outward. Intended for TeleOp.
     */
    public void extend(boolean ignoreOverextension) {
        slide(true, ignoreOverextension);
    }

    /**
     * Moves the slide inward. Intended for TeleOp.
     */
    public void retract(boolean ignoreOverextension) {
        slide(false, ignoreOverextension);
    }

    /**
     * Sets the run mode to {@code RUN_USING_ENCODER} and stop the motor.
     * This method is {@code RUN_TO_POSITION}-aware.
     */
    public void resetSlide() {
        motor.setVelocity(0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Sets the slide to run to a position asynchronously. <b>Warning: </b> This does not reset the
     * run mode to {@code RUN_USING_ENCODER}. To interrupt or get out of {@code RUN_TO_POSITION}, use
     * {@link Outtake#resetSlide()}.
     * @param barcode target level to slide to / barcode scan result
     */
    public void slideToAsync(Barcode barcode) {
        resetSlide();
        switch (barcode) {
            case TOP:
                runToPositionAsync(TOP_POSITION);
                break;
            case MID:
                runToPositionAsync(MID_POSITION);
                break;
            default:
            case BOT:
                runToPositionAsync(BOT_POSITION);
                break;
            case ZERO:
                runToPositionAsync(0);
                break;
        }
    }

    /**
     * Moves the slide to a hub level / tier synchronously
     * @param barcode destination hub level / tier
     */
    public void slideTo(Barcode barcode) {
        slideToAsync(barcode);
        while (motor.isBusy() && !Thread.interrupted());
        resetSlide();
    }

    /**
     * Treats current encoder position as zero
     */
    public void setInitialPos() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Synchronously move the slide to an encoder position. Interrupt-aware.
     * @param position destination encoder position
     */
    public void runToPositionAsync(int position) {
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (motor.getCurrentPosition() < position) {
            motor.setVelocity(OUTTAKE_UP_SPEED);
        } else {
            motor.setVelocity(OUTTAKE_DOWN_SPEED);
        }
    }

    /**
     * @return the {@link DcMotorEx} object that corresponds to the slide motor.
     */
    public DcMotorEx getMotor() {
        return motor;
    }

    /**
     * @return the {@link Servo} object that corresponds to the servo motor.
     */
    public Servo getServo() {
        return servo;
    }

    /**
     * Asynchronously toggles the position of the dump (in/out). This method is oblivious to the actual current
     * position of the robot and decides which position to turn to based on an assumed internal
     * state (defaults to closed at initialization.
     */
    public void toggleDump() {
        if (isDumpOut) {
            dumpIn();
        }
        else {
            dumpOut();
        }
    }

    /**
     * Asynchronously turn the dump inward.
     */
    public void dumpIn() {
        isDumpOut = false;
        servo.setPosition(DUMP_IN_POSITION);
    }

    /**
     * Asynchronously turn the dump outward.
     */
    public void dumpOut() {
        isDumpOut = true;
        servo.setPosition(DUMP_OUT_POSITION);
    }
}

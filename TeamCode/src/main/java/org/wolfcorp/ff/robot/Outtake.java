package org.wolfcorp.ff.robot;

import static org.wolfcorp.ff.opmode.util.Match.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.wolfcorp.ff.opmode.util.Match;
import org.wolfcorp.ff.vision.Barcode;

import java.util.Objects;

public class Outtake {
    public static final double SLIDE_TICKS_PER_REV = 1425.1;
    public static final double SLIDE_MAX_SPEED = 117 / 60.0 * SLIDE_TICKS_PER_REV; // ticks/sec
    public static final double SLIDE_UP_SPEED = 1 * SLIDE_MAX_SPEED; // ticks/sec
    public static final double SLIDE_DOWN_SPEED = -1 * SLIDE_MAX_SPEED; // ticks/sec

    public static final int SLIDE_TOP_POSITION = 1900;
    public static final int SLIDE_MID_POSITION = 1000;
    public static final int SLIDE_EXCESS_POSITION = 400;
    public static final int SLIDE_BOT_POSITION = 400;

    public static final int SLIDE_MIN_POSITION = -100;
    public static final int SLIDE_MAX_POSITION = 2100;

    public static final double DUMP_EXCESS_POSITION = 0.99;
    public static final double DUMP_IN_POSITION = 0.6;
    public static final double DUMP_OUT_POSITION = 0.025;

    public static final double DUMP_OVERFLOW_DIST = 1.6;
    public static final double DUMP_FULL_DIST = 1.6;

    private final DcMotorEx motor; // slide motor
    private final Servo servo; // dump servo
    private final Servo pivservo; // dump servo

    private final Object motorModeLock = new Object();
    private boolean isDumpOut = false;

    public Outtake(HardwareMap hwMap) {
        motor = hwMap.get(DcMotorEx.class, "outtake");
        pivservo = hwMap.get(Servo.class, "outtakePivot");
        servo = hwMap.get(Servo.class, "dump");

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0);
        synchronized (motorModeLock) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Sets the slide to move. Intended for TeleOp.
     * @param extend whether to extend or retract the slide
     */
    public void slide(boolean extend, boolean overextend) {
        boolean isOverextension = (extend && motor.getCurrentPosition() >= SLIDE_MAX_POSITION)
                || (!extend && motor.getCurrentPosition() <= SLIDE_MIN_POSITION);
        if (!isOverextension || overextend) {
            motor.setVelocity(extend ? SLIDE_UP_SPEED : SLIDE_DOWN_SPEED);
        }
        else {
            Match.status("Overextension");
            motor.setPower(0);
        }
    }

    /**
     * Moves the slide outward. Intended for TeleOp.
     */
    public void extend(boolean overextend) {
        slide(true, overextend);
    }

    /**
     * Moves the slide inward. Intended for TeleOp.
     */
    public void retract(boolean overextend) {
        slide(false, overextend);
    }

    /**
     * Sets the run mode to {@code RUN_USING_ENCODER} and stop the motor.
     * This method can stop {@code RUN_TO_POSITION}.
     */
    public void resetSlide() {
        synchronized (motorModeLock) {
            motor.setVelocity(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
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
                slideToPositionAsync(SLIDE_TOP_POSITION);
                break;
            case MID:
                slideToPositionAsync(SLIDE_MID_POSITION);
                break;
            case EXCESS:
                slideToPositionAsync(SLIDE_EXCESS_POSITION);
                break;
            default:
            case BOT:
                slideToPositionAsync(SLIDE_BOT_POSITION);
                break;
            case ZERO:
                slideToPositionAsync(0);
                break;
        }
    }

    /**
     * Moves the slide to a hub level / tier synchronously
     * @param barcode destination hub level / tier
     */
    public void slideTo(Barcode barcode) {
        slideToAsync(barcode);
        Telemetry.Item currentPositionItem = Match.createLogItem("Outtake - current position", motor.getCurrentPosition());
        Telemetry.Item targetPositionItem = Match.createLogItem("Outtake - target position", motor.getTargetPosition());
        Match.log("Outtake slideTo() loop begins");
        while (motor.isBusy() && !Thread.currentThread().isInterrupted()) {
            Match.status("Outtake looping");
            Objects.requireNonNull(currentPositionItem).setValue(motor.getCurrentPosition());
            Match.update();
        }
        Match.status("Outtake loop done");
        Match.log("Outtake slideTo() loop ends");
        Match.removeLogItem(currentPositionItem);
        Match.removeLogItem(targetPositionItem);
        resetSlide();
    }

    /**
     * Treats current encoder position as zero
     */
    public void setInitialPos() {
        synchronized (motorModeLock) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Synchronously move the slide to an encoder position. Interrupt-aware.
     * @param position destination encoder position
     */
    public void slideToPositionAsync(int position) {
        synchronized (motorModeLock) {
            motor.setTargetPosition(position);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (motor.getCurrentPosition() < position) {
                motor.setVelocity(SLIDE_UP_SPEED);
            } else {
                motor.setVelocity(SLIDE_DOWN_SPEED);
            }
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
    public Servo getServo2() {
        return pivservo;
    }

    /**
     * Asynchronously toggles the position of the dump (in/out). This method is oblivious to the
     * current position of the servo and decides which position to turn to based on an assumed
     * internal state (defaults to closed at initialization).
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
     * Asynchronously turns the dump inward.
     */
    public void dumpIn() {
        isDumpOut = false;
        pivservo.setPosition(DUMP_IN_POSITION);
        servo.setPosition(1.025-DUMP_IN_POSITION*2/3);
    }

    /**
     * Asynchronously turns the dump outward.
     */
    public void dumpOut() {
        isDumpOut = true;
        pivservo.setPosition(DUMP_OUT_POSITION);
        servo.setPosition(1.025-DUMP_OUT_POSITION*2/3);
    }

    /**
     * Asynchronously turns the dump inward to dispose of the extra game element.
     */
    public void dumpExcess() {
        isDumpOut = false;
        servo.setPosition(DUMP_EXCESS_POSITION);
    }

    /**
     * Returns whether the motor has reached its target position.
     * @return whether the motor has reached its target position
     */
    public boolean reachedTargetPosition() {
        return Math.abs(motor.getCurrentPosition() - motor.getTargetPosition()) < motor.getTargetPositionTolerance();
    }
}

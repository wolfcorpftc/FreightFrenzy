package org.wolfcorp.ff.robot;
import static org.wolfcorp.ff.vision.Barcode.BOT;
import static org.wolfcorp.ff.vision.Barcode.DIRTY;
import static org.wolfcorp.ff.vision.Barcode.EXCESS;
import static org.wolfcorp.ff.vision.Barcode.MID;
import static org.wolfcorp.ff.vision.Barcode.TOP;
import static org.wolfcorp.ff.vision.Barcode.ZERO;
import static org.wolfcorp.ff.vision.Barcode.SUPERTOP;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Function;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.wolfcorp.ff.opmode.OpMode;
import org.wolfcorp.ff.opmode.util.Match;
import org.wolfcorp.ff.vision.Barcode;

import java.util.Objects;

public class Outtake {
    public static final double SLIDE_TICKS_PER_REV = 1425.1;
    public static final double SLIDE_MAX_SPEED = 117 / 60.0 * SLIDE_TICKS_PER_REV; // ticks/sec
    public static final double SLIDE_UP_SPEED = SLIDE_MAX_SPEED; // ticks/sec
    public static final double SLIDE_DOWN_SPEED = -SLIDE_MAX_SPEED; // ticks/sec

    public static final int SLIDE_TOP_POSITION = 1800;
    public static final int SLIDE_SUPERTOP_POSITION = 1900;
    public static final int SLIDE_MID_POSITION = 1000;
    public static final int SLIDE_EXCESS_POSITION = 80;
    public static final int SLIDE_BOT_POSITION = 160;

    public static final int SLIDE_MIN_POSITION = 0;
    public static final int SLIDE_MAX_POSITION = 2100;

    public static final double DUMP_EXCESS_POSITION = 0.96;
    public static final double DUMP_IN_POSITION = 0.93;
    public static final double DUMP_OUT_POSITION = 0.51;

    public static final double DUMP_OVERFLOW_DIST = 1.65;
    public static final double DUMP_FULL_DIST = 1.8;

    /** Milliseconds to wait for after running dumpOut. */
    public static final int DUMP_DELAY = 0;

    private final DcMotorEx motor; // slide motor
    private final Servo servo; // dump servo

    private final Object motorModeLock = new Object();
    private boolean isDumpOut = false;

    public Outtake(HardwareMap hwMap) {
        motor = hwMap.get(DcMotorEx.class, "outtake");
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
            if (extend && motor.getCurrentPosition() > (SLIDE_BOT_POSITION + SLIDE_MIN_POSITION) / 2
                    && motor.getCurrentPosition() < SLIDE_BOT_POSITION / 2) {
            } else if (motor.getCurrentPosition() > SLIDE_BOT_POSITION / 2 ) {
            } else {
                dumpIn();
            }
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
        slideToPositionAsync(barcodeToPosition(barcode));
    }

    public static int barcodeToPosition(Barcode barcode) {
        switch (barcode) {
            case SUPERTOP:
                return SLIDE_SUPERTOP_POSITION;
            case TOP:
                return SLIDE_TOP_POSITION;
            case MID:
                return SLIDE_MID_POSITION;
            case EXCESS:
                return SLIDE_EXCESS_POSITION;
            case BOT:
                return SLIDE_BOT_POSITION;
            case ZERO:
                return 0;
            default:
                return Integer.MIN_VALUE;
        }
    }

    public static Barcode positionToBarcode(int pos) {
        switch (pos) {
            case SLIDE_SUPERTOP_POSITION:
                return SUPERTOP;
            case SLIDE_TOP_POSITION:
                return TOP;
            case SLIDE_MID_POSITION:
                return MID;
            case SLIDE_EXCESS_POSITION:
                return EXCESS;
            case SLIDE_BOT_POSITION:
                return BOT;
            case 0:
                return ZERO;
            default:
                return DIRTY;
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
        while (motor.isBusy() && OpMode.isActive()) {
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

    public boolean isDumpOut() {
        return isDumpOut;
    }

    /**
     * Asynchronously turns the dump inward.
     */
    public void dumpIn() {
        isDumpOut = false;
        servo.setPosition(DUMP_IN_POSITION);
    }

    /**
     * Asynchronously turns the dump outward.
     */
    public void dumpOut() {
        isDumpOut = true;
        servo.setPosition(DUMP_OUT_POSITION);
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

    public Barcode getSlidePosition() {
        int pos = motor.getCurrentPosition();
        Function<Integer, Boolean> inRange = (Integer expectedPos) -> Math.abs(pos - expectedPos) < 10;
        for (Barcode barcode : Barcode.values()) {
            if (inRange.apply(barcodeToPosition(barcode)))
                return barcode;
        }
        return DIRTY;
    }

    public Barcode getSlideTarget() {
        return positionToBarcode(motor.getTargetPosition());
    }

    public boolean isSlideActiveTarget(Barcode barcode) {
        return motor.isBusy() && getSlideTarget() == barcode;
    }

    public boolean isApproaching(Barcode barcode) {
        return getSlidePosition() == barcode || isSlideActiveTarget(barcode);
    }
}

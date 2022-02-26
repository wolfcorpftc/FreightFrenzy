package org.wolfcorp.ff.robot;

import static org.wolfcorp.ff.vision.Barcode.BOT;
import static org.wolfcorp.ff.vision.Barcode.DIRTY;
import static org.wolfcorp.ff.vision.Barcode.MID;
import static org.wolfcorp.ff.vision.Barcode.TOP;
import static org.wolfcorp.ff.vision.Barcode.ZERO;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Function;
import org.wolfcorp.ff.opmode.OpMode;
import org.wolfcorp.ff.opmode.util.Match;
import org.wolfcorp.ff.vision.Barcode;

public class Outtake {
    public static final double SLIDE_TICKS_PER_REV = 1425.1;
    public static final double SLIDE_MAX_SPEED = 117 / 60.0 * SLIDE_TICKS_PER_REV; // ticks/sec
    public static final double SLIDE_UP_VEL = SLIDE_MAX_SPEED; // ticks/sec
    public static final double SLIDE_DOWN_VEL = -SLIDE_MAX_SPEED; // ticks/sec

    public static final int SLIDE_TOP_POSITION = 1500;
    public static final int SLIDE_MID_POSITION = 1350; // FIXME: tune
    public static final int SLIDE_BOT_POSITION = 1050; // FIXME: tune

    public static final int SLIDE_MIN_POSITION = -100; // FIXME: tune, or get rid of it
    public static final int SLIDE_MAX_POSITION = 2100; // FIXME: tune, or get rid of it

    public static final double PIVOT_IN_POSITION = 0.04;
    public static final double PIVOT_OUT_TOP_POSITION = 0.6;
    public static final double PIVOT_OUT_MID_POSITION = 0; // FIXME: tune
    public static final double PIVOT_OUT_BOT_POSITION = 0; // FIXME: tune

    public static final double DUMP_OUT_TOP_POSITION = 1 - PIVOT_OUT_TOP_POSITION;
    public static final double DUMP_OUT_MID_POSITION = 1 - PIVOT_OUT_MID_POSITION; // FIXME: tune
    public static final double DUMP_OUT_BOT_POSITION = 1 - PIVOT_OUT_BOT_POSITION; // FIXME: tune
    public static final double DUMP_IN_POSITION = 1.03 - PIVOT_IN_POSITION;
    public static final double DUMP_DROP_POSITION = 0;
    public static final double DUMP_DROP_SHARED_POSITION = 0.6;

    public static final double DUMP_OVERFLOW_DIST = 1.60; // FIXME: tune
    public static final double DUMP_FULL_DIST = 1.60; // FIXME: tune

    private final DcMotorEx slide;
    private final Servo dump;
    private final Servo pivot;

    /** Lock that prevents simultaneous mode change race condition */
    private final Object slideModeLock = new Object();
    /** Lock that prevents simultaneous execution of {@link #cycle(Barcode)} or its asynchronous variant. */
    private final Object cycleLock = new Object();
    /** Lock that prevents simultaneous execution of {@link #in()}, {@link #out(Barcode)}, or their asynchronous variants. */
    private final Object cycleStepLock = new Object();

    public Outtake(HardwareMap hwMap) {
        slide = hwMap.get(DcMotorEx.class, "outtake");
        pivot = hwMap.get(Servo.class, "pivot");
        dump = hwMap.get(Servo.class, "dump");

        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setPower(0);
        synchronized (slideModeLock) {
            slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Sets the slide to move. Intended for TeleOp.
     *
     * @param extend whether to extend or retract the slide
     */
    public void slide(boolean extend, boolean overextend) {
        boolean isOverextension = (extend && slide.getCurrentPosition() >= SLIDE_MAX_POSITION)
                || (!extend && slide.getCurrentPosition() <= SLIDE_MIN_POSITION);
        if (!isOverextension || overextend) {
            slide.setVelocity(extend ? SLIDE_UP_VEL : SLIDE_DOWN_VEL);
            if (extend && slide.getCurrentPosition() < 300) {
                pivot.setPosition(0);
            }
        } else {
            Match.status("Overextension");
            slide.setPower(0);
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
     * Wait until the slide is not moving toward a target position, and resets the slide.
     */
    public void waitForSlide() {
        while (slide.isBusy()) ;
        resetSlideMode();
    }

    /**
     * Sets the run mode to {@code RUN_USING_ENCODER} and stop the motor.
     * This method can stop {@code RUN_TO_POSITION}.
     */
    public void resetSlideMode() {
        synchronized (slideModeLock) {
            slide.setVelocity(0);
            slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Sets the slide to run to a position asynchronously. <b>Warning: </b> This does not reset the
     * run mode to {@code RUN_USING_ENCODER}. To interrupt or get out of {@code RUN_TO_POSITION}, use
     * {@link Outtake#resetSlideMode()}.
     *
     * @param barcode target level to slide to / barcode scan result
     */
    public void slideToAsync(Barcode barcode) {
        resetSlideMode(); // to make sure that motor is not in RUN_TO_POSITION
        slideToPositionAsync(barcodeToPosition(barcode));
    }

    /**
     * Converts a shipping hub level (barcode) to a slide encoder count.
     *
     * @param barcode shipping hub level
     * @return slide encoder count that corresponds with the given hub level
     */
    public static int barcodeToPosition(Barcode barcode) {
        switch (barcode) {
            case TOP:
                return SLIDE_TOP_POSITION;
            case MID:
                return SLIDE_MID_POSITION;
            case BOT:
                return SLIDE_BOT_POSITION;
            case ZERO:
                return 0;
            default:
                return Integer.MIN_VALUE;
        }
    }

    /**
     * Converts a slide encoder count to its corresponding shipping hub level.
     *
     * @param pos slide encoder count
     * @return shipping hub level that corresponds to given position
     * @implNote Zero error tolerance.
     */
    public static Barcode positionToBarcode(int pos) {
        switch (pos) {
            case SLIDE_TOP_POSITION:
                return TOP;
            case SLIDE_MID_POSITION:
                return MID;
            case SLIDE_BOT_POSITION:
                return BOT;
            case 0:
                return ZERO;
            default:
                return DIRTY;
        }
    }

    /**
     * Moves the slide to a hub level.
     *
     * @param barcode desired hub level
     */
    public void slideTo(Barcode barcode) {
        slideToAsync(barcode);
        while (slide.isBusy() && OpMode.isActive()) ;
        resetSlideMode();
    }

    /**
     * Treats current encoder position as zero.
     */
    public void resetSlideEncoder() {
        synchronized (slideModeLock) {
            slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Asynchronously move the slide to an encoder position.
     *
     * @param position destination encoder position
     */
    public void slideToPositionAsync(int position) {
        synchronized (slideModeLock) {
            slide.setTargetPosition(position);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (slide.getCurrentPosition() < position) {
                slide.setVelocity(SLIDE_UP_VEL);
            } else {
                slide.setVelocity(Math.abs(SLIDE_DOWN_VEL));
            }
        }
    }


    /**
     * Returns the slide motor.
     *
     * @return slide motor
     */
    public DcMotorEx getSlide() {
        return slide;
    }

    /**
     * Returns the dump (a.k.a. bucket) servo.
     *
     * @return dump (a.k.a. bucket) servo
     */
    public Servo getDump() {
        return dump;
    }

    /**
     * Returns the pivot servo.
     *
     * @return pivot servo
     */
    public Servo getPivot() {
        return pivot;
    }

    /**
     * Discards any excess freight.
     */
    public void ridExcess() {
        // TODO: implement
    }

    /**
     * Returns whether the motor has reached its target position.
     *
     * @return whether the motor has reached its target position
     */
    public boolean isAtTargetPosition() {
        return Math.abs(slide.getCurrentPosition() - slide.getTargetPosition()) < slide.getTargetPositionTolerance();
    }

    /**
     * Returns the hub level that corresponds to current slide position.
     *
     * @return hub level corresponding to current slide position. {@link Barcode#DIRTY} if not found.
     */
    public Barcode getSlidePosition() {
        int pos = slide.getCurrentPosition();
        Function<Integer, Boolean> isInRange = (Integer expectedPos) -> Math.abs(pos - expectedPos) < 10;
        for (Barcode barcode : Barcode.values()) {
            if (isInRange.apply(barcodeToPosition(barcode)))
                return barcode;
        }
        return DIRTY;
    }

    /**
     * Returns the hub level that the slide is moving toward.
     *
     * @return hub level that the slide is moving toward
     */
    public Barcode getSlideTargetLevel() {
        return positionToBarcode(slide.getTargetPosition());
    }

    /**
     * Returns whether the slide is moving toward a given hub level.
     *
     * @param barcode hub level to be checked for
     * @return whether the slide is moving toward a given hub level
     */
    public boolean isSlidingTo(Barcode barcode) {
        return slide.isBusy() && getSlideTargetLevel() == barcode;
    }

    /**
     * Returns whether the slide is at or is sliding toward a hub level.
     *
     * @param barcode hub level to be checked for
     * @return whether the slide is at or is sliding toward a hub level
     */
    public boolean isAtOrSlidingTo(Barcode barcode) {
        return getSlidePosition() == barcode || isSlidingTo(barcode);
    }

    /**
     * Scores one freight into the designated hub level.
     *
     * @param barcode hub level to score into
     * @see #cycleAsync(Barcode)
     */
    public void cycle(Barcode barcode) {
        synchronized (cycleLock) {
            out(barcode);
            OpMode.waitFor(600);
            drop();
            OpMode.waitFor(800);
            in();
        }
    }

    /**
     * Moves the outtake to scoring position. Call {@link #drop()} to drop the freight.
     *
     * @param barcode barcode corresponding to the desired shipping hub level
     * @implNote Does not wait for the movement to finish.
     * @see #outAsync(Barcode)
     */
    public void out(Barcode barcode) {
        synchronized (cycleStepLock) {
            dump.setPosition(0.7);
            OpMode.waitFor(100);
            slideToAsync(barcode);
            OpMode.waitFor(100);
            double pivotPosition;
            double dumpPosition;
            switch (barcode) {
                case BOT:
                    pivotPosition = PIVOT_OUT_BOT_POSITION;
                    dumpPosition = DUMP_OUT_BOT_POSITION;
                    break;
                case MID:
                    pivotPosition = PIVOT_OUT_MID_POSITION;
                    dumpPosition = DUMP_OUT_MID_POSITION;
                    break;
                default:
                case TOP:
                    pivotPosition = PIVOT_OUT_TOP_POSITION;
                    dumpPosition = DUMP_OUT_TOP_POSITION;
                    break;
            }
            pivot.setPosition(pivotPosition);
            dump.setPosition(dumpPosition);
        }
    }

    /**
     * Asynchronously lower the dump bucket to drop cargo. Must call {@link #out(Barcode)} before it.
     */
    public void drop() {
        synchronized (cycleStepLock) {
            dump.setPosition(DUMP_DROP_POSITION);
        }
    }

    /**
     * Asynchronously raise the dump bucket after dropping cargo. Must call {@link #out(Barcode)} before it.
     */
    public void undrop() {
        synchronized (cycleStepLock) {
            dump.setPosition(DUMP_IN_POSITION); // FIXME: Add other in positions
        }
    }

    /**
     * Return boolean value of whether the dump is in drop position. Call {@link #toggleDump()} to flip dump state.
     */
    public boolean isDumpOut() {
        return (Math.abs(dump.getPosition() - DUMP_DROP_POSITION) < 0.0001 || Math.abs(dump.getPosition() - DUMP_DROP_SHARED_POSITION) < 0.0001); // FIXME: Add other possible out positions
    }

    /**
     * Toggle dump.
     */
    public void toggleDump() {
        if (isDumpOut()) {
            undrop();
        } else if (pivot.getPosition() < 0.2) {
            dump.setPosition(DUMP_DROP_SHARED_POSITION);
        } else {
            drop();
        }
    }

    /**
     * Moves the outtake to intake position.
     *
     * @implNote Waits for slide movement to finish.
     * @see #inAsync()
     */
    public void in() {
        synchronized (cycleStepLock) {
            slideToAsync(ZERO); // DOES NOT BELONG, make sure the resulting method is async
            OpMode.waitFor(75);
            dump.setPosition(DUMP_IN_POSITION);
            pivot.setPosition(PIVOT_IN_POSITION);
            waitForSlide();
        }
    }

    /**
     * Asynchronously scores one freight into the designated hub level.
     *
     * @param barcode hub level to score into
     * @return a thread running {@link #cycle(Barcode)}
     * @see #cycle(Barcode)
     */
    public Thread cycleAsync(Barcode barcode) {
        Thread t = new Thread(() -> cycle(barcode));
        t.start();
        return t;
    }

    // FIXME: test if it works for TeleOp

    /**
     * Asynchronously moves the outtake to intake position.
     *
     * @return a thread running {@link #in}
     * @see #in()
     */
    public Thread inAsync() {
        Thread t = new Thread(this::in);
        t.start();
        return t;
    }

    // FIXME: test if it works for TeleOp

    /**
     * Asynchronously moves the outtake to outtake position. Call {@link #drop()} to drop the freight.
     *
     * @return a thread running {@link #out}
     * @see #out(Barcode)
     */
    public Thread outAsync(Barcode barcode) {
        Thread t = new Thread(() -> out(barcode));
        t.start();
        return t;
    }
}

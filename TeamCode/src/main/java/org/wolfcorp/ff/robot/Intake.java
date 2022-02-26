package org.wolfcorp.ff.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.wolfcorp.ff.opmode.util.Match;

public class Intake {
    /** Encoder ticks per motor revolution */
    public static final double TICKS_PER_REV = 103.8;
    /** Theoretical maximum speed of motor */
    public static final double MAX_SPEED = 1620 / 60.0 * TICKS_PER_REV; // ticks/sec
    /** Default inward speed in ticks per second. */
    public static final double IN_VEL = -0.65 * MAX_SPEED; // ticks/sec
    /** Default outward speed in ticks per second. */
    public static final double OUT_VEL = 0.4 * MAX_SPEED; // ticks/sec;

    /** Motor facing the warehouse */
    private final DcMotorEx front;
    /** Motor facing away from the warehouse */
    private final DcMotorEx back;

    /** Lock object that prevents motor mode change race condition */
    private final Object motorModeLock = new Object();

    public Intake(HardwareMap hwMap) {
        // FIXME: front back motor switching based on alliance
        if (Match.RED) {
            front = hwMap.get(DcMotorEx.class, "frontIntake");
            back = hwMap.get(DcMotorEx.class, "backIntake");
        } else {
            back = hwMap.get(DcMotorEx.class, "frontIntake");
            front = hwMap.get(DcMotorEx.class, "backIntake");
        }
        front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front.setDirection(DcMotor.Direction.FORWARD);
        back.setDirection(DcMotor.Direction.FORWARD);
        front.setPower(0);
        back.setPower(0);
        front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    /**
     * Turns on both intake motors.
     *
     * @param f speed of motor facing the warehouse as a fraction of default intake speed
     * @param b speed of motor facing away from warehouse as a fraction of default intake speed
     */
    public void in(double f, double b) {
        front.setVelocity(f * IN_VEL);
        back.setVelocity(b * IN_VEL);
    }

    /**
     * Turns on both intake motors
     *
     * @param s speed of motors as a fraction of default intake speed
     */
    public void in(double s) {
        in(s, s);
    }

    /**
     * Turns on both intake motors with default intake speed.
     */
    public void in() {
        in(1.0);
    }

    /**
     * Toggles the intake motors to take in game elements.
     */
    public void toggleIn() {
        if (isOn()) {
            off();
        } else {
            front.setVelocity(IN_VEL);
            back.setVelocity(IN_VEL);
        }
    }

    /**
     * Rotates both intake motors inward for a given number of revolutions.
     *
     * @param revs the number of revolutions
     */
    public void in(int revs) {
        synchronized (motorModeLock) {
            front.setTargetPosition((int) (front.getCurrentPosition() + revs * TICKS_PER_REV));
            back.setTargetPosition((int) (back.getCurrentPosition() + revs * TICKS_PER_REV));
            front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            in();
        }
        while (front.isBusy() && back.isBusy() && !Thread.currentThread().isInterrupted());
        synchronized (motorModeLock) {
            off();
            front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Rotates both motors outward for a given number of revolutions.
     *
     * @param revs the number of revolutions
     */
    public void out(int revs) {
        synchronized (motorModeLock) {
            front.setTargetPosition((int) (front.getCurrentPosition() - revs * TICKS_PER_REV));
            back.setTargetPosition((int) (back.getCurrentPosition() - revs * TICKS_PER_REV));
            front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            out();
        }
        while (front.isBusy() && !Thread.currentThread().isInterrupted());
        synchronized (motorModeLock) {
            off();
            front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Makes the intake spit out / repel game elements.
     */
    public void out() {
        front.setVelocity(OUT_VEL);
        back.setVelocity(OUT_VEL);
    }

    /**
     * Makes the intake spit out / repel game elements in the warehouse direction.
     */
    public void directedOut() {
        front.setVelocity(OUT_VEL);
        back.setVelocity(IN_VEL); // prevents game element from exiting warehouse illegally
    }

    /**
     * Toggles the intake to repel game elements. Intended for TeleOp.
     */
    public void toggleOut() {
        if (isOn()) {
            off();
        } else {
            out();
        }
    }

    /**
     * Toggles the intake to repel game elements in the warehouse direction. Intended for TeleOp.
     * @see #directedOut()
     */
    public void toggleDirectedOut() {
        if (isOn()) {
            off();
        } else {
            directedOut();
        }
    }

    /**
     * Turns both intakes off by setting velocity to zero.
     */
    public void off() {
        front.setVelocity(0);
        back.setVelocity(0);
    }

    /**
     * Returns whether the intake motor is on.
     *
     * @return whether the intake motor is on
     */
    public boolean isOn() {
        return Math.abs(front.getVelocity()) > 10;
    }

    /**
     * Returns the intake motor facing the warehouse. Alliance is taken into account.
     *
     * @return the intake motor facing the warehouse
     */
    public DcMotorEx getFront() {
        return front;
    }

    /**
     * Returns the intake motor facing away from warehouse. Alliance is taken into account.
     *
     * @return the intake motor facing away from warehouse
     */
    public DcMotorEx getBack() {
        return back;
    }
}

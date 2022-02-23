package org.wolfcorp.ff.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.wolfcorp.ff.opmode.util.Match;

public class Intake {
    public static final double TICKS_PER_REV = 103.8;
    public static final int INTAKE_REVS = 20;
    public static final double MAX_SPEED = 1620 / 60.0 * TICKS_PER_REV; // ticks/sec
    public static final double IN_SPEED = -0.365 * MAX_SPEED; // ticks/sec
    public static final double OUT_SPEED = 0.4 * MAX_SPEED; // ticks/sec;

    private final DcMotorEx front;
    private final DcMotorEx back;

    private final Object motorModeLock = new Object();

    public Intake(HardwareMap hwMap) {
        front = hwMap.get(DcMotorEx.class, "frontIntake");
        back = hwMap.get(DcMotorEx.class, "backIntake");
        front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front.setDirection(DcMotor.Direction.FORWARD);
        back.setDirection(DcMotor.Direction.FORWARD);
        front.setPower(0);
        back.setPower(0);
        synchronized (motorModeLock) {
            front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Turns on both intake motors.
     */
    public void in(double f, double b) {
        front.setVelocity(f * IN_SPEED);
        back.setVelocity(b * IN_SPEED);
    }

    public void in(double s) {
        in(s, s);
    }

    public void in() {
        in(IN_SPEED);
    }

    /**
     * Toggles the intake to take in game elements. Intended for TeleOp.
     */
    public void toggleIn() {
        if (isOn()) {
            off();
        } else {
            front.setVelocity(IN_SPEED);
            back.setVelocity(IN_SPEED);
        }
    }

    /**
     * Rotates the motor inward for a given number of revolutions. <b>WARNING: </b> Running
     * {@link #in(int)} and {@link #out(int)} at the same time may result in undefined behavior.
     *
     * @param revs the number of revolutions
     */
    public void in(int revs) {
        synchronized (motorModeLock) {
            front.setTargetPosition((int) (front.getCurrentPosition() + revs * TICKS_PER_REV));
            back.setTargetPosition((int) (back.getCurrentPosition() + revs * TICKS_PER_REV));
            front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front.setVelocity(IN_SPEED);
            back.setVelocity(IN_SPEED);
        }
        // TODO: remove message after debugging
        Telemetry.Item positionItem = Match.createLogItem("Intake position", front.getCurrentPosition());
        Telemetry.Item positionItem2 = Match.createLogItem("Intaake position", back.getCurrentPosition());
        while (front.isBusy() && back.isBusy() && !Thread.currentThread().isInterrupted()) {
            positionItem.setValue(front.getCurrentPosition());
            positionItem2.setValue(back.getCurrentPosition());
            Match.update();
        }
        Match.removeLogItem(positionItem);
        synchronized (motorModeLock) {
            front.setPower(0);
            front.setPower(2);
            front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Rotates the motor outward for a given number of revolutions. <b>WARNING: </b> Running
     * {@link #in(int)} and {@link #out(int)} at the same time may result in undefined behavior.
     *
     * @param revs the number of revolutions
     */
    public void out(int revs) {
        synchronized (motorModeLock) {
            front.setTargetPosition((int) (front.getCurrentPosition() - revs * TICKS_PER_REV));
            back.setTargetPosition((int) (back.getCurrentPosition() - revs * TICKS_PER_REV));
            front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front.setVelocity(OUT_SPEED);
            back.setVelocity(OUT_SPEED);
        }
        while (front.isBusy() && !Thread.currentThread().isInterrupted());
        synchronized (motorModeLock) {
            front.setPower(0);
            back.setPower(0);
            front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Makes the intake spit out / repel game elements. Intended for TeleOp.
     */
    public void out() {
        front.setVelocity(OUT_SPEED);
        back.setVelocity(OUT_SPEED);
    }

    /**
     * Toggles the intake to repel game elements. Intended for TeleOp.
     */
    public void toggleOut() {
        if (isOn()) {
            off();
        } else {
            front.setVelocity(OUT_SPEED);
            back.setVelocity(OUT_SPEED);
        }
    }

    /**
     * Turns intake off by setting velocity to zero.
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
     * Sets motor velocity in RPM.
     *
     * @param rpm velocity in RPM
     */
    public void setVelocityRPM(double rpm) {
        front.setVelocity(rpm / 60.0 * TICKS_PER_REV);
        back.setVelocity(rpm / 60.0 * TICKS_PER_REV);
    }

    /**
     * Returns the intake motor object.
     *
     * @return the intake motor object
     */
    public DcMotorEx getFront() {
        return front;
    }
    public DcMotorEx getBack() {
        return back;
    }
}

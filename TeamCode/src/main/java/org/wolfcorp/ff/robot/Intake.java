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
    public static final double IN_SPEED = -0.4 * MAX_SPEED; // ticks/sec
    public static final double OUT_SPEED = 0.6 * MAX_SPEED; // ticks/sec;

    private final DcMotorEx motor;

    private final Object motorModeLock = new Object();

    public Intake(HardwareMap hwMap) {
        motor = hwMap.get(DcMotorEx.class, "intake");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setPower(0);
        synchronized (motorModeLock) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    // FIXME: add comment and ensure all usages are correct
    public void in() {
        motor.setVelocity(IN_SPEED);
    }

    /**
     * Toggles the intake to take in game elements. Intended for TeleOp.
     */
    public void toggleIn() {
        toggleIn(1);
    }

    /** Toggle intake with custom speed ***/
    public void toggleIn(double factor) {
        if (isOn()) {
            off();
        } else {
            motor.setVelocity(IN_SPEED * factor);
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
            motor.setTargetPosition((int) (motor.getCurrentPosition() + revs * TICKS_PER_REV));
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setVelocity(IN_SPEED);
        }
        // TODO: remove message after debugging
        Telemetry.Item positionItem = Match.createLogItem("Intake position", motor.getCurrentPosition());
        while (motor.isBusy() && !Thread.currentThread().isInterrupted()) {
            positionItem.setValue(motor.getCurrentPosition());
            Match.update();
        }
        Match.removeLogItem(positionItem);
        synchronized (motorModeLock) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
            motor.setTargetPosition((int) (motor.getCurrentPosition() - revs * TICKS_PER_REV));
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setVelocity(OUT_SPEED);
        }
        while (motor.isBusy() && !Thread.currentThread().isInterrupted());
        synchronized (motorModeLock) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Makes the intake spit out / repel game elements. Intended for TeleOp.
     */
    public void out() {
        out(1.0);
    }

    public void out(double factor) {
        motor.setVelocity(OUT_SPEED * factor);
    }

    /**
     * Toggles the intake to repel game elements. Intended for TeleOp.
     */
    public void toggleOut() {
        if (isOn()) {
            off();
        } else {
            motor.setVelocity(OUT_SPEED);
        }
    }

    /**
     * Turns intake off by setting velocity to zero.
     */
    public void off() {
        motor.setVelocity(0);
    }

    /**
     * Returns whether the intake motor is on.
     *
     * @return whether the intake motor is on
     */
    public boolean isOn() {
        return Math.abs(motor.getVelocity()) > 10;
    }

    /**
     * Sets motor velocity in RPM.
     *
     * @param rpm velocity in RPM
     */
    public void setVelocityRPM(double rpm) {
        motor.setVelocity(rpm / 60.0 * TICKS_PER_REV);
    }

    /**
     * Returns the intake motor object.
     *
     * @return the intake motor object
     */
    public DcMotorEx getMotor() {
        return motor;
    }
}

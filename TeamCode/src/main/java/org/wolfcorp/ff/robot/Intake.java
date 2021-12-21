package org.wolfcorp.ff.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.wolfcorp.ff.opmode.Match;

public class Intake {
    public static final double TICKS_PER_REV = 103.8;
    public static final int INTAKE_REVS = 20;
    public static final double MAX_SPEED = 1620 / 60.0 * TICKS_PER_REV; // ticks/sec
    public static final double IN_SPEED = 0.4 * MAX_SPEED; // ticks/sec
    public static final double OUT_SPEED = -0.25 * MAX_SPEED; // ticks/sec; TODO: tune

    private final DcMotorEx motor;

    public Intake(HardwareMap hwMap) {
        motor = hwMap.get(DcMotorEx.class, "intake");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Toggles the intake to take in game elements. Intended for TeleOp.
     */
    public void in() {
        if (isOn()) {
            off();
        } else {
            motor.setVelocity(IN_SPEED);
        }
    }

    /**
     * Rotates the motor inward for a given number of revolutions.
     *
     * @param revs the number of revolutions
     */
    public void in(int revs) {
        motor.setTargetPosition((int) (motor.getCurrentPosition() + revs * TICKS_PER_REV));
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setVelocity(IN_SPEED);
        // TODO: remove message after debugging
        Telemetry.Item positionItem = Match.createLogItem("Intake position", motor.getCurrentPosition());
        while (motor.isBusy() && !Thread.currentThread().isInterrupted()) {
            positionItem.setValue(motor.getCurrentPosition());
            Match.update();
        }
        Match.removeLogItem(positionItem);
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Rotates the motor outward for a given number of revolutions.
     *
     * @param revs the number of revolutions
     */
    public void out(int revs) {
        motor.setTargetPosition((int) (motor.getCurrentPosition() - revs * TICKS_PER_REV));
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setVelocity(OUT_SPEED);
        while (motor.isBusy() && !Thread.currentThread().isInterrupted());
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Toggles the intake to repel game elements. Intended for TeleOp.
     */
    public void out() {
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

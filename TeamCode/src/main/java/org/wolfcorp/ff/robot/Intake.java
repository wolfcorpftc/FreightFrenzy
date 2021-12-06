package org.wolfcorp.ff.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    public static final double TICKS_PER_REV = 103.8;
    public static final double MAX_SPEED = 1620 / 60.0 * TICKS_PER_REV; // ticks/sec
    public static final double IN_SPEED = 0.4 * MAX_SPEED; // ticks/sec
    public static final double OUT_SPEED = -0.1 * MAX_SPEED; // ticks/sec; TODO: tune

    private final DcMotorEx motor;

    public Intake(HardwareMap hwMap) {
        motor = hwMap.get(DcMotorEx.class, "intake");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
     * @return whether the intake motor is on
     */
    public boolean isOn() {
        return Math.abs(motor.getVelocity()) > 10;
    }

    /**
     * Set motor velocity in RPM
     *
     * @param rpm velocity in RPM
     */
    public void setVelocityRPM(double rpm) {
        motor.setVelocity(rpm / 60.0 * TICKS_PER_REV);
    }

    /**
     * @return the intake motor object
     */
    public DcMotor getMotor() {
        return motor;
    }
}

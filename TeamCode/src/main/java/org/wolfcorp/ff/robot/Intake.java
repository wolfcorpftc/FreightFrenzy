package org.wolfcorp.ff.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    public static double TICKS_PER_REV = 103.8;
    public static double INTAKE_MAX_SPEED = 1620;
    public static double INTAKE_OPTIMAL_SPEED = 640 / 60.0 * TICKS_PER_REV;
    public static double INTAKE_IN_SPEED = INTAKE_OPTIMAL_SPEED;
    public static double INTAKE_OUT_SPEED = -0.3 * INTAKE_OPTIMAL_SPEED; // TODO: tune

    private DcMotorEx motor;

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
        }
        else {
            motor.setVelocity(INTAKE_IN_SPEED);
        }
    }

    /**
     * Toggles the intake to repel game elements. Intended for TeleOp.
     */
    public void out() {
        if (isOn()) {
            off();
        }
        else {
            motor.setVelocity(INTAKE_OUT_SPEED);
        }
    }

    /**
     * Turns intake off
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
     * @param vel velocity in RPM
     */
    public void setVelocityRPM(double vel) {
        motor.setVelocity(vel / 60.0 * TICKS_PER_REV);
    }

    public DcMotor getMotor() {
        return motor;
    }
}

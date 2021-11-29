package org.wolfcorp.ff.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.wolfcorp.ff.opmode.OpMode;

public class Intake {
    public static double TICKS_PER_REV = 103.8;
    public static double INTAKE_MAX_SPEED = 1620;
    public static double INTAKE_OPTIMAL_SPEED = 640 / 60.0 * TICKS_PER_REV;

    private DcMotorEx motor;

    public Intake(HardwareMap hwMap) {
        motor = hwMap.get(DcMotorEx.class, "intake");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Toggle the intake to take in game elements. Intended for TeleOp.
     */
    public void in() {
        if (isOn()) {
            off();
        }
        else {
            motor.setVelocity(INTAKE_OPTIMAL_SPEED);
        }
    }

    /**
     * Toggle the intake to repel game elements. Intended for TeleOp.
     */
    public void out() {
        if (isOn()) {
            off();
        }
        else {
            motor.setVelocity(-INTAKE_OPTIMAL_SPEED);
        }
    }

    /**
     * Turn intake off
     */
    public void off() {
        motor.setVelocity(0);
    }

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

    public double getVelocity() {
        return motor.getVelocity();
    }

    public void setPower(double p) {
        motor.setPower(p);
    }

    public double getPower() {
        return motor.getPower();
    }

    public int getPos() {
        return motor.getCurrentPosition();
    }

    public DcMotor getMotor() {
        return motor;
    }
}

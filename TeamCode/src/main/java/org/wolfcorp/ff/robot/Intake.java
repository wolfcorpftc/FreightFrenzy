package org.wolfcorp.ff.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.wolfcorp.ff.opmode.OpMode;

public class Intake {
    public static double  TICKS_PER_REV = 103.8;
    public static int INTAKE_MAX_SPEED = 1620;
    public static int INTAKE_OPTIMAL_SPEED = (int) (800 * 0.8);

    private DcMotorEx motor;

    public Intake(HardwareMap hwMap) {
        motor = hwMap.get(DcMotorEx.class, "intake");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void in() {
        if (isOn()) {
            off();
        }
        else {
            motor.setVelocity(INTAKE_OPTIMAL_SPEED / 60.0 * TICKS_PER_REV);
        }
    }

    public void out() {
        if (isOn()) {
            off();
        }
        else {
            motor.setVelocity(-INTAKE_OPTIMAL_SPEED / 60.0 * TICKS_PER_REV);
        }
    }

    public void off() {
        motor.setVelocity(0);
    }

    public boolean isOn() {
        return Math.abs(motor.getVelocity()) > 10;
    }

    // IN RPM
    public void setVelocity(double vel) {
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
}

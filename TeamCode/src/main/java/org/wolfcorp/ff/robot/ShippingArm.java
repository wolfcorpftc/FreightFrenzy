package org.wolfcorp.ff.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.wolfcorp.ff.opmode.OpMode;

public class ShippingArm {
    private final DcMotorEx motor;
    private final Servo servo;
    private final Object motorModeLock = new Object();

    public static final double ARM_GEAR_RATIO = 2;
    public static final double ARM_TICKS_PER_REV = 1425.1;
    public static final double ARM_MAX_SPEED = 117 / 60.0 * ARM_TICKS_PER_REV; // ticks/sec
    public static final double ARM_HOLD_POSITION_SPEED = 1;
    public static final double ARM_IN_SPEED = -0.4 * ARM_MAX_SPEED - ARM_HOLD_POSITION_SPEED; // ticks/sec
    public static final double ARM_OUT_SPEED = 0.4 * ARM_MAX_SPEED + ARM_HOLD_POSITION_SPEED; // ticks/sec

    public static final int ARM_IN_POSITION = 0;
    public static final int ARM_OUT_POSITION = (int) (91.0 / 360 * ARM_TICKS_PER_REV * ARM_GEAR_RATIO);
    public static final int ARM_OUTERMOST_POSITION = (int) (230.0 / 360 * ARM_TICKS_PER_REV * ARM_GEAR_RATIO);

    public static final double CLAW_OPEN_POSITION = 0.69;
    public static final double CLAW_CLOSE_POSITION = 1;

    public ShippingArm(HardwareMap hwMap) {
        motor = hwMap.get(DcMotorEx.class, "armMotor");
        servo = hwMap.get(Servo.class, "armServo");

        motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setPower(0);
    }

    public void armInAsync(double multiplier) {
        motor.setTargetPosition(ARM_IN_POSITION);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setVelocity(ARM_OUT_SPEED * multiplier);
    }

    public void armInAsync() {
        armInAsync(1);
    }

    public void armOutAsync(double multiplier) {
        motor.setTargetPosition(ARM_OUT_POSITION);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setVelocity(ARM_OUT_SPEED * multiplier);
    }

    public void armOutAsync() {
        armOutAsync(1);
    }

    // TODO: create armTo() to unify the code
    public void armOutermostAsync(double multiplier) {
        synchronized (motorModeLock) {
            motor.setTargetPosition(ARM_OUTERMOST_POSITION);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setVelocity(ARM_OUT_SPEED * multiplier);
        }
    }

    public void armOutermostAsync() {
        armOutermostAsync(1);
    }

    public void armOutermost() {
        armOutermostAsync();
        while (motor.isBusy() && OpMode.isActive());
        synchronized (motorModeLock) {
            motor.setVelocity(0);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void setArmVelocity(double speed) {
        if (speed < 0 && motor.getCurrentPosition() < ARM_IN_POSITION) {
            return;
        }
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setVelocity(speed);
    }

    public void holdPosition() {
        holdPosition(true);
    }

    public void holdPosition(boolean condition) {
        if (condition) {
            motor.setVelocity(motor.getCurrentPosition() > ARM_OUT_POSITION ? -1 : 1);
        } else {
            motor.setVelocity(0);
        }
    }

    public void resetArm() {
        synchronized (motorModeLock) {
            motor.setVelocity(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void resetArmAsync() {
        new Thread(this::resetArm).start();
    }

    public void toggleClaw() {
        if (Math.abs(servo.getPosition() - CLAW_CLOSE_POSITION) < 0.01) {
            openClaw();
        } else {
            closeClaw();
        }
    }

    public void openClaw() {
        servo.setPosition(CLAW_OPEN_POSITION);
    }

    public void closeClaw() {
        servo.setPosition(CLAW_CLOSE_POSITION);
    }

    public DcMotorEx getMotor() {
        return motor;
    }

    public Servo getServo() {
        return servo;
    }
}

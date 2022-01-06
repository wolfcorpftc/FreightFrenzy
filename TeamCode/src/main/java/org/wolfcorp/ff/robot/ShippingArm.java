package org.wolfcorp.ff.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ShippingArm {
    private DcMotorEx motor;
    private Servo servo;
    private Object motorModeLock = new Object();

    public static final double ARM_TICKS_PER_REV = 1425.1;
    public static final double ARM_MAX_SPEED = 117 / 60.0 * ARM_TICKS_PER_REV; // ticks/sec
    public static final double ARM_IN_SPEED = -0.8 * ARM_MAX_SPEED; // ticks/sec
    public static final double ARM_OUT_SPEED = 0.2 * ARM_MAX_SPEED; // ticks/sec

    public static final int ARM_IN_POSITION = 0;
    public static final int ARM_OUT_POSITION = (int) (91.0 / 360 * ARM_TICKS_PER_REV);
    public static final int ARM_OUTERMOST_POSITION = (int) (230.0 / 360 * ARM_TICKS_PER_REV);

    public static final double CLAW_OPEN_POSITION = 1;
    public static final double CLAW_CLOSE_POSITION = 0.65;

    public ShippingArm(HardwareMap hwMap) {
        motor = hwMap.get(DcMotorEx.class, "armMotor");
        servo = hwMap.get(Servo.class, "armServo");

        motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0);
    }

    public void armInAsync() {
        motor.setTargetPosition(ARM_IN_POSITION);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setVelocity(ARM_OUT_SPEED);
    }

    public void armOutAsync() {
        motor.setTargetPosition(ARM_OUTERMOST_POSITION);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setVelocity(ARM_OUT_SPEED);
    }

    public void resetArm() {
        synchronized (motorModeLock) {
            motor.setVelocity(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

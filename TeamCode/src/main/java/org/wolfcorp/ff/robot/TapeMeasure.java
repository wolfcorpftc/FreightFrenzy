package org.wolfcorp.ff.robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.wolfcorp.ff.opmode.OpMode;

public class TapeMeasure {
    private static final double TAPE_PITCH_TICKS_PER_REV = 1425.1;
    public static final double TAPE_PITCH_MAX_SPEED = 117 / 60.0 * TAPE_PITCH_TICKS_PER_REV;
    private static final double TAPE_FORWARD_ROTATE = 0.81;

    private final Servo tapeRotate;
    private final DcMotorEx tapePitch;
    private final CRServo tapeSpool;

    public TapeMeasure(HardwareMap hwMap) {
        tapePitch = hwMap.get(DcMotorEx.class, "tapePitch");
        tapeRotate = hwMap.get(Servo.class, "tapeRotate");
        tapeSpool = hwMap.get(CRServo.class, "tapeSpool");

        tapePitch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tapePitch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tapePitch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tapePitch.setPower(0);
        tapeRotate.setPosition(TAPE_FORWARD_ROTATE);
        spoolTape(0);
    }

    public void rotateTape(double position) {
        tapeRotate.setPosition(Math.min(Math.max(position, 0),1));
    }

    public void rotateTapeIncrement(double increment) {
        rotateTape(tapeRotate.getPosition() + increment);
    }

    public void pitchTape(double speed) {
        tapePitch.setVelocity(TAPE_PITCH_MAX_SPEED * speed);
    }

    public void spoolTape(double speed) {
        tapeSpool.setPower(speed);
    }

    public DcMotorEx getMotor() {
        return tapePitch;
    }

    public CRServo getSpool() {
        return tapeSpool;
    }

    public Servo getRotate() {
        return tapeRotate;
    }
}

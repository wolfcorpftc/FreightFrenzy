package org.wolfcorp.ff.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Shovel {
    public static final int MARGIN_OF_ERROR = 10; // margin of error
    public static final int MARGIN_OF_ACCEPTANCE = 3; // margin of acceptance
    public static final int DRIFT_TIME_DELAY = 1000;
    public static final int TICKS = 150;

    private DcMotorEx motor;
    private int targetPos = 0;

    private ElapsedTime timer = new ElapsedTime();

    public Shovel(HardwareMap hwMap) {
        motor = hwMap.get(DcMotorEx.class, "shovel");
        motor.setPower(0);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setTargetPositionTolerance(MARGIN_OF_ACCEPTANCE);
    }

    public void setTargetPos(int pos) {
        targetPos = pos;
        timer.reset();
    }

    public int getCurrentPos() {
        return motor.getCurrentPosition();
    }
    public int getTargetPos() {
        return targetPos;
    }

    public void eliminateDrift() {
        if (Math.abs(motor.getCurrentPosition() - targetPos) > MARGIN_OF_ERROR &&
                timer.milliseconds() > DRIFT_TIME_DELAY) {
            motor.setTargetPosition(targetPos);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(0.2);
            while (motor.isBusy());
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void up() {
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(0.4);
        motor.setTargetPosition(motor.getCurrentPosition() + TICKS);
        while (motor.isBusy());
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void down() {
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(0.2);
        motor.setTargetPosition(motor.getCurrentPosition() - TICKS);
        while (motor.isBusy());
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setPower(double p) {
        motor.setPower(p);
    }

    public double getPower() {
        return motor.getPower();
    }
}

package org.wolfcorp.ff.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.wolfcorp.ff.opmode.OpMode;

public class Shovel {
    public static final int MARGIN_OF_ERROR = 10; // margin of error
    public static final int MARGIN_OF_ACCEPTANCE = 10; // margin of acceptance
    public static final int DRIFT_TIME_DELAY = 1000;
    public static final int TICKS = 150;
    public static final double TIMEOUT = 2000;

    private DcMotorEx motor;
    private int restPos = 0;
    private Thread stayStillThread = null;

    private ElapsedTime driftTimer = new ElapsedTime();
    private ElapsedTime moveTimer = new ElapsedTime();

    public Shovel(HardwareMap hwMap) {
        motor = hwMap.get(DcMotorEx.class, "shovel");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setRestPos(int pos) {
        restPos = pos;
        driftTimer.reset();
    }

    public void recordRestPos() {
        setRestPos(getCurrentPos());
    }

    public int getCurrentPos() {
        return motor.getCurrentPosition();
    }

    public int getRestPos() {
        return restPos;
    }

    /**
     * Make the shovel stay still (at restPos) and wait until
     * the motor reaches the position. Intended to be run from
     * within a loop (i.e., TeleOp).
     */
    public void eliminateDrift() {
        if (Math.abs(motor.getCurrentPosition() - restPos) > MARGIN_OF_ERROR &&
                driftTimer.milliseconds() > DRIFT_TIME_DELAY) {
            motor.setTargetPosition(restPos);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(0.2);
            // TODO: try to eliminate loop
            while (motor.isBusy() && !Thread.currentThread().isInterrupted());
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driftTimer.reset();
        }
    }

    /**
     * Make the shovel stay still (at restPos).
     * Intended for one-time use (call and forget about it).
     */
    public void stayStill() {
        // return if thread is already running
        if (stayStillThread != null && stayStillThread.isAlive()) {
            return;
        }

        stayStillThread = new Thread(() -> {
            driftTimer.reset();
            while (!Thread.interrupted()) {
                eliminateDrift();
            }
        });
        stayStillThread.start();
    }

    /**
     * Release the motor from the effect of stayStill()
     * and RUN_TO_POSITION
     */
    public void setFree() throws InterruptedException {
        if (stayStillThread != null && stayStillThread.isAlive()) {
            stayStillThread.interrupt();
            stayStillThread.join();
            OpMode.log("Waiting for stayStillThread to die... Done");
        }
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void up() {
        motor.setTargetPosition(motor.getCurrentPosition() + TICKS);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(0.2);
        moveTimer.reset();
        while (motor.isBusy() && moveTimer.milliseconds() < TIMEOUT);
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void down() throws InterruptedException {
        motor.setTargetPosition(motor.getCurrentPosition() - TICKS);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(0.1);
        moveTimer.reset();
        while (motor.isBusy() && moveTimer.milliseconds() < TIMEOUT);
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setPower(double p) {
        motor.setPower(p);
    }

    public double getPower() {
        return motor.getPower();
    }
}

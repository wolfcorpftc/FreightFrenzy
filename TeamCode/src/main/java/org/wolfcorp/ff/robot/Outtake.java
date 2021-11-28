package org.wolfcorp.ff.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.wolfcorp.ff.opmode.OpMode;
import org.wolfcorp.ff.vision.Barcode;

public class Outtake {
    public static double  TICKS_PER_REV = 1425.1;
    public static int OUTTAKE_MAX_SPEED = 117;
    public static int OUTTAKE_UP_SPEED = 40;
    public static int OUTTAKE_DOWN_SPEED = -40;

    public static int TOP_POSITION = 1000;
    public static int MID_POSITION = 500;
    public static int BOT_POSITION = 0;

    public static final int MARGIN_OF_ERROR = 10; // margin of error
    public static final int MARGIN_OF_ACCEPTANCE = 10; // margin of acceptance
    public static final int DRIFT_TIME_DELAY = 1000;
    public static final int TICKS = 100;
    public static final double TIMEOUT = 2000;

    private DcMotorEx motor;
    private int restPos = 0;
    private Thread stayStillThread = null;

    private ElapsedTime driftTimer = new ElapsedTime();
    private ElapsedTime moveTimer = new ElapsedTime();

    public Outtake(HardwareMap hwMap) {
        motor = hwMap.get(DcMotorEx.class, "outtake");
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
            motor.setPower(OUTTAKE_UP_SPEED);
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

    public boolean overextended() {
        return motor.getCurrentPosition() <= 0;
    }

    public void slide(boolean extend) {
        if (/*!overextended()*/ true) {
            double speed = 0;
            if (extend)
                speed = OUTTAKE_UP_SPEED;
            else
                speed = OUTTAKE_DOWN_SPEED;
            motor.setVelocity(speed / 60.0 * TICKS_PER_REV);
            recordRestPos();
        }
    }

    public void extend() {
        slide(true);
    }

    public void retract() {
        slide(false);
    }

    public void slideToLevel(Barcode barcode) {
        switch (barcode) {
            case TOP:
                runToPosition(TOP_POSITION);
                break;
            case MID:
                runToPosition(MID_POSITION);
                break;
            case BOT:
                runToPosition(BOT_POSITION);
            default:
                break;
        }
    }

    public void setInitialPos() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runToPosition(int position) {
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (motor.getCurrentPosition() < position) {
            motor.setVelocity(OUTTAKE_UP_SPEED);
        } else {
            motor.setVelocity(OUTTAKE_DOWN_SPEED);
        }
        while (motor.isBusy() && !Thread.interrupted());
        motor.setVelocity(0);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

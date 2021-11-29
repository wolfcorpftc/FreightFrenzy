package org.wolfcorp.ff.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.wolfcorp.ff.opmode.OpMode;
import org.wolfcorp.ff.vision.Barcode;

import java.util.concurrent.atomic.AtomicReference;

public class Outtake {
    public static double TICKS_PER_REV = 1425.1;
    public static int OUTTAKE_MAX_SPEED = 117;
    public static double OUTTAKE_UP_SPEED = 80 / 60.0 * TICKS_PER_REV;
    public static double OUTTAKE_DOWN_SPEED = -80 / 60.0 * TICKS_PER_REV;

    public static int TOP_POSITION = 2000;
    public static int MID_POSITION = 1100;
    public static int BOT_POSITION = 20;

    public static int MIN_POSITION = -100;
    public static int MAX_POSITION = 2100;

    public static double DUMP_IN_POSITION = 1.0;
    public static double DUMP_OUT_POSITION = 0.6;

    public static final int MARGIN_OF_ERROR = 10; // margin of error
    public static final int MARGIN_OF_ACCEPTANCE = 10; // margin of acceptance
    public static final int DRIFT_TIME_DELAY = 1000;
    public static final int TICKS = 100;
    public static final double DRIFT_TIMEOUT = 2000;

    private DcMotorEx motor;
    private Servo servo;

    private int restPos = 0;
    private AtomicReference<Boolean> stayStill = new AtomicReference<>(false);
    private Thread stayStillThread = null;

    private ElapsedTime driftDelayTimer = new ElapsedTime(); // between calls to eliminateDrift()
    private ElapsedTime driftTimer = new ElapsedTime(); // duration of eliminateDrift() loop

    private boolean isDumpOut = false;

    public Outtake(HardwareMap hwMap) {
        motor = hwMap.get(DcMotorEx.class, "outtake");
        servo = hwMap.get(Servo.class, "dump");

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setRestPos(int pos) {
        restPos = pos;
        driftDelayTimer.reset();
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
     * Make the slide stay still (at restPos) and wait until
     * the motor reaches the position. Intended to be run within TeleOp or a separate thread.
     */
    public void eliminateDrift() {
        if (stayStill.get()
                && Math.abs(motor.getCurrentPosition() - restPos) > MARGIN_OF_ERROR
                && driftDelayTimer.milliseconds() > DRIFT_TIME_DELAY) {
            motor.setTargetPosition(restPos);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setVelocity(OUTTAKE_UP_SPEED);
            // TODO: try to eliminate loop
            driftTimer.reset();
            while (motor.isBusy()
                    && stayStill.get()
                    && Math.abs(motor.getCurrentPosition() - restPos) > MARGIN_OF_ACCEPTANCE
                    && !Thread.currentThread().isInterrupted()
                    && driftTimer.milliseconds() < DRIFT_TIMEOUT);
            motor.setVelocity(0);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driftDelayTimer.reset();
        }
        else {
            motor.setVelocity(0);
        }
    }

    /**
     * Make the shovel stay still (at restPos).
     * Intended for one-time use (call and forget about it).
     */
    public void stayStill() {
        // return if thread is already running
        stayStill.set(true);
        if (stayStillThread != null && stayStillThread.isAlive()) {
            return;
        }
        stayStillThread = new Thread(() -> {
            driftDelayTimer.reset();
            while (!Thread.currentThread().isInterrupted()) {
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
        stayStill.set(false);
        if (stayStillThread != null && stayStillThread.isAlive()) {
            stayStillThread.interrupt();
            OpMode.log("Waiting for stayStillThread to die...");
            stayStillThread.join();
            OpMode.log("Waiting for stayStillThread to die... Done");
        }
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Set the slide to move. Intended for TeleOp.
     * @param extend whether to extend or retract the slide
     */
    public void slide(boolean extend) {
        boolean isOverextension = (extend && motor.getCurrentPosition() >= MAX_POSITION)
                || (!extend && motor.getCurrentPosition() <= MIN_POSITION);
        if (!isOverextension) {
            motor.setVelocity(extend ? OUTTAKE_UP_SPEED : OUTTAKE_DOWN_SPEED);
        }
        else {
            OpMode.log("Overextension");
            motor.setPower(0);
        }
    }

    /**
     * Move the slide outward. Intended for TeleOp.
     */
    public void extend() {
        slide(true);
    }

    /**
     * Move the slide inward. Intended for TeleOp.
     */
    public void retract() {
        slide(false);
    }

    /**
     * Move the slide to a hub level / tier
     * @param barcode destination hub level / tier
     */
    public void slideTo(Barcode barcode) {
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

    /**
     * Treat current encoder position as zero
     */
    public void setInitialPos() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Synchronously move the slide to an encoder position. Interrupt-aware.
     * @param position destination encoder position
     */
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

    public DcMotorEx getMotor() {
        return motor;
    }

    public void toggleDump() {
        if (isDumpOut) {
            dumpIn();
        }
        else {
            dumpOut();
        }
    }

    public void dumpIn() {
        isDumpOut = false;
        servo.setPosition(DUMP_IN_POSITION);
    }

    public void dumpOut() {
        isDumpOut = true;
        servo.setPosition(DUMP_OUT_POSITION);
    }

    public boolean isDumpOut() {
        return isDumpOut;
    }

    public Servo getServo() {
        return servo;
    }
}

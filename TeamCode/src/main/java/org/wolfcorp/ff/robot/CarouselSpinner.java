package org.wolfcorp.ff.robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.wolfcorp.ff.opmode.util.Match;
import org.wolfcorp.ff.opmode.OpMode;

import java.util.function.Consumer;

public class CarouselSpinner {
    public static final double SPIN_TIME = 2600; // millis
    public static final Long WAIT_TIME = 1000L; // millis
    public static final double TURN_POWER = 1.0;
    public static Thread spinThread = null;

    private final CRServo servo;
    private final ElapsedTime spinTimer = new ElapsedTime();
    private final Consumer<Long> sleep;

    public CarouselSpinner(HardwareMap hwMap, Consumer<Long> s) {
        servo = hwMap.get(CRServo.class, "spinner");
        sleep = s;
        if (Match.RED) {
            servo.setDirection(CRServo.Direction.REVERSE);
        } else {
            servo.setDirection(CRServo.Direction.FORWARD);
        }
    }

    /**
     * Returns whether the carousel spinner is on.
     *
     * @return whether the carousel spinner is on
     */
    public boolean isOn() {
        return servo.getPower() != 0;
    }

    /**
     * Turns the carousel spinner on.
     *
     * @see #TURN_POWER
     */
    public void on() {
        servo.setPower(TURN_POWER);
    }

    /**
     * Turns the carousel spinner off.
     */
    public void off() {
        servo.setPower(0);
    }

    /**
     * Spins the carousel for the specified amount of times with waits in between.
     *
     * @param times how many times the motor should spin
     * @see #WAIT_TIME
     */
    public Thread spinAsync(int times, double spinTime, long waitTime) {
        Runnable runnable = () -> {
            for (int i = times; i >= 1 && !Thread.interrupted(); i--) {
                spinTimer.reset();
                on();
                OpMode.dumpIndicator.full();
                while (spinTimer.milliseconds() < spinTime && !Thread.currentThread().isInterrupted());
                off();
                OpMode.dumpIndicator.overflow();
                if (i != 1) {
                    sleep.accept(waitTime);
                }
            }
        };

        spinThread = new Thread(runnable);
        spinThread.start();
        return spinThread;
    }

    /**
     * Spins the carousel once.
     *
     * @see #spin(int)
     */

    public void spin() throws InterruptedException {
        spinAsync(1, 1.2 * SPIN_TIME, WAIT_TIME).join();
    }

    public void spin(int time) throws InterruptedException {
        spinAsync(time, SPIN_TIME, WAIT_TIME).join();
    }

    public void stopSpin() {
        if (spinThread != null && !spinThread.isInterrupted()) {
            spinThread.interrupt();
        }
    }

    public CRServo getServo() {
        return servo;
    }
}

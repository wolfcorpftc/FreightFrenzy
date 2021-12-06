package org.wolfcorp.ff.robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.wolfcorp.ff.opmode.Match;

import java.util.function.Consumer;

public class CarouselSpinner {
    public static final double SPIN_TIME = 3500; // millis
    public static final Long WAIT_TIME = 1000L; // millis
    public static final double TURN_POWER = 1;

    private final CRServo servo;
    private final ElapsedTime spinTimer = new ElapsedTime();
    private final Consumer<Long> sleep;

    public CarouselSpinner(HardwareMap hwMap, Consumer<Long> s) {
        servo = hwMap.get(CRServo.class, "spinner");
        sleep = s;
        if (Match.isRed) {
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
    public void spin(int times) {
        for (int i = times; i >= 1; i--) {
            spinTimer.reset();
            on();
            while (spinTimer.milliseconds() < SPIN_TIME && !Thread.currentThread().isInterrupted())
                ;
            off();
            if (i != 1) {
                sleep.accept(WAIT_TIME);
            }
        }
    }

    /**
     * Spins the carousel once.
     *
     * @see #spin(int)
     */
    public void spin() {
        spin(1);
    }
}

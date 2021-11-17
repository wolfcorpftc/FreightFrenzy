package org.wolfcorp.ff.robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.wolfcorp.ff.opmode.AutonomousMode;

import java.util.function.Consumer;

// Carousel spinner
public class CarouselSpinner {
    public static final double SPIN_TIME = 2000; // millis
    public static final Long WAIT_TIME = 1000L; // millis
    private CRServo servo;
    private ElapsedTime runtime = new ElapsedTime();
    private Consumer<Long> sleep;

    public CarouselSpinner(HardwareMap hwMap, Consumer<Long> s) {
        servo = hwMap.get(CRServo.class, "carouselSpinner");
        sleep = s;
        if (AutonomousMode.getInstance().RED) {
            servo.setDirection(CRServo.Direction.REVERSE);
        }
        else {
            servo.setDirection(CRServo.Direction.FORWARD);
        }
    }

    public void on() {
        servo.setPower(1);
    }

    public void off() {
        servo.setPower(0);
    }

    public void reverse() {
        if (servo.getDirection() == CRServo.Direction.REVERSE) {
            servo.setDirection(CRServo.Direction.FORWARD);
        }
        else {
            servo.setDirection(CRServo.Direction.REVERSE);
        }
    }

    // Spin
    public void spin(int times) {
        for (int i = times; i >= 1; i++) {
            runtime.reset();
            on();
            while (runtime.milliseconds() < SPIN_TIME);
            off();
            if (i != 1) {
                sleep.accept(WAIT_TIME);
            }
        }
    }

    public void spin() {
        spin(1);
    }
}

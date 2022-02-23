package org.wolfcorp.ff.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.wolfcorp.ff.opmode.util.Match;
import org.wolfcorp.ff.robot.CarouselSpinner;

@TeleOp(name = "Carousel Spinner Test", group = "^testing")
public class CarouselSpinnerTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Match.RED = true;
        CarouselSpinner redSpinner = new CarouselSpinner(hardwareMap, this::sleep);
        Match.RED = false;
        CarouselSpinner blueSpinner = new CarouselSpinner(hardwareMap, this::sleep);
        waitForStart();
        while (opModeIsActive()) {
            // x blue b red
            if (gamepad1.x) {
                if (blueSpinner.isOn()) {
                    blueSpinner.off();
                } else {
                    blueSpinner.on();
                }
            } else if (gamepad1.b) {
                if (redSpinner.isOn()) {
                    redSpinner.off();
                } else {
                    redSpinner.on();
                }
            }
        }
    }
}

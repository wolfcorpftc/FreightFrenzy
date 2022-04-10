package org.wolfcorp.ff.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Servo Port Test", group = "!test")
public class ServoPortTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo = hardwareMap.get(Servo.class, "s");
        waitForStart();
        while (opModeIsActive()) {
            servo.setPosition(0.3);
            sleep(3000);
            servo.setPosition(0.5);
            sleep(3000);
        }
    }
}

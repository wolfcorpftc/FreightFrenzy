package org.wolfcorp.ff.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;

import org.wolfcorp.ff.opmode.Match;
import org.wolfcorp.ff.robot.Outtake;

@Autonomous(name = "Dump Test", group = "test")
public class DumpTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Match.setupTelemetry();
        Servo servo = hardwareMap.get(Servo.class, "dump");

        waitForStart();

        for (int i = 0; i < 1; i++) {
            servo.setPosition(Outtake.DUMP_IN_POSITION);
            waitFor(1);
            servo.setPosition(Outtake.DUMP_OUT_POSITION);
            waitFor(1);
        }
    }

    public void waitFor(double seconds) {
        ElapsedTime timer = new ElapsedTime();
        while (timer.seconds() < seconds && opModeIsActive());
    }
}

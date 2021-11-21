package org.wolfcorp.ff.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.wolfcorp.ff.opmode.AutonomousMode;
import org.wolfcorp.ff.opmode.meet0.Meet0Auto;
import org.wolfcorp.ff.robot.Shovel;

@Autonomous(name = "Shovel Test", group = "test")
public class ShovelTest extends Meet0Auto {
    @Override
    public void runOpMode() throws InterruptedException {
        log("Initializing robot");
        shovel = new Shovel(hardwareMap);

        log("Robot initialized, waiting for start");
        waitForStart();

        log("Request shovel to stay still");
        shovel.setPower(0);

        log("Wait 5 seconds");
        waitFor(5);

        log("Shovel down");
        shovel.down();

        log("Wait 5 seconds");
        waitFor(5);

        log("Shovel up");
        shovel.up();

        log("Done");
    }

    public void waitFor(double seconds) {
        ElapsedTime timer = new ElapsedTime();
        while (timer.seconds() < seconds);
    }
}

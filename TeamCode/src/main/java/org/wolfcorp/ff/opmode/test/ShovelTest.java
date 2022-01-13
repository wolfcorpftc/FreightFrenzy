package org.wolfcorp.ff.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.wolfcorp.ff.opmode.util.Match;
import org.wolfcorp.ff.opmode.meet0.Meet0Auto;
import org.wolfcorp.ff.robot.Shovel;

@Autonomous(name = "Shovel Test", group = "test")
public class ShovelTest extends Meet0Auto {
    @Override
    public void runOpMode() {
        Match.setupTelemetry();
        Match.status("Initializing robot");
        shovel = new Shovel(hardwareMap);

        Match.status("Robot initialized, waiting for start");
        waitForStart();

        Match.status("Request shovel to stay still");
        shovel.setPower(0);

        Match.status("Wait 5 seconds");
        waitFor(5);

        Match.status("Shovel down");
        shovel.down();

        Match.status("Wait 5 seconds");
        waitFor(5);

        Match.status("Shovel up");
        shovel.up();

        Match.status("Done");
    }

    public void waitFor(double seconds) {
        ElapsedTime timer = new ElapsedTime();
        while (timer.seconds() < seconds);
    }
}

package org.wolfcorp.ff.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.wolfcorp.ff.opmode.AutonomousMode;
import org.wolfcorp.ff.opmode.Match;

@Autonomous(name = "Guide Vision Test", group = "test")
public class GuideAuto extends AutonomousMode {
    @Override
    public void runOpMode() {
        Match.setupTelemetry();
        initVisionWebcam();
        guide.start();
        waitForStart();
        while (opModeIsActive()) {
            // Guide annotates the stream
            idle();
            if (Thread.interrupted()) {
                guide.stop();
                break;
            }
        }
    }
}

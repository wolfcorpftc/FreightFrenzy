package org.wolfcorp.ff.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.wolfcorp.ff.opmode.AutonomousMode;

@Autonomous(name = "Guide Vision Test", group = "test")
public class NQGuideAuto extends AutonomousMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initVision();
        guide.start();
        while (opModeIsActive()) {
            // Guide annotates the stream
            idle();
            if (Thread.interrupted()) {
                guide.stop();
                throw new InterruptedException();
            }
        }
    }
}
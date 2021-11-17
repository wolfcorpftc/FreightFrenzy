package org.wolfcorp.ff.opmode.meet0;

import org.wolfcorp.ff.opmode.AutonomousMode;
import org.wolfcorp.ff.opmode.Match;
import org.wolfcorp.ff.robot.CarouselSpinner;
import org.wolfcorp.ff.robot.DriveConstants;
import org.wolfcorp.ff.robot.Drivetrain;
import org.wolfcorp.ff.robot.Shovel;

public class Meet0Auto extends AutonomousMode {
    protected Shovel shovel = null;

    @Override
    public void runOpMode() throws InterruptedException {
        // *** Initialization ***
        drive = new Drivetrain(hardwareMap);
        drive.setPoseEstimate(initialPose);

        shovel = new Shovel(hardwareMap);
        spinner = new CarouselSpinner(hardwareMap, this::sleep);

        log("Robot Initialized, preparing task queue");

        // *** Carousel ***
        if (CAROUSEL) {
            queue(fromHere().lineToLinearHeading(carouselPose));
            queue(() -> {
                drive.setPoseEstimate(pos(-54, -72 + DriveConstants.WIDTH / 2, 90));
                spinner.spin();
            });
        }

        // *** Score pre-loaded cube ***
        queue(fromHere().lineToLinearHeading(initialPose).lineTo(hubPose.vec()));
        queue(() -> {
            shovel.down();
            sleep(500);
            shovel.up();
        });

        // *** Park ***
        queue(fromHere().lineTo(whPose.vec()).lineTo(parkPose.vec()));

        // *** START ***
        waitForStart();
        runTasks();
        sleep(1000);
        Match.teleOpInitialPose = drive.getPoseEstimate();
        Match.hubPose = hubPose;

        // allow the current object to be GC'd
        instance = null;
    }
}

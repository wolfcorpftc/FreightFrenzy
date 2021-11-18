package org.wolfcorp.ff.opmode.meet0;

import com.acmerobotics.roadrunner.geometry.Pose2d;

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

        log("Robot initialized, preparing task queue");

        // *** Spin carousel & go to hub ***
        if (CAROUSEL) {
            log("Initializing: carousel");
            queue(fromHere().lineToLinearHeading(fakePreCarouselPose));
            queue(() -> drive.setPoseEstimate(preCarouselPose));
            queue(from(preCarouselPose).lineTo(carouselPose.vec()));
            queue((Runnable) spinner::spin);
        }
        else {
            // Compensation for lack of error
            hubPose = hubPose.plus(pos(2, 0));
        }

        // *** Score pre-loaded cube ***
        log("Initializing: hub & score");
        queue(fromHere().lineToLinearHeading(hubPose));
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

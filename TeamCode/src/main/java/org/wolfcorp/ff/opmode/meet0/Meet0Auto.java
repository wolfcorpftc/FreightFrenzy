package org.wolfcorp.ff.opmode.meet0;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.wolfcorp.ff.opmode.AutonomousMode;
import org.wolfcorp.ff.opmode.Match;
import org.wolfcorp.ff.robot.CarouselSpinner;
import org.wolfcorp.ff.robot.DriveConstants;
import org.wolfcorp.ff.robot.Drivetrain;
import org.wolfcorp.ff.robot.Shovel;

public abstract class Meet0Auto extends AutonomousMode {
    protected Shovel shovel = null;
    protected Pose2d preWhPose;

    public Meet0Auto() {
        // shovel is short and in the back
        hubPose = pos(-48 + DriveConstants.LENGTH / 2, -12, 90);
        preWhPose = pos(-72 + DriveConstants.WIDTH / 2, 12, 180);
        parkPose = whPose;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        log("Meet 0 runOpMode; initializing robot");
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
        log("Initializing: park");
        queue(fromHere().lineToLinearHeading(preWhPose).lineTo(parkPose.vec()));

        // *** START ***
        log("Task queue initialized, waiting for start");
        waitForStart();
        runTasks();
        log("All tasks done");

        sleep(1000);
        Match.teleOpInitialPose = drive.getPoseEstimate();
        Match.hubPose = hubPose;

        // allow the current object to be GC'd
        instance = null;
    }
}

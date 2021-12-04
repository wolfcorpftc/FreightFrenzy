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
        hubPose = pos(-46 + DriveConstants.LENGTH / 2, -12, 90);
        preWhPose = pos(-72 + DriveConstants.WIDTH / 2, 12, 180);
        parkPose = whPose;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Match.status("Meet 0 runOpMode; initializing robot");
        // *** Initialization ***
        drive = new Drivetrain(hardwareMap);
        drive.setPoseEstimate(initialPose);

        shovel = new Shovel(hardwareMap);
        spinner = new CarouselSpinner(hardwareMap, this::sleep);

        Match.status("Robot initialized, preparing task queue");

//        queue(shovel::stayStill);

        // *** Spin carousel & go to hub ***
        if (CAROUSEL) {
            Match.status("Initializing: carousel");
            queue(fromHere().lineToLinearHeading(calibratePreCarouselPose));
            //queue(fromHere().lineToLinearHeading(preCarouselPose));
            // TODO: replace one axis only
            queue(() -> drive.setPoseEstimate(preCarouselPose));
            queue(from(preCarouselPose).lineTo(carouselPose.vec()));
            queue((Runnable) spinner::spin);
        }
        else {
            // Compensation for lack of error
            hubPose = hubPose.plus(pos(2, 0));
        }

        // *** Score pre-loaded cube ***
        Match.status("Initializing: hub & score");
        queue(fromHere().lineToLinearHeading(hubPose));
        queue(() -> {
            try {
                shovel.down();
                sleep(250);
                shovel.up();
            } catch (InterruptedException e) {
                // FIXME: properly handle (define custom functional interface that throws)
                e.printStackTrace();
            }
        });

        // *** Park ***
        Match.status("Initializing: park");
        queue(fromHere().splineToSplineHeading(preWhPose, deg(0)).lineTo(parkPose.vec()));

        // *** START ***
        Match.status("Task queue initialized, waiting for start");
        waitForStart();
        runTasks();
        Match.status("All tasks done");

        sleep(1000);
        Match.teleOpInitialPose = drive.getPoseEstimate();
        Match.hubPose = hubPose;
    }
}
